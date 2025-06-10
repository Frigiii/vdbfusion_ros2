// MIT License
//
// # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "vdbfusion/VDBVolume.h"

// OpenVDB
#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

namespace {

float ComputeSDF(const Eigen::Vector3d& origin, const Eigen::Vector3d& point,
                 const Eigen::Vector3d& voxel_center) {
  const Eigen::Vector3d v_voxel_origin = voxel_center - origin;
  const Eigen::Vector3d v_point_voxel = point - voxel_center;
  const double dist = v_point_voxel.norm();
  const double proj = v_voxel_origin.dot(v_point_voxel);
  const double sign = proj / std::abs(proj);
  return static_cast<float>(sign * dist);
}

Eigen::Vector3d GetVoxelCenter(const openvdb::Coord& voxel,
                               const openvdb::math::Transform& xform) {
  const float voxel_size = xform.voxelSize()[0];
  openvdb::math::Vec3d v_wf = xform.indexToWorld(voxel) + voxel_size / 2.0;
  return Eigen::Vector3d(v_wf.x(), v_wf.y(), v_wf.z());
}

}  // namespace

namespace vdbfusion {

VDBVolume::VDBVolume(float voxel_size, float sdf_trunc,
                     bool space_carving /* = false*/,
                     float min_var /* = 100.0f*/, float var_punish /* = 0.0f*/,
                     float tsdf_punish /* = 0.0f*/)
    : voxel_size_(voxel_size),
      sdf_trunc_(sdf_trunc),
      space_carving_(space_carving),
      min_var_(min_var),
      var_punish_(var_punish),
      tsdf_punish_(tsdf_punish) {
  tsdf_ = openvdb::FloatGrid::create(sdf_trunc_);
  tsdf_->setName("D(x): signed distance grid");
  tsdf_->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_));
  tsdf_->setGridClass(openvdb::GRID_LEVEL_SET);

  variance_ = openvdb::FloatGrid::create(100.0f);
  variance_->setName("var(x): variance grid");
  variance_->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_));
  variance_->setGridClass(openvdb::GRID_UNKNOWN);

  updated_ = openvdb::BoolGrid::create(false);
  updated_->setName("Updated Voxels");
  updated_->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_));
  updated_->setGridClass(openvdb::GRID_UNKNOWN);
}

void VDBVolume::UpdateTSDF(
    const float& sdf, const openvdb::Coord& voxel,
    const std::function<float(float)>& variance_function) {
  using AccessorRW = openvdb::tree::ValueAccessorRW<openvdb::FloatTree>;
  if (sdf > -sdf_trunc_) {
    AccessorRW tsdf_acc = AccessorRW(tsdf_->tree());
    AccessorRW variance_acc = AccessorRW(variance_->tree());

    const float obs_tsdf = std::min(sdf_trunc_, sdf);
    const float obs_var = variance_function(sdf);
    const float prior_var = variance_acc.getValue(voxel);
    const float prior_tsdf = tsdf_acc.getValue(voxel);
    const float new_var = (1 / (1 / prior_var + 1 / obs_var));
    const float new_tsdf =
        (obs_var * prior_tsdf + prior_var * obs_tsdf) / (obs_var + prior_var);
    tsdf_acc.setValue(voxel, new_tsdf);
    variance_acc.setValue(voxel, std::max(min_var_, new_var));
  }
}

void VDBVolume::Integrate(
    openvdb::FloatGrid::Ptr grid,
    const std::function<float(float)>& variance_function) {
  for (auto iter = grid->cbeginValueOn(); iter.test(); ++iter) {
    const auto& sdf = iter.getValue();
    const auto& voxel = iter.getCoord();
    this->UpdateTSDF(sdf, voxel, variance_function);
  }
}

void VDBVolume::Integrate(
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
    const std::function<float(float)>& variance_function) {
  if (points.empty()) {
    std::cerr << "PointCloud provided is empty\n";
    return;
  }

  // Get some variables that are common to all rays
  const openvdb::math::Transform& xform = tsdf_->transform();
  const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());

  // Get the "unsafe" version of the grid acessors
  auto tsdf_acc = tsdf_->getAccessor();
  auto variance_acc = variance_->getAccessor();
  auto updated_acc = updated_->getAccessor();

  // Launch an for_each execution, use std::execution::par to parallelize this
  // region
  std::for_each(points.cbegin(), points.cend(), [&](const auto& point) {
    // Get the direction from the sensor origin to the point and normalize it
    const Eigen::Vector3d direction = point - origin;
    openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
    dir.normalize();

    // Truncate the Ray before and after the source unless space_carving_ is
    // specified.
    const auto depth = static_cast<float>(direction.norm());
    const float t0 = space_carving_ ? 0.0f : depth - sdf_trunc_;
    const float t1 = depth + sdf_trunc_;

    // Create one DDA per ray(per thread), the ray must operate on voxel grid
    // coordinates.
    const auto ray =
        openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf_);
    openvdb::math::DDA<decltype(ray)> dda(ray);
    do {
      const auto voxel = dda.voxel();
      const auto voxel_center = GetVoxelCenter(voxel, xform);
      const auto sdf = ComputeSDF(origin, point, voxel_center);
      if (sdf > -sdf_trunc_) {
        const float obs_tsdf = std::min(sdf_trunc_, sdf);
        const float obs_var = variance_function(sdf);
        const float prior_var = variance_acc.getValue(voxel);
        const float prior_tsdf = tsdf_acc.getValue(voxel);
        const float new_var = (1 / (1 / prior_var + 1 / obs_var));
        const float new_tsdf = (obs_var * prior_tsdf + prior_var * obs_tsdf) /
                               (obs_var + prior_var);
        tsdf_acc.setValue(voxel, new_tsdf);
        variance_acc.setValue(voxel, std::max(min_var_, new_var));
        updated_acc.setValue(voxel, true);
      }
    } while (dda.step());
  });
}

openvdb::FloatGrid::Ptr VDBVolume::Prune(float min_var) const {
  const auto vars = variance_->tree();
  const auto tsdf = tsdf_->tree();
  const auto background = sdf_trunc_;
  openvdb::FloatGrid::Ptr clean_tsdf = openvdb::FloatGrid::create(sdf_trunc_);
  clean_tsdf->setName("D(x): Pruned signed distance grid");
  clean_tsdf->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_));
  clean_tsdf->setGridClass(openvdb::GRID_LEVEL_SET);
  clean_tsdf->tree().combine2Extended(
      tsdf, vars, [=](openvdb::CombineArgs<float>& args) {
        if (args.aIsActive() && args.b() > min_var) {
          args.setResult(args.a());
          args.setResultIsActive(true);
        } else {
          args.setResult(background);
          args.setResultIsActive(false);
        }
      });
  return clean_tsdf;
}

void VDBVolume::PunishNotUpdatedVoxels() {
  // Punish the not updated voxels by increasing their tsdf value and increasing
  // their vars
  const float background = sdf_trunc_;
  const float var_punish = var_punish_;
  const float tsdf_punish = tsdf_punish_;

  // quick check if updated_, tsdf_ and variance_ are initialized
  if (!updated_ || !tsdf_ || !variance_) return;

  auto updated_acc = updated_->getAccessor();
  auto tsdf_acc = tsdf_->getAccessor();
  auto variance_acc = variance_->getAccessor();

  for (auto iter = tsdf_->beginValueOn(); iter.test(); ++iter) {
    const auto& voxel = iter.getCoord();
    if (!updated_acc.isValueOn(voxel)) {
      // Punish the TSDF values of the not updated voxel
      float new_tsdf_value = iter.getValue() + tsdf_punish;
      if (new_tsdf_value > background) {
        tsdf_acc.setValueOff(voxel);
      } else {
        tsdf_acc.setValue(voxel, new_tsdf_value);
      }
    } else {
      // Reset the updated voxel
      updated_acc.setValueOff(voxel);
    }
    float new_var = variance_acc.getValue(voxel) + var_punish;
    if (new_var > 100.0f) {
      variance_acc.setValueOff(voxel);
    } else {
      variance_acc.setValue(voxel, std::max(min_var_, new_var));
    }
  }
}

}  // namespace vdbfusion

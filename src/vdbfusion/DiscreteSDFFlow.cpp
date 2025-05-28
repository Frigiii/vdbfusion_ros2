#include "vdbfusion/DiscreteSDFFlow.h"

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

#include "openvdb/tools/FastSweeping.h"

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
void DiscreteSDFFlow::Integrate(
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
    const std::function<float(float)>& weighting_function) {
  // The plan is to create a one-shot TSDF of the point cloud, then create the
  // interpolated difference of that to tsdf_ (interpolate the surfaces), which
  // will give us a flow field estimate, and finally apply the flow field
  // estimate to tsdf_ and integrate the point cloud into tsdf_.

  // Create a one-shot TSDF from the points
  auto os_tsdf = CreateOneShotTSDF(points, origin, weighting_function);

  // Create a flow field from the one-shot TSDF
  std::shared_ptr<openvdb::FloatGrid> flow_field = GetSparseFlowField(os_tsdf);

  // Apply the flow field to the existing tsdf_
  ApplyFlowField(flow_field);

  // Integrate the point cloud into the existing tsdf_
  VDBVolume::Integrate(points, origin, [](float /* unused*/) { return 0.2; });
}

void DiscreteSDFFlow::Integrate(
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
    const std::function<float(float)>& weighting_function,
    std::shared_ptr<openvdb::FloatGrid> tsdf,
    std::shared_ptr<openvdb::FloatGrid> weights) {
  // Get some variables that are common to all rays
  const openvdb::math::Transform& xform = tsdf->transform();
  const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());

  // Get the "unsafe" version of the grid acessors
  auto tsdf_acc = tsdf->getAccessor();
  // auto weights_acc = weights->getAccessor();
  auto weights_acc = weights ? weights->getAccessor() : tsdf_acc;

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
        openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
    openvdb::math::DDA<decltype(ray)> dda(ray);
    do {
      const auto voxel = dda.voxel();
      const auto voxel_center = GetVoxelCenter(voxel, xform);
      const auto sdf = ComputeSDF(origin, point, voxel_center);
      if (sdf > -sdf_trunc_) {
        const float tsdf = std::min(sdf_trunc_, sdf);
        const float weight = weighting_function(sdf);
        const float last_weight = weights ? weights_acc.getValue(voxel) : 0.0f;
        const float last_tsdf = tsdf_acc.getValue(voxel);
        const float new_weight = weight + last_weight;
        const float new_tsdf =
            (last_tsdf * last_weight + tsdf * weight) / (new_weight);
        tsdf_acc.setValue(voxel, new_tsdf);
        if (weights) weights_acc.setValue(voxel, new_weight);
      }
    } while (dda.step());
  });
}

std::shared_ptr<openvdb::FloatGrid> DiscreteSDFFlow::GetSparseFlowField(
    std::shared_ptr<openvdb::FloatGrid> os_tsdf) {
  // create the the sparse difference of the one-shot tsdf (os_tsdf) to tsdf_

  // Get the accessors for the grids
  auto tsdf_acc = tsdf_->getAccessor();
  auto os_tsdf_acc = os_tsdf->getAccessor();

  // Create a new FloatGrid for the flow field with default value 0
  std::shared_ptr<openvdb::FloatGrid> flow_field =
      openvdb::FloatGrid::create(0);
  flow_field->setName("D(x,t)/dt : discrete SDF flow field");
  flow_field->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_));
  flow_field->setGridClass(openvdb::GRID_LEVEL_SET);
  auto flow_acc = flow_field->getAccessor();

  // Iterate over all voxels in the os_tsdf grid
  for (auto iter = os_tsdf->cbeginValueOn(); iter; ++iter) {
    const openvdb::Coord& voxel = iter.getCoord();

    // Check if the voxel is filled in the tsdf_ grid
    if (tsdf_acc.isValueOn(voxel)) {
      const float os_tsdf_value = iter.getValue();
      const float tsdf_value = tsdf_acc.getValue(voxel);
      // Set the flow field value as the difference between the one-shot TSDF
      // and tsdf_
      flow_acc.setValue(voxel, os_tsdf_value - tsdf_value);
    }
  }

  return flow_field;
}

}  // namespace vdbfusion
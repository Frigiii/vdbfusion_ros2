#include "vdbfusion/DiscreteSDFFlow.h"
#include "vdbfusion/VDBVolume.h"

// OpenVDB
#include <assert.h>
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

#include "openvdb/tools/GridOperators.h"
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
    const std::function<float(float)>& variance_function) {
  // The plan is to create a one-shot TSDF of the point cloud, then create
  // the interpolated difference of that to tsdf_ (interpolate the
  // surfaces), which will give us a flow field estimate, and finally apply
  // the flow field estimate to tsdf_ and integrate the point cloud into
  // tsdf_.

  if (points.empty()) {
    std::cerr << "Empty point cloud received." << std::endl;
    return;  // Skip if no points are provided
  }
  if (points.size() < 6) {
    std::cerr << "Not enough points to estimate scene flow. "
              << "Received " << points.size() << " points." << std::endl;
    // If we have less than 6 points, we cannot estimate a flow field
    // and we just integrate the points into the existing tsdf_.
    VDBVolume::Integrate(points, origin, variance_function);
    return;  // Skip if not enough points are provided
  }

  // Assume, we receive pcs at 10Hz
  const float dt = 0.1f;
  int min_num_nn = 10;  // Absolute minimum would be 6, with some more we get a
                        // smoother flow estimate
  min_num_nn = std::min(min_num_nn, static_cast<int>(points.size()));
  float downsample_radius = 0.5;

  // Create a one-shot TSDF from the points
  auto os_tsdf = CreateOneShotTSDF(points, origin, variance_function);
  assert(os_tsdf);

  // Create a flow field from the one-shot TSDF
  std::shared_ptr<openvdb::FloatGrid> dsdt = GetSparseSdfFlow(os_tsdf, dt);
  assert(dsdt);

  // Calculate the sdf gradient
  std::shared_ptr<openvdb::Vec3fGrid> grad_s = openvdb::tools::gradient(*tsdf_);
  assert(grad_s);

  // Estimate the sparse scene flow
  SdfFlowPointCloudPtr sparse_scene_flow =
      EstimateSparseSceneFlow(dsdt, grad_s, downsample_radius, min_num_nn);

  // // Apply the flow field to the existing tsdf_
  // ApplySparseSceneFlow(sparse_scene_flow);

  // Integrate the point cloud into the existing tsdf_
  VDBVolume::Integrate(points, origin, variance_function);
}

void DiscreteSDFFlow::Integrate(
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
    const std::function<float(float)>& variance_function,
    std::shared_ptr<openvdb::FloatGrid> tsdf,
    std::shared_ptr<openvdb::FloatGrid> variance, float integration_distance) {
  // Get some variables that are common to all rays
  const openvdb::math::Transform& xform = tsdf->transform();
  const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());

  // Get the grid acessors
  auto tsdf_acc = tsdf->getAccessor();
  auto variance_acc = variance ? variance->getAccessor() : tsdf_acc;
  // auto updated_acc = updated_->getAccessor();

  // Launch an for_each execution, use std::execution::par to parallelize
  // this region
  std::for_each(points.cbegin(), points.cend(), [&](const auto& point) {
    // Get the direction from the sensor origin to the point and normalize
    // it
    const Eigen::Vector3d direction = point - origin;
    openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
    dir.normalize();

    // Truncate the Ray before and after the source unless space_carving_ is
    // specified.
    const auto depth = static_cast<float>(direction.norm());
    const float t0 = (integration_distance < 0.0f)
                         ? space_carving_ ? 0.0f : depth - sdf_trunc_
                         : depth - integration_distance;
    const float t1 = (integration_distance < 0.0f)
                         ? depth + sdf_trunc_
                         : depth + integration_distance;

    // Create one DDA per ray(per thread), the ray must operate on voxel
    // grid coordinates.
    const auto ray =
        openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
    openvdb::math::DDA<decltype(ray)> dda(ray);
    do {
      const auto voxel = dda.voxel();
      const auto voxel_center = GetVoxelCenter(voxel, xform);
      const auto sdf = ComputeSDF(origin, point, voxel_center);
      if (sdf > -sdf_trunc_) {
        if (variance) {
          const float obs_tsdf = std::min(sdf_trunc_, sdf);
          const float obs_var = variance_function(sdf);
          const float prior_var = variance_acc.getValue(voxel);
          const float prior_tsdf = tsdf_acc.getValue(voxel);
          const float new_var = (1 / (1 / prior_var + 1 / obs_var));
          const float new_tsdf = (obs_var * prior_tsdf + prior_var * obs_tsdf) /
                                 (obs_var + prior_var);
          tsdf_acc.setValue(voxel, new_tsdf);
          variance_acc.setValue(voxel, std::max(min_var_, new_var));
        } else {
          // If variance is not provided, just update the TSDF
          tsdf_acc.setValue(voxel, std::min(sdf_trunc_, sdf));
        }
        // updated_acc.setValue(voxel, true);
      }
    } while (dda.step());
  });
}

std::shared_ptr<openvdb::FloatGrid> DiscreteSDFFlow::CreateOneShotTSDF(
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
    const std::function<float(float)>& variance_function) {
  std::shared_ptr<openvdb::FloatGrid> os_tsdf =
      openvdb::FloatGrid::create(sdf_trunc_);
  os_tsdf->setName("One-shot TSDF");
  os_tsdf->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_));
  os_tsdf->setGridClass(openvdb::GRID_LEVEL_SET);
  Integrate(points, origin, variance_function, os_tsdf, nullptr,
            voxel_size_ / 2.0f);
  return os_tsdf;
}

std::shared_ptr<openvdb::FloatGrid> DiscreteSDFFlow::GetSparseSdfFlow(
    std::shared_ptr<openvdb::FloatGrid> os_tsdf, const float dt) {
  // create the the sparse difference of the one-shot tsdf (os_tsdf) to
  // tsdf_ and divide the difference by dt, giving us the sdf flow

  // Get the accessors for the grids
  auto tsdf_acc = tsdf_->getAccessor();
  auto os_tsdf_acc = os_tsdf->getAccessor();

  // Create a new FloatGrid for the flow field with default value 0
  std::shared_ptr<openvdb::FloatGrid> sdf_flow_field =
      openvdb::FloatGrid::create(0);
  sdf_flow_field->setName("D(x,t)/dt : discrete SDF flow field");
  sdf_flow_field->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_));
  sdf_flow_field->setGridClass(openvdb::GRID_LEVEL_SET);
  auto flow_acc = sdf_flow_field->getAccessor();

  // Iterate over all voxels in the os_tsdf grid
  for (auto iter = os_tsdf->cbeginValueOn(); iter; ++iter) {
    const openvdb::Coord& voxel = iter.getCoord();

    // Check if the voxel is filled in the tsdf_ grid
    if (tsdf_acc.isValueOn(voxel)) {
      const float os_tsdf_value = iter.getValue();
      const float tsdf_value = tsdf_acc.getValue(voxel);
      // Set the flow field value as the difference between the one-shot
      // TSDF and tsdf_
      flow_acc.setValue(voxel, (os_tsdf_value - tsdf_value) / dt);
    }
  }

  return sdf_flow_field;
}

SdfFlowPointCloudPtr DiscreteSDFFlow::EstimateSparseSceneFlow(
    std::shared_ptr<openvdb::FloatGrid> dsdt,
    std::shared_ptr<openvdb::Vec3fGrid> grad_s, float downsample_radius,
    int min_num_nn) {
  // Use pcl to create a point cloud with our features. We'll use the kd-tree
  // implementation of pcl to find the nearest neighbors
  SdfFlowPointCloudPtr sparse_scene_flow =
      std::make_shared<SdfFlowPointCloud>();

  if (dsdt->activeVoxelCount() < static_cast<size_t>(min_num_nn)) {
    std::cerr << "Not enough active voxels in the sparse sdf flow field. "
              << "Required: " << min_num_nn
              << ", but got: " << dsdt->activeVoxelCount() << std::endl;
    return sparse_scene_flow;  // Return empty point cloud if not enough voxels
  }

  // Fill the point cloud with the flow field data
  for (auto iter = dsdt->cbeginValueOn(); iter; ++iter) {
    const openvdb::Coord& voxel = iter.getCoord();
    if (tsdf_->getAccessor().isValueOn(voxel)) {
      SdfFlowPoint point;

      // Convert the voxel coordinate to world coordinates
      const Eigen::Vector3d voxel_center =
          GetVoxelCenter(voxel, tsdf_->transform());

      point.x = voxel_center.x();
      point.y = voxel_center.y();
      point.z = voxel_center.z();
      point.dsdt = iter.getValue();  // Discrete SDF flow value

      // Get the normal vector from the gradient
      Eigen::Vector3d normal(grad_s->getAccessor().getValue(voxel).x(),
                             grad_s->getAccessor().getValue(voxel).y(),
                             grad_s->getAccessor().getValue(voxel).z());
      normal.normalize();  // Normalize the normal vector
      point.nx = normal.x();
      point.ny = normal.y();
      point.nz = normal.z();

      // Check if the point is valid
      assert(!(std::isnan(point.x) || std::isnan(point.y) ||
               std::isnan(point.z) || std::isnan(point.nx) ||
               std::isnan(point.ny) || std::isnan(point.nz) ||
               std::isnan(point.dsdt)));

      sparse_scene_flow->points.push_back(point);
    }
  }

  assert(!sparse_scene_flow->points.empty());

  // create a kd-tree for the flow field
  // set sorted to false to speed up radius search
  // (we don't need sorted results, just the nearest neighbors)
  pcl::KdTreeFLANN<SdfFlowPoint> kdtree;
  kdtree.setInputCloud(sparse_scene_flow);

  // get the min, max, mean coords
  SdfFlowPoint min_pt;
  SdfFlowPoint max_pt;
  pcl::getMinMax3D(*sparse_scene_flow, min_pt, max_pt);

  // For each point in the point cloud, find the nearest neighbors in the flow
  // field and estimate the scene flow at that point. Multiply the downsample
  // radius by 0.7 (/sqrt(2)) to ensure we capture all points
  for (float x_query = min_pt.x; x_query <= max_pt.x + downsample_radius * 0.7;
       x_query += downsample_radius * 0.7) {
    for (float y_query = min_pt.y;
         y_query <= max_pt.y + downsample_radius * 0.7;
         y_query += downsample_radius * 0.7) {
      for (float z_query = min_pt.z;
           z_query <= max_pt.z + downsample_radius * 0.7;
           z_query += downsample_radius * 0.7) {
        // Create a query point
        SdfFlowPoint query_point;
        query_point.x = x_query;
        query_point.y = y_query;
        query_point.z = z_query;

        // Find the nearest neighbors in the flow field
        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
        kdtree.radiusSearch(query_point, downsample_radius, nn_indices,
                            nn_dists);

        // If we don't have enough neighbors, skip this point
        size_t num_nn = nn_indices.size();
        if (num_nn < min_num_nn) {
          continue;  // Skip this point if not enough neighbors
        }

        // The scene flow is estimated by solving the following linear system
        // for w and v over num_nn neighbors using linear least squares: dsdt
        // = -a^T * [w, v]^T, where a = [x cross n, n]^T and [w, v] is the
        // angular and linear velocity vector.

        Eigen::MatrixXd A(num_nn, 6);
        Eigen::VectorXd b(num_nn);

        // Iterate over the nearest neighbors and fill the matrix A and vector
        // b
        for (int i = 0; i < num_nn; ++i) {
          const auto& neighbor = sparse_scene_flow->points[nn_indices[i]];
          Eigen::Vector3d x(neighbor.x, neighbor.y, neighbor.z);
          Eigen::Vector3d n(neighbor.nx, neighbor.ny, neighbor.nz);
          n.normalize();

          // Fill the matrix A and vector b
          A(i, 0) = x.y() * n.z() - x.z() * n.y();  // x cross n
          A(i, 1) = x.z() * n.x() - x.x() * n.z();
          A(i, 2) = x.x() * n.y() - x.y() * n.x();
          A(i, 3) = n.x();
          A(i, 4) = n.y();
          A(i, 5) = n.z();
          b(i) = -neighbor.dsdt;
        }

        // Solve the linear system A * [w, v]^T = b for [w, v]^T
        Eigen::VectorXd wv = A.colPivHouseholderQr().solve(b);
        if (wv.hasNaN()) {
          std::cerr << "Warning: NaN in scene flow estimation at point ("
                    << query_point.x << ", " << query_point.y << ", "
                    << query_point.z << ")." << std::endl;
          continue;  // Skip this point if the solution is invalid
        }

        // Store the estimated angular and linear velocities in the flow field
        SdfFlowPoint& updated_point = sparse_scene_flow->points[nn_indices[0]];
        updated_point.wx = wv(0);
        updated_point.wy = wv(1);
        updated_point.wz = wv(2);
        updated_point.vx = wv(3);
        updated_point.vy = wv(4);
        updated_point.vz = wv(5);

        // Update the point in the flow field
        sparse_scene_flow->points[nn_indices[0]] = updated_point;
      }
    }
  }

  this->setLatestSparseSceneFlow(sparse_scene_flow);

  return sparse_scene_flow;
}

void DiscreteSDFFlow::ApplySparseSceneFlow(
    std::shared_ptr<openvdb::FloatGrid> flow_field) {
  // Apply the flow field to the current tsdf_ grid
  using AccessorRW = openvdb::tree::ValueAccessorRW<openvdb::FloatTree>;
  AccessorRW tsdf_acc = AccessorRW(tsdf_->tree());
  AccessorRW flow_acc = AccessorRW(flow_field->tree());
  for (auto iter = flow_field->cbeginValueOn(); iter; ++iter) {
    const openvdb::Coord& voxel = iter.getCoord();
    if (tsdf_acc.isValueOn(voxel)) {
      // Update the tsdf_ value by adding the flow field value
      const float tsdf_value = tsdf_acc.getValue(voxel);
      const float flow_value = flow_acc.getValue(voxel);
      tsdf_acc.setValue(voxel, tsdf_value + flow_value);
    }
  }
}

}  // namespace vdbfusion
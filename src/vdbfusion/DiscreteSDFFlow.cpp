#include "vdbfusion/DiscreteSDFFlow.h"
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

#include "openvdb/tools/GridOperators.h"
// #include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
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

  // Assume, we receive pcs at 10Hz
  const float dt = 0.1f;  // time step in seconds
  const int num_nn = 10;  // number of nearest neighbors to consider

  // Create a one-shot TSDF from the points
  auto os_tsdf = CreateOneShotTSDF(points, origin, variance_function);

  // Create a flow field from the one-shot TSDF
  std::shared_ptr<openvdb::FloatGrid> ds = GetSparseFlowField(os_tsdf);

  // Calculate the sdf gradient
  std::shared_ptr<openvdb::Vec3fGrid> grad_s = openvdb::tools::gradient(*tsdf_);

  // Use pcl to create a point cloud with our features. We'll use the kd-tree
  // implementation of pcl to find the nearest neighbors
  pcl::PointCloud<SdfFlowPoint>::Ptr flow_field(
      new pcl::PointCloud<SdfFlowPoint>);

  // Fill the point cloud with the flow field data
  for (auto iter = ds->cbeginValueOn(); iter; ++iter) {
    const openvdb::Coord& voxel = iter.getCoord();
    if (tsdf_->getAccessor().isValueOn(voxel)) {
      SdfFlowPoint point;
      point.x = voxel.x();
      point.y = voxel.y();
      point.z = voxel.z();
      point.dsdt = iter.getValue() / dt;  // Discrete SDF flow value
      point.nx = grad_s->getAccessor().getValue(voxel).x();
      point.ny = grad_s->getAccessor().getValue(voxel).y();
      point.nz = grad_s->getAccessor().getValue(voxel).z();
      flow_field->points.push_back(point);
    }
  }

  // // create a kd-tree for the flow field
  // pcl::KdTreeFLANN<SdfFlowPoint> kdtree;
  // kdtree.setInputCloud(flow_field);

  // // For each point in the point cloud, find the nearest neighbors in the flow
  // // field and estimate the scene flow at that point
  // for (const auto& flow_point : flow_field->points) {
  //   // Create a query point
  //   SdfFlowPoint query_point;
  //   query_point.x = flow_point.x;
  //   query_point.y = flow_point.y;
  //   query_point.z = flow_point.z;

  //   // Find the nearest neighbors in the flow field
  //   std::vector<int> nn_indices(num_nn);
  //   std::vector<float> nn_dists(num_nn);
  //   kdtree.nearestKSearch(query_point, num_nn, nn_indices, nn_dists);

  //   // The scene flow is estimated by solving the following linear system for w
  //   // and v over num_nn neighbors using linear least squares:
  //   // dsdt = -a^T * [w, v]^T, where a = [x cross n, n]^T and [w, v] is the
  //   // angular and linear velocity vector.

  //   Eigen::MatrixXd A(num_nn, 6);
  //   Eigen::VectorXd b(num_nn);

  //   // Add the query point to the matrix A and vector b
  //   A(0, 0) = query_point.y * flow_point.nz -
  //             query_point.z * flow_point.ny;  // x cross n
  //   A(0, 1) = query_point.z * flow_point.nx - query_point.x * flow_point.nz;
  //   A(0, 2) = query_point.x * flow_point.ny - query_point.y * flow_point.nx;
  //   A(0, 3) = flow_point.nx;
  //   A(0, 4) = flow_point.ny;
  //   A(0, 5) = flow_point.nz;
  //   b(0) = -flow_point.dsdt;

  //   // Iterate over the nearest neighbors and fill the matrix A and vector b
  //   for (int i = 0; i < num_nn; ++i) {
  //     const auto& neighbor = flow_field->points[nn_indices[i]];
  //     Eigen::Vector3d x(neighbor.x, neighbor.y, neighbor.z);
  //     Eigen::Vector3d n(neighbor.nx, neighbor.ny, neighbor.nz);
  //     n.normalize();

  //     // Fill the matrix A and vector b
  //     A(i + 1, 0) = x.y() * n.z() - x.z() * n.y();  // x cross n
  //     A(i + 1, 1) = x.z() * n.x() - x.x() * n.z();
  //     A(i + 1, 2) = x.x() * n.y() - x.y() * n.x();
  //     A(i + 1, 3) = n.x();
  //     A(i + 1, 4) = n.y();
  //     A(i + 1, 5) = n.z();
  //     b(i + 1) = -neighbor.dsdt;
  //   }

  //   // Solve the linear system A * [w, v]^T = b for [w, v]^T
  //   Eigen::VectorXd wv = A.colPivHouseholderQr().solve(b);
  //   if (wv.hasNaN()) {
  //     std::cerr << "Warning: NaN in scene flow estimation at point ("
  //               << query_point.x << ", " << query_point.y << ", "
  //               << query_point.z << ")." << std::endl;
  //     continue;  // Skip this point if the solution is invalid
  //   }

  //   // Store the estimated angular and linear velocities in the flow field
  //   SdfFlowPoint& updated_point = flow_field->points[nn_indices[0]];
  //   updated_point.wx = wv(0);
  //   updated_point.wy = wv(1);
  //   updated_point.wz = wv(2);
  //   updated_point.vx = wv(3);
  //   updated_point.vy = wv(4);
  //   updated_point.vz = wv(5);

  //   // Update the point in the flow field
  //   flow_field->points[nn_indices[0]] = updated_point;
  // }

  // // Apply the flow field to the existing tsdf_
  // ApplyFlowField(flow_field);

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
  auto variance_acc = variance ? variance_->getAccessor() : tsdf_acc;
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
    const float t1 = depth + (integration_distance < 0.0f)
                         ? sdf_trunc_
                         : integration_distance;

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
        const float obs_tsdf = std::min(sdf_trunc_, sdf);
        const float obs_var = variance_function(sdf);
        const float prior_var = variance_acc.getValue(voxel);
        const float prior_tsdf = tsdf_acc.getValue(voxel);
        const float new_var = (1 / (1 / prior_var + 1 / obs_var));
        const float new_tsdf = (obs_var * prior_tsdf + prior_var * obs_tsdf) /
                               (obs_var + prior_var);
        tsdf_acc.setValue(voxel, new_tsdf);
        variance_acc.setValue(voxel, std::max(min_var_, new_var));
        // updated_acc.setValue(voxel, true);
      }
    } while (dda.step());
  });
}

std::shared_ptr<openvdb::FloatGrid> DiscreteSDFFlow::GetSparseFlowField(
    std::shared_ptr<openvdb::FloatGrid> os_tsdf) {
  // create the the sparse difference of the one-shot tsdf (os_tsdf) to
  // tsdf_

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
      // Set the flow field value as the difference between the one-shot
      // TSDF and tsdf_
      flow_acc.setValue(voxel, os_tsdf_value - tsdf_value);
    }
  }

  return flow_field;
}

}  // namespace vdbfusion
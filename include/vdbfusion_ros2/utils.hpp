#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vector>

#include "vdbfusion/VDBVolume.h"
#include "visualization_msgs/msg/marker.hpp"

namespace {
// Directly convert sensor_msgs/msg/PointCloud2 to std::vector<Eigen::Vector3>
// representation
std::vector<Eigen::Vector3d> pointCloudToEigen(
    const sensor_msgs::msg::PointCloud2& pcd) {
  std::vector<Eigen::Vector3d> points;
  const size_t point_step = pcd.point_step;
  const size_t num_points = pcd.width * pcd.height;
  points.reserve(num_points);
  const uint8_t* data = pcd.data.data();
  for (size_t i = 0; i < num_points; ++i) {
    const size_t offset = i * point_step;
    const float x = *reinterpret_cast<const float*>(data + offset);
    const float y =
        *reinterpret_cast<const float*>(data + offset + sizeof(float));
    const float z =
        *reinterpret_cast<const float*>(data + offset + 2 * sizeof(float));
    points.emplace_back(Eigen::Vector3d{x, y, z});
  }
  return points;
}

// Preprocess the scan by removing points that are outside the specified range
void preprocessScan(std::vector<Eigen::Vector3d>& scan, float min_range,
                    float max_range) {
  scan.erase(
      std::remove_if(
          scan.begin(), scan.end(),
          [&](auto p) { return p.norm() < min_range || p.norm() > max_range; }),
      scan.end());
}

// converts a VDBVolume to a visualization_msgs::msg::Marker message
visualization_msgs::msg::Marker vdbVolumeToCubeMarker(
    vdbfusion::VDBVolume& volume, const std_msgs::msg::Header& header,
    const float& min_weight) {
  auto marker = visualization_msgs::msg::Marker{};
  marker.header = header;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = volume.voxel_size_;
  // default color is green
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  auto weights_acc = volume.weights_->getConstAccessor();
  for (auto it = volume.tsdf_->beginValueOn(); it; ++it) {
    const openvdb::Coord& coord = it.getCoord();
    if (it.getValue() > 0.0f || weights_acc.getValue(coord) < min_weight) {
      continue;
    }
    geometry_msgs::msg::Point point;
    point.x = coord.x() * volume.voxel_size_;
    point.y = coord.y() * volume.voxel_size_;
    point.z = coord.z() * volume.voxel_size_;
    marker.points.push_back(point);
  }

  return marker;
}

// converts a VDBVolume to a visualization_msgs::msg::Marker message
visualization_msgs::msg::Marker vdbVolumeToMeshMarker(
    vdbfusion::VDBVolume& volume, const std_msgs::msg::Header& header,
    const bool& fill_holes, const float& min_weight) {
  auto marker = visualization_msgs::msg::Marker{};
  marker.header = header;
  marker.id = 0;
  marker.ns = "vdbfusion_mesh";
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  // default color is green
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  auto [vertices, triangles] =
      volume.ExtractTriangleMesh(fill_holes, min_weight);
  for (const auto& triangle : triangles) {
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::msg::Point point;
      point.x = vertices[triangle[i]].x();
      point.y = vertices[triangle[i]].y();
      point.z = vertices[triangle[i]].z();
      marker.points.push_back(point);
    }
  }

  return marker;
}

// Converts a VDBVolume to visualization_msgs::msg::Marker message
visualization_msgs::msg::Marker vdbVolumeVolumetoMeshMarker(
    vdbfusion::VDBVolume& volume, const std_msgs::msg::Header& header,
    const bool& fill_holes, const float& min_weight) {
  auto marker = visualization_msgs::msg::Marker{};
  marker.header = header;
  marker.id = 0;
  marker.ns = "vdbfusion_volume_mesh";
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  // default color is green
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  auto volume_ptr = volume.getVolumePtr();
  auto [vertices, triangles] =
      volume.ExtractTriangleMesh(fill_holes, min_weight, volume_ptr);
  for (const auto& triangle : triangles) {
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::msg::Point point;
      point.x = vertices[triangle[i]].x();
      point.y = vertices[triangle[i]].y();
      point.z = vertices[triangle[i]].z();
      marker.points.push_back(point);
    }
  }

  return marker;
}

}  // namespace
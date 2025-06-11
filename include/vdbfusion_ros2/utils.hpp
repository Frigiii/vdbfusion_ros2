#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vector>

#include "vdbfusion/DiscreteSDFFlow.h"
#include "vdbfusion/VDBVolume.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    const float& max_var) {
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

  auto variance_acc = volume.variance_->getConstAccessor();
  for (auto it = volume.tsdf_->beginValueOn(); it; ++it) {
    const openvdb::Coord& coord = it.getCoord();
    if (it.getValue() > 0.0f || variance_acc.getValue(coord) > max_var) {
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
    const bool& fill_holes, const float& max_var) {
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

  auto [vertices, triangles] = volume.ExtractTriangleMesh(fill_holes, max_var);
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
    const bool& fill_holes, const float& max_var, const float& iso_level) {
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
  marker.color.a = 0.5f;

  auto volume_ptr = volume.getVolumeExtractorVolume();
  auto [vertices, triangles] =
      volume.ExtractTriangleMesh(fill_holes, max_var, volume_ptr, iso_level);
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
visualization_msgs::msg::MarkerArray vdbVolumeToFlowMarker(
    vdbfusion::DiscreteSDFFlow& volume, const std_msgs::msg::Header& header,
    const float& max_var) {
  auto marker_array = visualization_msgs::msg::MarkerArray{};

  auto flow_field = volume.getLatestFlowField();

  if (!flow_field) {
    return marker_array;  // Return empty marker array if no flow field is
                          // available
  }

  // Create a marker for the flow field
  auto marker = visualization_msgs::msg::Marker{};
  marker.header = header;
  marker.ns = "vdbfusion_flow";

  // Clear previous markers
  marker.type = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);

  // Reset marker for flow arrows
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.a = 1.0f;  // Fully opaque
  marker.color.r = 1.0f;  // Red color for flow arrows
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.scale.z = 0.01;  // Arrow shaft diameter
  marker.scale.y = 0.01;  // Arrow head diameter

  for (auto& point : flow_field->points) {
    // Arrow length, scaled down for visibility
    marker.scale.x = std::sqrt(point.vx * point.vx + point.vy * point.vy +
                               point.vz * point.vz) /
                     20.0f;

    // Convert the velocity vector to a quaternion for orientation
    Eigen::Quaterniond orientation;
    if (marker.scale.x > 0.0) {
      Eigen::Vector3d direction(point.vx, point.vy, point.vz);
      direction.normalize();
      orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),
                                                       direction);
      marker.scale.x = std::min(marker.scale.x, 0.2);  // Limit max length
    } else {
      continue;  // Skip if the length is zero
    }

    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    // Add the marker to the marker array
    marker.id++;  // Increment ID for each marker
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

}  // namespace
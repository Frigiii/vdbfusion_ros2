// MIT License
// Copyright (c) 2025 Andre Pang
// Modified to work as a standalone ROS2 package
//
// Original Copyright Notice:
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

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vector>

#include "vdbfusion_ros2/vdbfusion_node.hpp"

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
}  // namespace

namespace vdbfusion {
vdbfusion_node::vdbfusion_node(const rclcpp::NodeOptions& options)
    : Node("vdbfusion_node", options) {
  initializeParameters();
  retrieveParameters();

  initializeVDBVolume();

  // tf2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "trimesh_self_filter/lidar_boom",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&vdbfusion_node::integratePointCloudCB, this,
                std::placeholders::_1));

  // publishers
  tsdf_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("output/tsdf", 10);
  mesh_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("output/mesh", 10);

  // publishing timers
  if (get_parameter("publish_tsdf").as_bool()) {
    auto publish_interval_ms = get_parameter("publish_interval_ms").as_int();
    tsdf_pub_timer_ =
        create_wall_timer(std::chrono::milliseconds(publish_interval_ms),
                          std::bind(&vdbfusion_node::tsdfTimerCB, this));
  }
  if (get_parameter("publish_mesh").as_bool()) {
    auto publish_interval_ms = get_parameter("publish_interval_ms").as_int();
    mesh_pub_timer_ =
        create_wall_timer(std::chrono::milliseconds(publish_interval_ms),
                          std::bind(&vdbfusion_node::meshTimerCB, this));
  }

  RCLCPP_INFO(get_logger(), "Successfully initialized vdbfusion_node");
}

void vdbfusion_node::initializeParameters() {
  declare_parameter("voxel_size", 0.05f);
  declare_parameter("truncation_distance", 0.15f);
  declare_parameter("space_carving", true);

  declare_parameter("preprocess", true);
  declare_parameter("apply_pose", true);
  declare_parameter("min_range", 0.0f);
  declare_parameter("max_range", 3.0f);

  declare_parameter("fill_holes", false);
  declare_parameter("min_weight", 0.0f);

  declare_parameter("timestamp_tolerance_ns", 10000);
  declare_parameter("static_frame_id", "world");

  declare_parameter("publish_interval_ms", 1000);
  declare_parameter("publish_tsdf", true);
  declare_parameter("publish_mesh", true);
}

void vdbfusion_node::retrieveParameters() {
  get_parameter("preprocess", preprocess_);
  get_parameter("apply_pose", apply_pose_);
  get_parameter("min_range", min_range_);
  get_parameter("max_range", max_range_);

  get_parameter("fill_holes", fill_holes_);
  get_parameter("min_weight", min_weight_);
  get_parameter("static_frame_id", static_frame_id_);

  int timestamp_tolerance_ns = 10000;
  get_parameter("timestamp_tolerance_ns", timestamp_tolerance_ns);
  timestamp_tolerance_ = rclcpp::Duration(0, timestamp_tolerance_ns);

  get_parameter("use_sim_time", this->use_sim_time_);
}

void vdbfusion_node::initializeVDBVolume() {
  float voxel_size, truncation_distance;
  bool space_carving;
  get_parameter("voxel_size", voxel_size);
  get_parameter("truncation_distance", truncation_distance);
  get_parameter("space_carving", space_carving);
  vdb_volume_ = std::make_shared<VDBVolume>(voxel_size, truncation_distance,
                                            space_carving);
}

void vdbfusion_node::integratePointCloudCB(
    const sensor_msgs::msg::PointCloud2::SharedPtr pcd_in) {
  geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::msg::PointCloud2 pcd_out = *pcd_in;
  auto timestamp = pcd_in->header.stamp;
  auto child_frame_id = pcd_in->header.frame_id;
  if (!child_frame_id.empty() && child_frame_id[0] == '/') {
    child_frame_id = child_frame_id.substr(1);
  }

  if (tf_buffer_->canTransform(static_frame_id_, child_frame_id, timestamp,
                               timestamp_tolerance_)) {
    RCLCPP_DEBUG(get_logger(), "Transform available from %s to %s",
                 static_frame_id_.c_str(), child_frame_id.c_str());

    if (use_sim_time_) {
      // If using simulation time, we need to capture the latest point cloud
      // header
      latest_pc_header_stamp_ = pcd_in->header.stamp;
    } else {
      // If not using simulation time, we can use the current time
      latest_pc_header_stamp_ = this->now();
    }

    auto transform = tf_buffer_->lookupTransform(
        static_frame_id_, child_frame_id, timestamp, timestamp_tolerance_);
    if (apply_pose_) {
      tf2::doTransform(*pcd_in, pcd_out, transform);
    }
    auto scan = pointCloudToEigen(pcd_out);
    if (preprocess_) {
      preprocessScan(scan, min_range_, max_range_);
    }
    const auto& x = transform.transform.translation.x;
    const auto& y = transform.transform.translation.y;
    const auto& z = transform.transform.translation.z;
    auto origin = Eigen::Vector3d{x, y, z};

    vdb_volume_->Integrate(scan, origin,
                           [](float /* unuused*/) { return 0.2; });
  }
}

void vdbfusion_node::tsdfTimerCB() { this->publishTSDF(); }

void vdbfusion_node::meshTimerCB() { this->publishMesh(); }

void vdbfusion_node::publishTSDF() {
  auto header = std_msgs::msg::Header{};
  header.stamp = latest_pc_header_stamp_;
  header.frame_id = static_frame_id_;
  auto tsdf_marker = vdbVolumeToCubeMarker(*vdb_volume_, header, min_weight_);

  tsdf_pub_->publish(tsdf_marker);
}

void vdbfusion_node::publishMesh() {
  auto header = std_msgs::msg::Header{};
  header.stamp = latest_pc_header_stamp_;
  header.frame_id = static_frame_id_;
  auto mesh_marker =
      vdbVolumeToMeshMarker(*vdb_volume_, header, fill_holes_, min_weight_);

  mesh_pub_->publish(mesh_marker);
}

}  // namespace vdbfusion
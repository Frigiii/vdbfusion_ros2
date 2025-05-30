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

#include "vdbfusion_ros2/utils.hpp"
#include "vdbfusion_ros2/vdbfusion_node.hpp"

namespace vdbfusion {
vdbfusion_node::vdbfusion_node(const rclcpp::NodeOptions& options)
    : Node("vdbfusion_node", options) {
  initializeParameters();
  retrieveParameters();

  initializeVDBVolume();

  // tf2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a callback group for each subscription to allow them to run in
  // separate threads. This way we have a thread for each point cloud input.
  auto subscription_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  for (const auto& topic : pointcloud_inputs_) {
    auto cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = cb_group;
    auto qos = rclcpp::SystemDefaultsQoS();
    auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos,
        std::bind(&vdbfusion_node::integratePointCloudCB, this,
                  std::placeholders::_1),
        options);
    pointcloud_subs_.push_back(sub);
    pointcloud_callback_groups_.push_back(cb_group);
  }

  // publishers
  tsdf_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      output_topic_ + "/tsdf", 10);
  mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      output_topic_ + "/mesh", 10);

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
  declare_parameter("pointcloud_inputs", std::vector<std::string>{"cloud_in"});
  declare_parameter("output_topic", "output");

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
  get_parameter("pointcloud_inputs", pointcloud_inputs_);
  get_parameter("output_topic", output_topic_);

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

  RCLCPP_INFO(get_logger(), "Parameters retrieved successfully:");
  RCLCPP_INFO(get_logger(), "   pointcloud_inputs:");
  for (const auto& topic : pointcloud_inputs_) {
    RCLCPP_INFO(get_logger(), "     - %s", topic.c_str());
  }
  RCLCPP_INFO(get_logger(), "   output_topic: %s", output_topic_.c_str());
  RCLCPP_INFO(get_logger(), "   voxel_size: %f",
              get_parameter("voxel_size").as_double());
  RCLCPP_INFO(get_logger(), "   truncation_distance: %f",
              get_parameter("truncation_distance").as_double());
  RCLCPP_INFO(get_logger(), "   space_carving: %s",
              get_parameter("space_carving").as_bool() ? "true" : "false");
  RCLCPP_INFO(get_logger(), "   preprocess: %s",
              preprocess_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "   apply_pose: %s",
              apply_pose_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "   min_range: %f", min_range_);
  RCLCPP_INFO(get_logger(), "   max_range: %f", max_range_);
  RCLCPP_INFO(get_logger(), "   fill_holes: %s",
              fill_holes_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "   min_weight: %f", min_weight_);
  RCLCPP_INFO(get_logger(), "   static_frame_id: %s", static_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "   timestamp_tolerance_ns: %ld ns",
              timestamp_tolerance_ns);
  RCLCPP_INFO(get_logger(), "   use_sim_time: %s",
              use_sim_time_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "   publish_interval_ms: %d",
              get_parameter("publish_interval_ms").as_int());
  RCLCPP_INFO(get_logger(), "   publish_tsdf: %s",
              get_parameter("publish_tsdf").as_bool() ? "true" : "false");
  RCLCPP_INFO(get_logger(), "   publish_mesh: %s",
              get_parameter("publish_mesh").as_bool() ? "true" : "false");
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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<vdbfusion::vdbfusion_node>(options);
  executor->add_node(node);
  executor->spin();
  return 0;
}
// MIT License
// Copyright (c) 2025 Andre Pang

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "rclcpp_components/register_node_macro.hpp"
#include "vdbfusion/VDBVolume.h"
#include "vdbfusion/DiscreteSDFFlow.h"

namespace vdbfusion {
// using VDBVolumeType = vdbfusion::DiscreteSDFFlow;
using VDBVolumeType =
    vdbfusion::DiscreteSDFFlow;  // Use VDBVolume for basic TSDF integration
class vdbfusion_node : public rclcpp::Node {
 public:
  vdbfusion_node(const rclcpp::NodeOptions& options);

 private:
  void initializeParameters() {
    declare_parameter("pointcloud_inputs",
                      std::vector<std::string>{"cloud_in"});
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
    declare_parameter("max_weight", 1.0f);

    declare_parameter("timestamp_tolerance_ns", 10000);
    declare_parameter("static_frame_id", "world");

    declare_parameter("publish_interval_ms", 1000);
    declare_parameter("publish_tsdf", true);
    declare_parameter("publish_mesh", true);
  }

  void retrieveParameters() {
    get_parameter("pointcloud_inputs", pointcloud_inputs_);
    get_parameter("output_topic", output_topic_);

    get_parameter("preprocess", preprocess_);
    get_parameter("apply_pose", apply_pose_);
    get_parameter("min_range", min_range_);
    get_parameter("max_range", max_range_);

    get_parameter("fill_holes", fill_holes_);
    get_parameter("min_weight", min_weight_);
    get_parameter("max_weight", max_weight_);
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
    RCLCPP_INFO(get_logger(), "   max_weight: %f", max_weight_);
    RCLCPP_INFO(get_logger(), "   static_frame_id: %s",
                static_frame_id_.c_str());
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

  void initializeVDBVolume();

  void integratePointCloudCB(
      const sensor_msgs::msg::PointCloud2::SharedPtr pcd_in);
  void tsdfTimerCB();
  void meshTimerCB();
  void volumeTimerCB();
  void punishNotUpdatedVoxelsCB();

  void publishVolumeMesh();
  void publishVolumeValue();

  void publishTSDF();
  void publishMesh();

 private:
  std::shared_ptr<VDBVolumeType> vdb_volume_;

  // subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      pointcloud_subs_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> pointcloud_callback_groups_;

  // timers
  rclcpp::TimerBase::SharedPtr tsdf_pub_timer_;
  rclcpp::TimerBase::SharedPtr mesh_pub_timer_;
  rclcpp::TimerBase::SharedPtr volume_pub_timer_;
  rclcpp::TimerBase::SharedPtr punish_not_updated_voxels_timer_;

  // publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tsdf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      volume_mesh_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr volume_val_pub_;

  // tf2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::Duration timestamp_tolerance_{0, 10000};  // 10ms
  std::string static_frame_id_;

  // parameters
  bool preprocess_;
  bool apply_pose_;
  float min_range_;
  float max_range_;

  bool fill_holes_;
  float max_var_;
  float min_var_;

  float iso_level_;

  std::vector<std::string> pointcloud_inputs_;
  std::string output_topic_;
  std::string boundary_mesh_path_;
  bool use_sim_time_;
  builtin_interfaces::msg::Time latest_pc_header_stamp_;
};
}  // namespace vdbfusion

RCLCPP_COMPONENTS_REGISTER_NODE(vdbfusion::vdbfusion_node)
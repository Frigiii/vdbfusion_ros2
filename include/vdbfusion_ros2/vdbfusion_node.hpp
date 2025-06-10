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

namespace vdbfusion {
class vdbfusion_node : public rclcpp::Node {
 public:
  vdbfusion_node(const rclcpp::NodeOptions& options);

 private:
  void initializeParameters();
  void retrieveParameters();
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
  std::shared_ptr<VDBVolume> vdb_volume_;

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
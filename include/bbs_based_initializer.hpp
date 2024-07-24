#pragma once

#include <iostream>
#include <thread>
#include <Eigen/Core>
#include <boost/filesystem.hpp>

// ros2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gpu_bbs3d/bbs3d.cuh>

class BbsBasedInitializer : public rclcpp::Node {
public:
  BbsBasedInitializer(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~BbsBasedInitializer();

private:
  bool load_config(const std::string& config);
  void localize_callback(const std_msgs::msg::Bool::SharedPtr msg);
  int get_nearest_imu_index(const std::vector<sensor_msgs::msg::Imu>& imu_buffer, const builtin_interfaces::msg::Time& stamp);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // sub
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // pub
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  bool is_map_loaded;

  // msg buffer
  sensor_msgs::msg::PointCloud2::SharedPtr source_cloud_msg_;
  std::vector<sensor_msgs::msg::Imu> imu_buffer;

  gpu::BBS3D gpu_bbs3d;

  // topic name
  std::string lidar_topic_name, map_topic_name, imu_topic_name;
  std::string trigger_topic_name, pose_topic_name;

  // 3D-BBS parameters
  double min_level_res;
  int max_level;

  // angular search range
  Eigen::Vector3d min_rpy;
  Eigen::Vector3d max_rpy;

  // score threshold percentage
  double score_threshold_percentage;

  // downsample
  float map_leaf_size, src_leaf_size;
  double min_scan_range, max_scan_range;

  // timeout
  int timeout_msec;
};

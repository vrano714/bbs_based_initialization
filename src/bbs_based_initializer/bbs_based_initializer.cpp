#include <bbs_based_initializer.hpp>

#include <pointcloud_iof/pcl_eigen_converter.hpp>
#include <pointcloud_iof/pcd_loader.hpp>
#include <pointcloud_iof/gravity_alignment.hpp>
#include <chrono>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <yaml-cpp/yaml.h>
#include <math.h>

Eigen::Vector3d to_eigen(const std::vector<double>& vec) {
  Eigen::Vector3d e_vec;
  for (int i = 0; i < 3; ++i) {
    if (vec[i] == 6.28) {
      e_vec(i) = 2 * M_PI;
    } else {
      e_vec(i) = vec[i];
    }
  }
  return e_vec;
}

BbsBasedInitializer::BbsBasedInitializer(const rclcpp::NodeOptions& node_options) : Node("bbs_based_initializer", node_options) {
  //  ==== Load config file ====
  std::cout << "[ROS2] Loading config file..." << std::endl;
  std::string config = this->declare_parameter<std::string>("config");
  if (!load_config(config)) {
    std::cout << "[ERROR] Loading config file failed" << std::endl;
  };

  // ==== ROS 2 sub ====
  trigger_sub_ =  this->create_subscription<std_msgs::msg::Bool>(
    trigger_topic_name,
    rclcpp::SensorDataQoS(),
    std::bind(&BbsBasedInitializer::localize_callback, this, std::placeholders::_1));
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
    std::bind(&BbsBasedInitializer::cloud_callback, this, std::placeholders::_1));

  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    map_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable(),
    std::bind(&BbsBasedInitializer::map_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, 100, std::bind(&BbsBasedInitializer::imu_callback, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_name, 10);


  // initialize map load status
  is_map_loaded = false;

}

BbsBasedInitializer::~BbsBasedInitializer() {}

void BbsBasedInitializer::localize_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  // map not loaded == initialization not done, so return
  if (!is_map_loaded) return;
  if (!msg->data) return;
  if (!source_cloud_msg_) {
    std::cout << "point cloud msg is not received" << std::endl;
    return;
  }

  if (!imu_buffer.size()) {
    std::cout << "imu msg is not received" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*source_cloud_msg_, *src_cloud);

  // filter
  if (src_leaf_size != 0.0f) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(src_leaf_size, src_leaf_size, src_leaf_size);
    filter.setInputCloud(src_cloud);
    filter.filter(*filtered_cloud_ptr);
    *src_cloud = *filtered_cloud_ptr;
  }

  // Cut scan range
  if (!(min_scan_range == 0.0 && max_scan_range == 0.0)) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < src_cloud->points.size(); ++i) {
      pcl::PointXYZ point = src_cloud->points[i];
      double norm = pcl::euclideanDistance(point, pcl::PointXYZ(0.0f, 0.0f, 0.0f));

      if (norm >= min_scan_range && norm <= max_scan_range) {
        cut_cloud_ptr->points.push_back(point);
      }
    }
    *src_cloud = *cut_cloud_ptr;
  }

  int imu_index = get_nearest_imu_index(imu_buffer, source_cloud_msg_->header.stamp);
  const auto imu_msg = imu_buffer[imu_index];
  const Eigen::Vector3d acc = {imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z};
  pcl::transformPointCloud(*src_cloud, *src_cloud, pciof::calc_gravity_alignment_matrix(acc.cast<float>()));

  std::vector<Eigen::Vector3f> src_points;
  pciof::pcl_to_eigen(src_cloud, src_points);
  gpu_bbs3d.set_src_points(src_points);

  std::cout << "[Localize] start" << std::endl;
  gpu_bbs3d.localize();  // gloal localization

  if (!gpu_bbs3d.has_localized()) {
    if (gpu_bbs3d.has_timed_out())
      std::cout << "[Failed] Localization timed out." << std::endl;
    else
      std::cout << "[Failed] Score is below the threshold." << std::endl;
    return;
  }

  std::cout << "[Localize] Execution time: " << gpu_bbs3d.get_elapsed_time() << "[msec] " << std::endl;
  std::cout << "[Localize] score: " << gpu_bbs3d.get_best_score() << std::endl;

  auto estimated_pose = gpu_bbs3d.get_global_pose();

  // generate ros message
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_to_pub(new geometry_msgs::msg::PoseWithCovarianceStamped);
  // PoseWithCovarianceStamped
  // - header
  // - pose (PoseWithCovariance)
  //   - pose (Pose)
  //     - position
  //     - orientation
  pose_to_pub->header.frame_id ="map";
  pose_to_pub->header.stamp = source_cloud_msg_->header.stamp; // timestamp of lidar topic
  pose_to_pub->pose.pose.position.x = estimated_pose(0, 3);
  pose_to_pub->pose.pose.position.y = estimated_pose(1, 3);
  pose_to_pub->pose.pose.position.z = estimated_pose(2, 3);
  Eigen::Quaternionf q(estimated_pose.block<3, 3>(0, 0));
  pose_to_pub->pose.pose.orientation.x = q.x();
  pose_to_pub->pose.pose.orientation.y = q.y();
  pose_to_pub->pose.pose.orientation.z = q.z();
  pose_to_pub->pose.pose.orientation.w = q.w();
  // FIXME: whether to initialize covariance?
  pose_pub_->publish(*pose_to_pub);

}

// get imu message which has closest timestamp with given timestamp
int BbsBasedInitializer::get_nearest_imu_index(const std::vector<sensor_msgs::msg::Imu>& imu_buffer, const builtin_interfaces::msg::Time& stamp) {
  int imu_index = 0;
  double min_diff = 1000;
  for (int i = 0; i < imu_buffer.size(); ++i) {
    double diff = std::abs(
      imu_buffer[i].header.stamp.sec + imu_buffer[i].header.stamp.nanosec * 1e-9 - source_cloud_msg_->header.stamp.sec -
      source_cloud_msg_->header.stamp.nanosec * 1e-9);
    if (diff < min_diff) {
      imu_index = i;
      min_diff = diff;
    }
  }
  return imu_index;
}

void BbsBasedInitializer::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!msg) return;
  source_cloud_msg_ = msg;
}

void BbsBasedInitializer::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::cout << "[BBS3D:init] initial map callback" << std::endl;

  if (is_map_loaded) { // do nothing if map is already loaded
    std::cout << "[BBS3D:init] map already loaded. do nothing" << std::endl;
    return;
  }

  // convert ROS message
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud_ptr);
  // do downsample when leaf size is provided
  if (map_leaf_size != 0.0f) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(map_leaf_size, map_leaf_size, map_leaf_size);
    filter.setInputCloud(cloud_ptr);
    filter.filter(*filtered_cloud_ptr);
    *cloud_ptr = *filtered_cloud_ptr;
  }

  // pcl to eigen
  std::vector<Eigen::Vector3f> map_points;
  pciof::pcl_to_eigen(cloud_ptr, map_points);

  std::cout << "[BBS3D:init] Creating hierarchical voxel map..." << std::endl;
  gpu_bbs3d.set_tar_points(map_points, min_level_res, max_level);
  gpu_bbs3d.set_trans_search_range(map_points);

  gpu_bbs3d.set_angular_search_range(min_rpy.cast<float>(), max_rpy.cast<float>());
  gpu_bbs3d.set_score_threshold_percentage(static_cast<float>(score_threshold_percentage));
  if (timeout_msec > 0) {
    gpu_bbs3d.enable_timeout();
    gpu_bbs3d.set_timeout_duration_in_msec(timeout_msec);
  }

  std::cout << "[BBS3D:init] READY!" << std::endl;

  is_map_loaded = true;
}

void BbsBasedInitializer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!msg) return;
  imu_buffer.emplace_back(*msg);
  if (imu_buffer.size() > 30) {
    imu_buffer.erase(imu_buffer.begin());
  }
}

bool BbsBasedInitializer::load_config(const std::string& config) {
  YAML::Node conf = YAML::LoadFile(config);

  std::cout << "[YAML] Loading topic name..." << std::endl;
  lidar_topic_name = conf["lidar_topic_name"].as<std::string>();
  map_topic_name = conf["map_topic_name"].as<std::string>();
  imu_topic_name = conf["imu_topic_name"].as<std::string>();
  std::cout << "[YAML] LiDAR topic name..." << lidar_topic_name << std::endl;
  std::cout << "[YAML] imu topic name..." << imu_topic_name << std::endl;

  trigger_topic_name = conf["trigger_topic_name"].as<std::string>();
  // for pub
  pose_topic_name = conf["pose_topic_name"].as<std::string>();

  std::cout << "[YAML] Loading 3D-BBS parameters..." << std::endl;
  min_level_res = conf["min_level_res"].as<double>();
  max_level = conf["max_level"].as<int>();

  if (min_level_res == 0.0 || max_level == 0) {
    std::cout << "[ERROR] Set min_level and num_layers except for 0" << std::endl;
    return false;
  }

  std::cout << "[YAML] Loading angular search range..." << std::endl;
  std::vector<double> min_rpy_temp = conf["min_rpy"].as<std::vector<double>>();
  std::vector<double> max_rpy_temp = conf["max_rpy"].as<std::vector<double>>();
  if (min_rpy_temp.size() == 3 && max_rpy_temp.size() == 3) {
    min_rpy = to_eigen(min_rpy_temp);
    max_rpy = to_eigen(max_rpy_temp);
  } else {
    std::cout << "[ERROR] Set min_rpy and max_rpy correctly" << std::endl;
    return false;
  }

  std::cout << "[YAML] Loading score threshold percentage..." << std::endl;
  score_threshold_percentage = conf["score_threshold_percentage"].as<double>();

  std::cout << "[YAML] Loading downsample parameters..." << std::endl;
  map_leaf_size = conf["map_leaf_size"].as<float>();
  src_leaf_size = conf["src_leaf_size"].as<float>();
  min_scan_range = conf["min_scan_range"].as<double>();
  max_scan_range = conf["max_scan_range"].as<double>();

  timeout_msec = conf["timeout_msec"].as<int>();

  return true;
}

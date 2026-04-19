// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pcd2pgm/pcd2pgm.hpp"

#include "pcl/common/transforms.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

#include <fstream>

namespace pcd2pgm
{
Pcd2PgmNode::Pcd2PgmNode(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);
  pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_cloud", 10);

  if (use_pgm_) {
    // ===== PGM 模式: 直接加载 PGM 文件作为地图 =====
    RCLCPP_INFO(get_logger(), "\u5730\u56fe\u6a21\u5f0f: PGM \u6587\u4ef6");
    if (!loadMapFromPgm()) {
      RCLCPP_ERROR(get_logger(), "PGM \u5730\u56fe\u52a0\u8f7d\u5931\u8d25!");
      return;
    }
  } else {
    // ===== PCD 模式: 从点云自动生成地图 =====
    RCLCPP_INFO(get_logger(), "\u5730\u56fe\u6a21\u5f0f: PCD \u81ea\u52a8\u751f\u6210");
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud_) == -1) {
      RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

    applyTransform();

    passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
    radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);
    statisticalOutlierFilter(cloud_after_radius_, sor_mean_k_, sor_stddev_thresh_);
    setMapTopicMsg(cloud_after_sor_, map_topic_msg_);
  }

  timer_ =
    create_wall_timer(std::chrono::seconds(1), std::bind(&Pcd2PgmNode::publishCallback, this));
}

void Pcd2PgmNode::publishCallback()
{
  if (!use_pgm_ && cloud_after_sor_) {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_after_sor_, output);
    output.header.frame_id = "map";
    output.header.stamp = now();
    pcd_publisher_->publish(output);
  }

  map_topic_msg_.header.stamp = now();
  map_publisher_->publish(map_topic_msg_);

  // 静态地图只需发布几次确保所有订阅者收到, 然后停止
  pub_count_++;
  if (pub_count_ >= 5) {
    RCLCPP_INFO(get_logger(), "Static map published %d times, stopping timer.", pub_count_);
    timer_->cancel();
  }
}

void Pcd2PgmNode::declareParameters()
{
  declare_parameter("pcd_file", "");
  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter("sor_mean_k", 50);             // 统计滤波: 计算均值的邻居数
  declare_parameter("sor_stddev_thresh", 1.0);      // 统计滤波: 标准差倍数阈值
  declare_parameter("map_topic_name", "map");
  declare_parameter("use_pgm", false);
  declare_parameter("pgm_file", "");
  declare_parameter("pgm_resolution", 0.05);
  declare_parameter("pgm_origin", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter("pgm_occupied_thresh", 0.65);
  declare_parameter("pgm_free_thresh", 0.25);
  declare_parameter("pgm_negate", 0);
  declare_parameter(
    "odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

void Pcd2PgmNode::getParameters()
{
  get_parameter("pcd_file", pcd_file_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("sor_mean_k", sor_mean_k_);
  get_parameter("sor_stddev_thresh", sor_stddev_thresh_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("use_pgm", use_pgm_);
  get_parameter("pgm_file", pgm_file_);
  get_parameter("pgm_resolution", pgm_resolution_);
  get_parameter("pgm_origin", pgm_origin_);
  get_parameter("pgm_occupied_thresh", pgm_occupied_thresh_);
  get_parameter("pgm_free_thresh", pgm_free_thresh_);
  get_parameter("pgm_negate", pgm_negate_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);
}

void Pcd2PgmNode::passThroughFilter(double thre_low, double thre_high, bool flag_in)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud_);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*filtered_cloud);

  cloud_after_pass_through_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After PassThrough filtering: %lu points",
    cloud_after_pass_through_->points.size());
}

void Pcd2PgmNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
  radius_outlier.setInputCloud(input_cloud);
  radius_outlier.setRadiusSearch(radius);
  radius_outlier.setMinNeighborsInRadius(thre_count);
  radius_outlier.filter(*filtered_cloud);

  cloud_after_radius_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After RadiusOutlier filtering: %lu points", cloud_after_radius_->points.size());
}

void Pcd2PgmNode::statisticalOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, int mean_k, double stddev_thresh)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input_cloud);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(stddev_thresh);
  sor.filter(*filtered_cloud);

  cloud_after_sor_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After StatisticalOutlier filtering: %lu points (removed %lu dynamic/noise points)",
    cloud_after_sor_->points.size(),
    input_cloud->points.size() - cloud_after_sor_->points.size());
}

void Pcd2PgmNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
  msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  for (const auto & point : cloud->points) {
    int i = std::floor((point.x - x_min) / map_resolution_);
    int j = std::floor((point.y - y_min) / map_resolution_);

    if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
      msg.data[i + j * msg.info.width] = 100;
    }
  }

  RCLCPP_INFO(get_logger(), "Map data size: %lu", msg.data.size());
}

bool Pcd2PgmNode::loadMapFromPgm()
{
  // 读取 PGM 文件
  std::ifstream pgm_file(pgm_file_, std::ios::binary);
  if (!pgm_file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open PGM: %s", pgm_file_.c_str());
    return false;
  }

  std::string magic;
  pgm_file >> magic;
  if (magic != "P5") {
    RCLCPP_ERROR(get_logger(), "Unsupported PGM format: %s (only P5 binary supported)", magic.c_str());
    return false;
  }

  // 跳过注释行
  char c;
  pgm_file.get(c);
  while (pgm_file.peek() == '#') {
    std::string comment;
    std::getline(pgm_file, comment);
  }

  int width, height, max_val;
  pgm_file >> width >> height >> max_val;
  pgm_file.get(c);  // 读取换行符

  std::vector<uint8_t> pixels(width * height);
  pgm_file.read(reinterpret_cast<char*>(pixels.data()), width * height);

  RCLCPP_INFO(get_logger(), "PGM loaded: %s (%dx%d, resolution=%.3f)",
    pgm_file_.c_str(), width, height, pgm_resolution_);

  // 转换为 OccupancyGrid
  map_topic_msg_.header.stamp = now();
  map_topic_msg_.header.frame_id = "map";
  map_topic_msg_.info.resolution = pgm_resolution_;
  map_topic_msg_.info.width = width;
  map_topic_msg_.info.height = height;
  map_topic_msg_.info.origin.position.x = pgm_origin_[0];
  map_topic_msg_.info.origin.position.y = pgm_origin_[1];
  map_topic_msg_.info.origin.position.z = 0.0;
  map_topic_msg_.info.origin.orientation.w = 1.0;
  map_topic_msg_.info.map_load_time = now();
  map_topic_msg_.data.resize(width * height);

  int occupied_count = 0, free_count = 0, unknown_count = 0;

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      // PGM 存储: 第一行是图片顶部 = 地图 Y 最大值
      // OccupancyGrid: data[0] 对应 origin (Y 最小值)
      int pgm_idx = (height - 1 - row) * width + col;
      int grid_idx = row * width + col;

      double pixel_val = static_cast<double>(pixels[pgm_idx]) / max_val;
      if (pgm_negate_) pixel_val = 1.0 - pixel_val;

      // pixel_val: 0=黑(障碍), 1=白(自由)
      double occ_prob = 1.0 - pixel_val;  // 占用概率

      if (occ_prob >= pgm_occupied_thresh_) {
        map_topic_msg_.data[grid_idx] = 100;  // 障碍
        occupied_count++;
      } else if (occ_prob <= pgm_free_thresh_) {
        map_topic_msg_.data[grid_idx] = 0;    // 自由
        free_count++;
      } else {
        map_topic_msg_.data[grid_idx] = -1;   // 未知
        unknown_count++;
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Map grid: %dx%d, occupied=%d, free=%d, unknown=%d",
    width, height, occupied_count, free_count, unknown_count);
  return true;
}

void Pcd2PgmNode::applyTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  transform.translation() << odom_to_lidar_odom_[0], odom_to_lidar_odom_[1], odom_to_lidar_odom_[2];
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[3], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[4], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[5], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform.inverse());
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::Pcd2PgmNode)

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

#ifndef PCD2PGM__PCD2PGM_HPP_
#define PCD2PGM__PCD2PGM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pcd2pgm
{
class Pcd2PgmNode : public rclcpp::Node
{
public:
  explicit Pcd2PgmNode(const rclcpp::NodeOptions & options);

private:
  void declareParameters();

  void getParameters();

  void passThroughFilter(double thre_low, double thre_high, bool flag_in);

  void radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud0, double radius, int thre_count);

  void statisticalOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, int mean_k, double stddev_thresh);

  void setMapTopicMsg(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg);

  void publishCallback();

  void applyTransform();

  bool loadMapFromPgm();

  bool use_pgm_;
  std::string pgm_file_;
  double pgm_resolution_;
  std::vector<double> pgm_origin_;
  double pgm_occupied_thresh_;
  double pgm_free_thresh_;
  int pgm_negate_;
  float thre_z_min_;
  float thre_z_max_;
  float thre_radius_;
  bool flag_pass_through_;
  float map_resolution_;
  int thres_point_count_;
  int sor_mean_k_;
  double sor_stddev_thresh_;
  std::string pcd_file_;
  std::string map_topic_name_;
  std::vector<double> odom_to_lidar_odom_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcd_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_pass_through_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_radius_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_sor_;
  nav_msgs::msg::OccupancyGrid map_topic_msg_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int pub_count_ = 0;
};
}  // namespace pcd2pgm

#endif  // PCD2PGM__PCD2PGM_HPP_

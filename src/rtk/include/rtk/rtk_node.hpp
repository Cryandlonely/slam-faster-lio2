#ifndef RTK_NODE_HPP
#define RTK_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace rtk
{

// ==================== GPS 数据结构 (NMEA 0183) ====================

struct GpsData
{
    double latitude      = 0.0;      // 纬度 (度, WGS84)
    double longitude     = 0.0;      // 经度 (度, WGS84)
    double altitude      = 0.0;      // 海拔高 MSL (米, GGA field 9)
    float  undulation    = 0.0f;     // 大地水准面差距 (米, GGA field 11)
    double speed         = 0.0;      // 地速 (m/s, RMC field 7 knots→m/s)
    int    fix_quality   = 0;        // GGA field 6: 0=无效,1=单点,2=差分,4=RTK固定,5=RTK浮动,6=航位推算
    int    num_satellites = 0;       // 使用卫星数 (GGA field 7)
    float  hdop          = 99.9f;    // 水平精度因子 (GGA field 8)
    float  lat_sigma     = 9999.0f;  // 纬度标准差 (米, GST field 6; 无GST时用 hdop×5 估算)
    float  lon_sigma     = 9999.0f;  // 经度标准差 (米, GST field 7)
    float  hgt_sigma     = 9999.0f;  // 高度标准差 (米, GST field 8)
    float  diff_age      = 0.0f;     // 差分改正龄期 (秒, GGA field 13)
    bool   valid         = false;    // fix_quality > 0

    // ---- HEADING 航向数据 ($GNHPR / $GNHDT 双天线) ----
    float  heading_deg   = 0.0f;     // 东向顺时针航向 (度, 0~360°; 0=E, 90=S, 180=W, 270=N)
    float  hdg_std_dev   = 0.5f;     // 航向标准差估计 (度, HPR/HDT无此字段, 固定0.5°)
    bool   heading_valid = false;    // 收到有效 $GNHPR/$GNHDT 时为 true
};

// ==================== EKF 状态 (GPS + IMU 融合) ====================
// 状态向量: [x, y, yaw, vx]  (ENU 坐标, m/rad/m/s)
struct EkfState {
    double x   = 0.0;    // 东向位置 (m)
    double y   = 0.0;    // 北向位置 (m)
    double yaw = 0.0;    // ENU 航向 (rad, [-π,π])
    double vx  = 0.0;    // 前向速度 (m/s)
    // 4×4 协方差矩阵 (行主序)
    double P[16] = {5.0,0,0,0, 0,5.0,0,0, 0,0,1.0,0, 0,0,0,1.0};
    bool   initialized = false;
    rclcpp::Time last_predict_stamp{0, 0, RCL_ROS_TIME};
};

// ==================== RTK 节点类 ====================

class RtkNode : public rclcpp::Node
{
public:
    explicit RtkNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RtkNode();

private:
    bool initSerial();
    void closeSerial();
    void serialReadThread();   // 接收串口字节，提取 NMEA 行
    void ProcessNmeaLine(const std::string& line);  // 校验并分发 NMEA 语句
    bool VerifyNmeaChecksum(const std::string& sentence) const;
    std::vector<std::string> SplitNmea(const std::string& sentence) const;
    double ParseNmeaLatLon(const std::string& value, const std::string& dir) const;
    void ParseGga(const std::vector<std::string>& fields);  // $GNGGA 位置
    void ParseHpr(const std::vector<std::string>& fields);  // $GNHPR 双天线航向/俯仰/横滚
    void ParseGst(const std::vector<std::string>& fields);  // $GNGST 位置精度
    void ParseRmc(const std::vector<std::string>& fields);  // $GNRMC 速度
    void publishGpsData();
    void timerCallback();
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);          // VRU IMU 回调
    void ChassisFeedbackCallback(const std_msgs::msg::String::SharedPtr msg); // 底盘速度回调

    // EKF 方法
    void EkfPredict(double gz, double dt);                    // 用陀螺仪预测
    void EkfUpdatePos(double x_gps, double y_gps, double r); // GPS 位置更新
    void EkfUpdateYaw(double yaw_gps, double r_yaw);          // GPS 航向更新

    // 将经纬度转换为相对济南参考点的局部 ENU 坐标 (米)
    void convertToLocalEnu(double lat, double lon,
                           double& x_east, double& y_north) const;

private:
    std::string serial_port_;
    std::string outdoor_odom_topic_;
    int         baud_rate_;
    int         serial_fd_;

    // 济南参考基准点 (WGS84 度)
    double ref_lat_;
    double ref_lon_;
    // 天线安装偏移 (度)
    double antenna_heading_offset_deg_ = 0.0;

    std::thread          read_thread_;
    std::atomic<bool>    running_;

    GpsData              current_gps_data_;
    std::mutex           data_mutex_;
    std::string          nmea_line_buf_;          // NMEA 行缓冲区
    uint32_t             gga_msg_count_ = 0;      // $GGA 语句接收计数
    uint32_t             hdt_msg_count_ = 0;      // $HDT 语句接收计数

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr      nav_sat_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          outdoor_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr            heading_pub_;   // 航向话题 (ENU yaw, rad)
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;            // VRU IMU 订阅
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         chassis_feedback_sub_; // 底盘速度订阅

    // EKF 融合状态
    EkfState    ekf_;
    std::mutex  ekf_mutex_;
    double      chassis_vx_ = 0.0;  // 底盘真实轮速 (m/s, 无锁读写)
};

}  // namespace rtk

#endif  // RTK_NODE_HPP

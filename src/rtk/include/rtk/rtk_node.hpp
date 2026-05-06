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

// ==================== Unicore 二进制协议定义 ====================

static const uint8_t  SYNC_BYTE_1    = 0xAA;
static const uint8_t  SYNC_BYTE_2    = 0x44;
static const uint8_t  SYNC_BYTE_3    = 0x12;
static const uint16_t BESTPOS_MSG_ID = 42;
static const uint16_t HEADING_MSG_ID = 971;   // 9.16.16 HEADING 航向信息

#pragma pack(push, 1)
// 二进制消息头 (28 字节) —— 参考表 9-21
struct BinaryHeader {
    uint8_t  sync1;            // 0xAA
    uint8_t  sync2;            // 0x44
    uint8_t  sync3;            // 0x12
    uint8_t  header_length;    // 头长度 0x1C(28)
    uint16_t msg_id;           // Log 的信息 ID
    uint8_t  msg_type;         // 00=二进制, 01=ASCII, 10=简化ASCII
    uint8_t  reserved1;        // 保留
    uint16_t msg_length;       // 信息长度(字节), 不包括头和CRC
    uint16_t reserved2;        // 保留
    uint8_t  idle_time;        // 处理器空闲时间 (0~200)
    uint8_t  time_status;      // GPS时间质量 (20=UNKNOWN, 160=FINE)
    uint16_t week;             // GPS 周数
    uint32_t milliseconds;     // GPS 周内毫秒
    uint32_t reserved3;        // 保留
    uint16_t bds_gps_offset;   // 北斗与GPS时系差
    uint16_t reserved4;        // 保留
};

// BESTPOS 消息体 (72 字节) —— 参考表 9-29
struct BestPosBody {
    uint32_t sol_status;        // H+0:  解状态 (表 9-48, 0=SOL_COMPUTED)
    uint32_t pos_type;          // H+4:  位置类型 (表 9-47)
    double   lat;               // H+8:  纬度, deg
    double   lon;               // H+16: 经度, deg
    double   hgt;               // H+24: 海拔高, m
    float    undulation;        // H+32: 大地水准面差距, m
    uint32_t datum_id;          // H+36: 坐标系 ID (WGS84=61)
    float    lat_sigma;         // H+40: 纬度标准差, m
    float    lon_sigma;         // H+44: 经度标准差, m
    float    hgt_sigma;         // H+48: 高度标准差, m
    char     stn_id[4];         // H+52: 基站 ID
    float    diff_age;          // H+56: 差分龄期, s
    float    sol_age;           // H+60: 解的龄期, s
    uint8_t  num_svs;           // H+64: 跟踪的卫星数
    uint8_t  num_soln_svs;      // H+65: 解中使用的卫星数
    uint8_t  reserved_66;       // H+66: 保留
    uint8_t  reserved_67;       // H+67: 保留
    uint8_t  reserved_68;       // H+68: 保留
    uint8_t  ext_sol_stat;      // H+69: 扩展解状态 (表 9-72)
    uint8_t  galileo_sig_mask;  // H+70: Galileo 信号掩码 (表 9-50)
    uint8_t  gps_glo_bds_mask;  // H+71: GPS/GLONASS/BDS 信号掩码 (表 9-49)
};

// HEADING 消息体 (44 字节) —— 参考表 9-40, Message ID=971
struct HeadingBody {
    uint32_t sol_status;        // H+0:  解状态 (表 9-48, 0=SOL_COMPUTED)
    uint32_t pos_type;          // H+4:  位置类型 (表 9-47)
    float    length;            // H+8:  基线长 (0~3000 m)
    float    heading;           // H+12: 航向 (0~360.0 deg, 顺时针自真北)
    float    pitch;             // H+16: 俯仰 (±90 deg)
    float    reserved;          // H+20: 保留
    float    hdg_std_dev;       // H+24: 航向标准偏差 (deg)
    float    ptch_std_dev;      // H+28: 俯仰标准偏差 (deg)
    char     stn_id[4];         // H+32: 基站 ID
    uint8_t  num_svs;           // H+36: 跟踪的卫星数
    uint8_t  num_soln_svs;      // H+37: 使用的卫星数
    uint8_t  num_obs;           // H+38: 截止高度角以上的卫星数
    uint8_t  num_multi;         // H+39: 有 L2 观测的卫星数
    uint8_t  reserved2;         // H+40: 保留
    uint8_t  ext_sol_stat;      // H+41: 扩展解状态 (表 9-72)
    uint8_t  galileo_sig_mask;  // H+42: Galileo 信号掩码 (表 9-50)
    uint8_t  gps_glo_bds_mask;  // H+43: GPS/GLONASS/BDS 信号掩码 (表 9-49)
};
#pragma pack(pop)

// ==================== GPS 数据结构 ====================

struct GpsData
{
    double   latitude    = 0.0;       // 纬度 (度)
    double   longitude   = 0.0;       // 经度 (度)
    double   altitude    = 0.0;       // 海拔高 (米)
    float    undulation  = 0.0f;      // 大地水准面差距 (米)
    double   speed       = 0.0;       // 地速 (m/s, BESTPOS 不含, 保留备用)
    double   course      = 0.0;       // 航向角 (度, BESTPOS 不含, 保留备用)
    uint32_t sol_status  = 1;         // 解算状态 (表9-48: 0=SOL_COMPUTED, 1=INSUFFICIENT_OBS, 2=NO_CONVERGENCE, 3=SINGULARITY)
    uint32_t pos_type    = 0;         // 位置类型 (表9-47)
    int      fix_quality = 0;         // 定位质量 (0=无效,1=单点,2=差分,4=RTK固定,5=RTK浮动)
    int      num_satellites  = 0;     // 跟踪卫星数
    int      num_soln_svs    = 0;     // 解中使用的卫星数
    float    lat_sigma   = 9999.0f;   // 纬度标准差 (米)
    float    lon_sigma   = 9999.0f;   // 经度标准差 (米)
    float    hgt_sigma   = 9999.0f;   // 高度标准差 (米)
    float    diff_age    = 0.0f;      // 差分龄期 (秒)
    float    sol_age     = 0.0f;      // 解的龄期 (秒)
    bool     valid       = false;

    // ---- HEADING 航向数据 (Message ID=971) ----
    float    heading_deg     = 0.0f;   // 航向角 (度, 0~360, 顺时针自真北)
    float    pitch_deg       = 0.0f;   // 俯仰角 (度, ±90)
    float    baseline_length = 0.0f;   // 基线长度 (米)
    float    hdg_std_dev     = 9999.0f;// 航向标准偏差 (度)
    uint32_t hdg_sol_status  = 1;      // 航向解状态 (0=SOL_COMPUTED)
    uint32_t hdg_pos_type    = 0;      // 航向位置类型
    bool     heading_valid   = false;  // 航向数据是否有效
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
    void serialReadThread();   // 接收并解析 Unicore 二进制数据
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
    std::vector<uint8_t> binary_buffer_;   // 二进制接收缓冲区
    uint32_t             heading_msg_count_ = 0;  // HEADING 消息接收计数
    uint32_t             bestpos_msg_count_ = 0;  // BESTPOS 消息接收计数

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

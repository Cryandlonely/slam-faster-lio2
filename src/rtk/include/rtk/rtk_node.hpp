#ifndef RTK_NODE_HPP
#define RTK_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

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

// ==================== GPS 数据结构 ====================

struct GpsData
{
    double   latitude    = 0.0;       // 纬度 (度)
    double   longitude   = 0.0;       // 经度 (度)
    double   altitude    = 0.0;       // 海拔高 (米)
    float    undulation  = 0.0f;      // 大地水准面差距 / 高程异常 (GGA geoid sep, 米)
    double   speed       = 0.0;       // 地速 (m/s, 保留备用)
    double   course      = 0.0;       // 航向角 (度, 保留备用)
    uint32_t sol_status  = 1;         // 解算状态: 0=有效解, 1=无效
    uint32_t pos_type    = 0;         // 位置类型 (Unicore 兼容: 50=RTK固定, 34=RTK浮动, 17=差分, 16=单点)
    int      fix_quality = 0;         // GGA quality: 0=无效,1=单点,2=差分,4=RTK固定,5=RTK浮动
    int      num_satellites  = 0;     // 使用卫星数 (GGA numSV)
    int      num_soln_svs    = 0;     // 同 num_satellites (NMEA 无单独字段)
    float    lat_sigma   = 9999.0f;   // 纬度标准差 (米, 来自 GST; 无 GST 时按质量估算)
    float    lon_sigma   = 9999.0f;   // 经度标准差 (米)
    float    hgt_sigma   = 9999.0f;   // 高度标准差 (米)
    float    diff_age    = 0.0f;      // 差分龄期 (秒, GGA 字段 13)
    float    sol_age     = 0.0f;      // 解的龄期 (NMEA 无此字段, 保留为 0)
    bool     valid       = false;

    // ---- 航向数据 (来自 $GNHDT / $GPHDT) ----
    float    heading_deg     = 0.0f;   // 航向角 (度, 0~360, 顺时针自真北)
    float    pitch_deg       = 0.0f;   // 俯仰角 (NMEA HDT 无此字段, 保留为 0)
    float    baseline_length = 0.0f;   // 基线长度 (NMEA HDT 无此字段, 保留为 0)
    float    hdg_std_dev     = 1.0f;   // 航向标准偏差 (NMEA HDT 无此字段, 默认 1°)
    uint32_t hdg_sol_status  = 1;      // 航向解状态: 0=有效
    uint32_t hdg_pos_type    = 0;      // 航向位置类型 (同 pos_type)
    bool     heading_valid   = false;  // 航向数据是否有效
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
    void serialReadThread();            // 逐行接收并路由 NMEA 0183 语句
    void parseNmeaSentence(const std::string& sentence);
    bool validateNmeaChecksum(const std::string& sentence) const;
    bool parseGGA(const std::vector<std::string>& fields);  // $G?GGA: 位置+质量
    bool parseHDT(const std::vector<std::string>& fields);  // $G?HDT: 真北航向
    bool parseGST(const std::vector<std::string>& fields);  // $G?GST: 误差统计
    void publishGpsData();
    void timerCallback();
    void convertToLocalEnu(double lat, double lon,
                           double& x_east, double& y_north) const;

private:
    std::string serial_port_;
    std::string outdoor_odom_topic_;
    int         baud_rate_;
    int         serial_fd_;

    double ref_lat_;
    double ref_lon_;
    double antenna_heading_offset_deg_ = 0.0;

    std::thread          read_thread_;
    std::atomic<bool>    running_;

    GpsData              current_gps_data_;
    std::mutex           data_mutex_;
    std::string          line_buffer_;   // NMEA 行组装缓冲区
    uint32_t             gga_count_ = 0; // 已解析的 GGA 语句数
    uint32_t             hdt_count_ = 0; // 已解析的 HDT 语句数

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr      nav_sat_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          outdoor_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           heading_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rtk

#endif  // RTK_NODE_HPP

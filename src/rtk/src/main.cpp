#include "rtk/rtk_node.hpp"

namespace rtk
{

// ==================== NMEA 0183 工具函数 ====================

// 校验 NMEA XOR 校验和: '$' 后到 '*' 前所有字节 XOR == '*' 后两位十六进制
static bool nmea_checksum_valid(const std::string& s)
{
    if (s.empty() || s[0] != '$') return false;
    size_t star = s.rfind('*');
    if (star == std::string::npos || star + 2 >= s.size()) return false;
    uint8_t chk = 0;
    for (size_t i = 1; i < star; ++i) chk ^= static_cast<uint8_t>(s[i]);
    char hex[3] = { s[star + 1], s[star + 2], '\0' };
    return chk == static_cast<uint8_t>(std::strtol(hex, nullptr, 16));
}

// 分割 NMEA 句子为字段列表 (去掉 '$' 前缀和 '*xx' 后缀)
static std::vector<std::string> nmea_split(const std::string& s)
{
    std::vector<std::string> fields;
    size_t start = (s[0] == '$') ? 1 : 0;
    size_t star  = s.rfind('*');
    std::string body = s.substr(start,
        star != std::string::npos ? star - start : std::string::npos);
    size_t pos = 0;
    while (true) {
        size_t comma = body.find(',', pos);
        fields.push_back(body.substr(pos,
            comma == std::string::npos ? std::string::npos : comma - pos));
        if (comma == std::string::npos) break;
        pos = comma + 1;
    }
    return fields;
}

// NMEA DDDMM.MMMM + 方向 → 十进制度
static double nmea_to_deg(const std::string& val, const std::string& dir)
{
    if (val.empty()) return 0.0;
    size_t dot = val.find('.');
    if (dot == std::string::npos || dot < 2) return 0.0;
    double deg = std::stod(val.substr(0, dot - 2));
    double min = std::stod(val.substr(dot - 2));
    double result = deg + min / 60.0;
    if (!dir.empty() && (dir[0] == 'S' || dir[0] == 'W')) result = -result;
    return result;
}

// ==================== RtkNode 实现 ====================

RtkNode::RtkNode(const rclcpp::NodeOptions& options)
    : Node("rtk_node", options),
      serial_fd_(-1),
      running_(false)
{
    this->declare_parameter<std::string>("serial_port", "/dev/ttyS3");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<std::string>("outdoor_odom_topic", "/outdoor/odom");
    this->declare_parameter<double>("reference.jinan_lat", 36.66111);
    this->declare_parameter<double>("reference.jinan_lon", 117.01665);
    this->declare_parameter<double>("antenna_heading_offset_deg", 0.0);

    serial_port_    = this->get_parameter("serial_port").as_string();
    baud_rate_      = this->get_parameter("baud_rate").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    outdoor_odom_topic_ = this->get_parameter("outdoor_odom_topic").as_string();
    ref_lat_        = this->get_parameter("reference.jinan_lat").as_double();
    ref_lon_        = this->get_parameter("reference.jinan_lon").as_double();
    antenna_heading_offset_deg_ = this->get_parameter("antenna_heading_offset_deg").as_double();

    RCLCPP_INFO(this->get_logger(), "RTK Node 启动 (NMEA 0183 协议)");
    RCLCPP_INFO(this->get_logger(), "串口: %s, 波特率: %d", serial_port_.c_str(), baud_rate_);
    RCLCPP_INFO(this->get_logger(), "解析语句: $G?GGA (位置+质量), $G?HDT (真北航向), $G?GST (误差统计)");
    RCLCPP_INFO(this->get_logger(), "济南参考点: lat=%.9f, lon=%.9f", ref_lat_, ref_lon_);
    if (std::abs(antenna_heading_offset_deg_) > 0.01)
        RCLCPP_INFO(this->get_logger(), "天线安装偏移: %.1f°", antenna_heading_offset_deg_);

    nav_sat_pub_      = this->create_publisher<sensor_msgs::msg::NavSatFix>("rtk/fix", 10);
    velocity_pub_     = this->create_publisher<geometry_msgs::msg::TwistStamped>("rtk/velocity", 10);
    odom_pub_         = this->create_publisher<nav_msgs::msg::Odometry>("rtk/odom", 10);
    outdoor_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(outdoor_odom_topic_, 10);
    heading_pub_      = this->create_publisher<std_msgs::msg::Float64>("rtk/heading", 10);

    if (!initSerial()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s", serial_port_.c_str());
        return;
    }

    running_     = true;
    read_thread_ = std::thread(&RtkNode::serialReadThread, this);

    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RtkNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "RTK Node 初始化完成，等待 NMEA 语句 ...");
    RCLCPP_INFO(this->get_logger(), "统一室外位姿话题: %s", outdoor_odom_topic_.c_str());
}

RtkNode::~RtkNode()
{
    running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
    closeSerial();
    RCLCPP_INFO(this->get_logger(), "RTK Node 关闭");
}

bool RtkNode::initSerial()
{
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s, 错误: %s",
                     serial_port_.c_str(), strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "获取串口属性失败: %s", strerror(errno));
        close(serial_fd_); serial_fd_ = -1;
        return false;
    }

    speed_t baud;
    switch (baud_rate_) {
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        default:
            RCLCPP_WARN(this->get_logger(), "不支持的波特率 %d, 使用默认 115200", baud_rate_);
            baud = B115200;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "设置串口属性失败: %s", strerror(errno));
        close(serial_fd_); serial_fd_ = -1;
        return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(this->get_logger(), "串口 %s 打开成功", serial_port_.c_str());
    return true;
}

void RtkNode::closeSerial()
{
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

// ==================== 串口读取线程: 逐行解析 NMEA ====================

void RtkNode::serialReadThread()
{
    char read_buf[256];

    while (running_) {
        if (serial_fd_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        ssize_t n = read(serial_fd_, read_buf, sizeof(read_buf) - 1);

        if (n > 0) {
            line_buffer_.append(read_buf, n);

            // 提取所有以 '\n' 结尾的完整行
            size_t pos = 0;
            while (true) {
                size_t nl = line_buffer_.find('\n', pos);
                if (nl == std::string::npos) break;

                std::string line = line_buffer_.substr(pos, nl - pos);
                pos = nl + 1;

                // 去除行尾 '\r'
                if (!line.empty() && line.back() == '\r') line.pop_back();

                // 只处理以 '$' 开头且长度合理的行
                if (line.size() > 6 && line[0] == '$')
                    parseNmeaSentence(line);
            }

            line_buffer_ = line_buffer_.substr(pos);

            // 防溢出保护: 超 4KB 时清空 (正常行长 < 100 字节)
            if (line_buffer_.size() > 4096) line_buffer_.clear();

        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN(this->get_logger(), "串口读取错误: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

// ==================== NMEA 句子路由 ====================

void RtkNode::parseNmeaSentence(const std::string& sentence)
{
    if (!validateNmeaChecksum(sentence)) {
        RCLCPP_DEBUG(this->get_logger(), "NMEA 校验和错误: %.80s", sentence.c_str());
        return;
    }

    auto fields = nmea_split(sentence);
    if (fields.empty() || fields[0].size() < 5) return;

    // 取语句类型 (去掉 2 字符 talker 前缀: GP/GN/GL/GA/GB 等)
    const std::string msg_type = fields[0].substr(2);

    if      (msg_type == "GGA") parseGGA(fields);
    else if (msg_type == "HDT") parseHDT(fields);
    else if (msg_type == "GST") parseGST(fields);
}

bool RtkNode::validateNmeaChecksum(const std::string& s) const
{
    return nmea_checksum_valid(s);
}

// ==================== GGA 解析 ====================
// $G?GGA,hhmmss.ss,Lat,N/S,Lon,E/W,quality,numSV,HDOP,alt,M,geoid,M,diffAge,diffRef*xx
bool RtkNode::parseGGA(const std::vector<std::string>& fields)
{
    try {
        if (fields.size() < 10) return false;

        int quality = fields[6].empty() ? 0 : std::stoi(fields[6]);

        double lat = 0.0, lon = 0.0, alt = 0.0;
        float  geoid_sep = 0.0f;
        int    num_sv    = 0;
        float  diff_age  = 0.0f;

        if (quality > 0) {
            if (fields[2].empty() || fields[4].empty()) return false;
            lat = nmea_to_deg(fields[2], fields[3]);
            lon = nmea_to_deg(fields[4], fields[5]);
            alt = fields[9].empty() ? 0.0 : std::stod(fields[9]);
            if (fields.size() > 11 && !fields[11].empty())
                geoid_sep = static_cast<float>(std::stod(fields[11]));
            num_sv = fields[7].empty() ? 0 : std::stoi(fields[7]);
            if (fields.size() > 13 && !fields[13].empty())
                diff_age = static_cast<float>(std::stod(fields[13]));
        }

        // GGA quality → Unicore 兼容 pos_type (供 publishGpsData 中 NavSatFix 映射复用)
        uint32_t pos_type = 0;
        switch (quality) {
            case 4: pos_type = 50; break;  // RTK 固定 → NARROW_INT
            case 5: pos_type = 34; break;  // RTK 浮动 → NARROW_FLOAT
            case 2: pos_type = 17; break;  // DGPS → PSRDIFF
            case 1: pos_type = 16; break;  // 单点 → SINGLE
            default: break;
        }

        // 按质量等级给出保守 sigma 默认值 (收到 GST 语句后会覆盖)
        float est = (quality == 4) ? 0.02f
                  : (quality == 5) ? 0.10f
                  : (quality == 2) ? 0.50f
                  : 5.0f;

        std::lock_guard<std::mutex> lock(data_mutex_);
        current_gps_data_.latitude       = lat;
        current_gps_data_.longitude      = lon;
        current_gps_data_.altitude       = alt;
        current_gps_data_.undulation     = geoid_sep;
        current_gps_data_.fix_quality    = quality;
        current_gps_data_.pos_type       = pos_type;
        current_gps_data_.sol_status     = (quality > 0) ? 0u : 1u;
        current_gps_data_.num_satellites = num_sv;
        current_gps_data_.num_soln_svs   = num_sv;
        current_gps_data_.diff_age       = diff_age;
        current_gps_data_.sol_age        = 0.0f;
        current_gps_data_.valid          = (quality > 0);
        current_gps_data_.lat_sigma      = est;
        current_gps_data_.lon_sigma      = est;
        current_gps_data_.hgt_sigma      = est * 2.0f;
        gga_count_++;
        return true;

    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "GGA 解析异常: %s", e.what());
        return false;
    }
}

// ==================== HDT 解析 ====================
// $G?HDT,heading,T*xx  heading: 真北顺时针, 0~360°
bool RtkNode::parseHDT(const std::vector<std::string>& fields)
{
    try {
        if (fields.size() < 2 || fields[1].empty()) return false;

        double heading = std::stod(fields[1]);

        std::lock_guard<std::mutex> lock(data_mutex_);
        current_gps_data_.heading_deg    = static_cast<float>(heading);
        current_gps_data_.pitch_deg      = 0.0f;
        current_gps_data_.baseline_length = 0.0f;
        current_gps_data_.hdg_std_dev    = 1.0f;   // NMEA HDT 无标准偏差字段, 默认 1°
        current_gps_data_.hdg_sol_status = 0;
        current_gps_data_.hdg_pos_type   = current_gps_data_.pos_type;
        current_gps_data_.heading_valid  = true;
        hdt_count_++;
        return true;

    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "HDT 解析异常: %s", e.what());
        return false;
    }
}

// ==================== GST 解析 ====================
// $G?GST,time,rms,semi_maj,semi_min,orient,std_lat,std_lon,std_alt*xx
bool RtkNode::parseGST(const std::vector<std::string>& fields)
{
    try {
        if (fields.size() < 9) return false;
        if (fields[6].empty() || fields[7].empty() || fields[8].empty()) return false;

        float std_lat = static_cast<float>(std::stod(fields[6]));
        float std_lon = static_cast<float>(std::stod(fields[7]));
        float std_alt = static_cast<float>(std::stod(fields[8]));

        if (std_lat <= 0.0f || std_lon <= 0.0f) return false;

        std::lock_guard<std::mutex> lock(data_mutex_);
        current_gps_data_.lat_sigma = std_lat;
        current_gps_data_.lon_sigma = std_lon;
        current_gps_data_.hgt_sigma = std_alt;
        return true;

    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "GST 解析异常: %s", e.what());
        return false;
    }
}

void RtkNode::timerCallback()
{
    publishGpsData();
}

void RtkNode::convertToLocalEnu(double lat, double lon,
                                 double& x_east, double& y_north) const
{
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kMetersPerDegreeLat = 111320.0;
    const double meters_per_degree_lon =
        111320.0 * std::cos(ref_lat_ * kDegToRad);

    x_east  = (lon - ref_lon_) * meters_per_degree_lon;
    y_north = (lat - ref_lat_) * kMetersPerDegreeLat;
}

void RtkNode::publishGpsData()
{
    GpsData  snap;
    uint32_t gga_cnt, hdt_cnt;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        snap    = current_gps_data_;
        gga_cnt = gga_count_;
        hdt_cnt = hdt_count_;
    }

    auto now = this->now();

    // -------- NavSatFix --------
    auto nav_msg = sensor_msgs::msg::NavSatFix();
    nav_msg.header.stamp    = now;
    nav_msg.header.frame_id = "gps";

    nav_msg.latitude  = snap.latitude;
    nav_msg.longitude = snap.longitude;
    nav_msg.altitude  = snap.altitude;

    // pos_type → NavSatStatus (使用与原 Unicore 版相同的映射)
    switch (snap.pos_type) {
        case 48: case 49: case 50: case 56:
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            break;
        case 32: case 33: case 34: case 55:
        case 17: case 18: case 54:
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
            break;
        case 1: case 16: case 52: case 53:
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            break;
        default:
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }

    nav_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS     |
                             sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS  |
                             sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO  |
                             sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;

    double cov_lat = static_cast<double>(snap.lat_sigma) * snap.lat_sigma;
    double cov_lon = static_cast<double>(snap.lon_sigma) * snap.lon_sigma;
    double cov_hgt = static_cast<double>(snap.hgt_sigma) * snap.hgt_sigma;
    nav_msg.position_covariance[0] = cov_lat;
    nav_msg.position_covariance[4] = cov_lon;
    nav_msg.position_covariance[8] = cov_hgt;
    nav_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    nav_sat_pub_->publish(nav_msg);

    // -------- TwistStamped (GGA 无速度信息, 置零) --------
    auto vel_msg = geometry_msgs::msg::TwistStamped();
    vel_msg.header.stamp    = now;
    vel_msg.header.frame_id = "gps";
    vel_msg.twist.linear.x  = 0.0;
    vel_msg.twist.linear.y  = 0.0;
    vel_msg.twist.linear.z  = 0.0;
    velocity_pub_->publish(vel_msg);

    // -------- Odometry --------
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp    = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link";

    double x_east = 0.0, y_north = 0.0;
    if (snap.valid)
        convertToLocalEnu(snap.latitude, snap.longitude, x_east, y_north);

    odom_msg.pose.pose.position.x = x_east;
    odom_msg.pose.pose.position.y = y_north;
    odom_msg.pose.pose.position.z = snap.altitude;

    // HDT 航向: 真北顺时针 → ENU yaw (yaw=0 朝东, 逆时针为正)
    // 1) 补偿天线安装偏移: heading_corrected = heading_raw + offset
    // 2) 转 ENU: yaw_enu = π/2 - heading_corrected_rad
    if (snap.heading_valid) {
        double heading_corrected = snap.heading_deg + antenna_heading_offset_deg_;
        double heading_rad = heading_corrected * M_PI / 180.0;
        double yaw_enu = M_PI / 2.0 - heading_rad;
        while (yaw_enu >  M_PI) yaw_enu -= 2.0 * M_PI;
        while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;

        odom_msg.pose.pose.orientation.w = std::cos(yaw_enu / 2.0);
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(yaw_enu / 2.0);

        double hdg_cov = snap.hdg_std_dev * M_PI / 180.0;
        hdg_cov *= hdg_cov;
        odom_msg.pose.covariance[35] = hdg_cov;

        auto hdg_msg = std_msgs::msg::Float64();
        hdg_msg.data = yaw_enu;
        heading_pub_->publish(hdg_msg);
    } else {
        odom_msg.pose.pose.orientation.w = 1.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
    }

    odom_msg.pose.covariance[0]  = cov_lat;
    odom_msg.pose.covariance[7]  = cov_lon;
    odom_msg.pose.covariance[14] = cov_hgt;
    odom_msg.pose.covariance[21] = 9999.0;
    odom_msg.pose.covariance[28] = 9999.0;
    if (!snap.heading_valid)
        odom_msg.pose.covariance[35] = 9999.0;

    odom_pub_->publish(odom_msg);
    outdoor_odom_pub_->publish(odom_msg);

    // 降频日志 (每 5 次输出一次)
    static int log_counter = 0;
    if (++log_counter >= 5) {
        log_counter = 0;
        RCLCPP_INFO(this->get_logger(),
            "GGA: lat=%.9f lon=%.9f alt=%.4fm | quality=%d svs=%d"
            " | σ(lat=%.4f lon=%.4f alt=%.4f)m diff_age=%.1fs"
            " | GGA#=%u HDT#=%u",
            snap.latitude, snap.longitude, snap.altitude,
            snap.fix_quality, snap.num_satellites,
            snap.lat_sigma, snap.lon_sigma, snap.hgt_sigma,
            snap.diff_age, gga_cnt, hdt_cnt);

        if (snap.heading_valid) {
            double hdg_corrected = snap.heading_deg + antenna_heading_offset_deg_;
            double yaw_enu = M_PI / 2.0 - hdg_corrected * M_PI / 180.0;
            while (yaw_enu >  M_PI) yaw_enu -= 2.0 * M_PI;
            while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;
            RCLCPP_INFO(this->get_logger(),
                "HDT: raw=%.2f° +offset=%.1f° → corrected=%.2f° → ENU_yaw=%.4f rad (%.2f°)",
                snap.heading_deg, antenna_heading_offset_deg_, hdg_corrected,
                yaw_enu, yaw_enu * 180.0 / M_PI);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "HDT: 无有效航向 | GGA#=%u HDT#=%u", gga_cnt, hdt_cnt);
        }
    }
}

}  // namespace rtk

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rtk::RtkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


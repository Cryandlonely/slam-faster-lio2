#include "rtk/rtk_node.hpp"

namespace rtk
{

// ==================== CRC32 校验 (Unicore 二进制协议) ====================

static uint32_t crc32_table_[256];
static bool     crc32_table_init_ = false;

static void init_crc32_table()
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
        crc32_table_[i] = crc;
    }
    crc32_table_init_ = true;
}

static uint32_t calc_crc32(const uint8_t* data, size_t len)
{
    if (!crc32_table_init_) init_crc32_table();
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = crc32_table_[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc;
}

// ==================== RtkNode 实现 ====================

RtkNode::RtkNode(const rclcpp::NodeOptions& options)
    : Node("rtk_node", options),
      serial_fd_(-1),
      running_(false)
{
    // 声明参数
    this->declare_parameter<std::string>("serial_port", "/dev/ttyS3");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<std::string>("outdoor_odom_topic", "/outdoor/odom");
    // 济南参考基准点 (WGS84 度)
    this->declare_parameter<double>("reference.jinan_lat", 36.66111);
    this->declare_parameter<double>("reference.jinan_lon", 117.01665);
    // 天线安装偏移: 天线基线方向与狗头朝向的夹角 (度, 顺时针为正)
    // 例如: 狗朝正西时 RTK 报 0°(正北) → offset = -90 (或 270)
    this->declare_parameter<double>("antenna_heading_offset_deg", 0.0);
    
    // 获取参数
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    outdoor_odom_topic_ = this->get_parameter("outdoor_odom_topic").as_string();
    ref_lat_ = this->get_parameter("reference.jinan_lat").as_double();
    ref_lon_ = this->get_parameter("reference.jinan_lon").as_double();
    antenna_heading_offset_deg_ = this->get_parameter("antenna_heading_offset_deg").as_double();
    if (std::abs(antenna_heading_offset_deg_) > 0.01) {
        RCLCPP_INFO(this->get_logger(), "天线安装偏移: %.1f°", antenna_heading_offset_deg_);
    }
    
    RCLCPP_INFO(this->get_logger(), "RTK Node 启动 (Unicore 二进制协议)");
    RCLCPP_INFO(this->get_logger(), "串口: %s, 波特率: %d", serial_port_.c_str(), baud_rate_);
    RCLCPP_INFO(this->get_logger(), "济南参考点: lat=%.9f, lon=%.9f", ref_lat_, ref_lon_);
    
    // 创建发布者
    nav_sat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("rtk/fix", 10);
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("rtk/velocity", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("rtk/odom", 10);
    outdoor_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(outdoor_odom_topic_, 10);
    heading_pub_ = this->create_publisher<std_msgs::msg::Float64>("rtk/heading", 10);
    
    // 预分配二进制缓冲区
    binary_buffer_.reserve(4096);
    
    // 初始化串口
    if (!initSerial()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s", serial_port_.c_str());
        return;
    }
    
    // 启动读取线程
    running_ = true;
    read_thread_ = std::thread(&RtkNode::serialReadThread, this);
    
    // 创建定时器用于定期发布数据
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RtkNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "RTK Node 初始化完成，等待 BESTPOS(ID=42) / HEADING(ID=971) 二进制消息 ...");
    RCLCPP_INFO(this->get_logger(), "统一室外位姿话题: %s", outdoor_odom_topic_.c_str());
}

RtkNode::~RtkNode()
{
    running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    closeSerial();
    RCLCPP_INFO(this->get_logger(), "RTK Node 关闭");
}

bool RtkNode::initSerial()
{
    // 打开串口
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s, 错误: %s", 
                     serial_port_.c_str(), strerror(errno));
        return false;
    }
    
    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "获取串口属性失败: %s", strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    // 设置波特率
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
    
    // 配置: 8N1, 无流控
    tty.c_cflag &= ~PARENB;         // 无校验
    tty.c_cflag &= ~CSTOPB;         // 1位停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;             // 8位数据位
    tty.c_cflag &= ~CRTSCTS;        // 无硬件流控
    tty.c_cflag |= CREAD | CLOCAL;  // 启用接收, 忽略调制解调器控制线
    
    // 输入模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 无软件流控
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    // 输出模式
    tty.c_oflag &= ~OPOST;          // 原始输出
    tty.c_oflag &= ~ONLCR;
    
    // 本地模式
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
    
    // 读取设置
    tty.c_cc[VMIN] = 0;             // 非阻塞读取
    tty.c_cc[VTIME] = 1;            // 100ms超时
    
    // 应用设置
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "设置串口属性失败: %s", strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    // 清空缓冲区
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

void RtkNode::serialReadThread()
{
    uint8_t read_buf[512];

    while (running_) {
        if (serial_fd_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 读取原始字节数据
        ssize_t bytes_read = read(serial_fd_, read_buf, sizeof(read_buf));

        if (bytes_read > 0) {
            binary_buffer_.insert(binary_buffer_.end(), read_buf, read_buf + bytes_read);
        } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN(this->get_logger(), "串口读取错误: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 在缓冲区中解析完整的 Unicore 二进制消息
        while (binary_buffer_.size() >= sizeof(BinaryHeader) + 4) {

            // 搜索同步头 0xAA 0x44 0x12
            size_t sync_pos = SIZE_MAX;
            for (size_t i = 0; i + 2 < binary_buffer_.size(); i++) {
                if (binary_buffer_[i]   == SYNC_BYTE_1 &&
                    binary_buffer_[i+1] == SYNC_BYTE_2 &&
                    binary_buffer_[i+2] == SYNC_BYTE_3) {
                    sync_pos = i;
                    break;
                }
            }

            // 未找到同步头，丢弃旧数据（保留末尾2字节防止跨包）
            if (sync_pos == SIZE_MAX) {
                if (binary_buffer_.size() > 2)
                    binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.end() - 2);
                break;
            }

            // 丢弃同步头之前的无效数据
            if (sync_pos > 0)
                binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.begin() + sync_pos);

            // 头部数据不足，等待更多数据
            if (binary_buffer_.size() < sizeof(BinaryHeader)) break;

            // 解析消息头
            BinaryHeader header;
            std::memcpy(&header, binary_buffer_.data(), sizeof(BinaryHeader));

            // 完整消息长度 = 头部长度 + 消息体长度 + 4字节CRC
            size_t total_msg_len = (size_t)header.header_length + header.msg_length + 4;

            // 数据不足，等待更多数据
            if (binary_buffer_.size() < total_msg_len) break;

            // CRC32 校验
            size_t   crc_data_len = (size_t)header.header_length + header.msg_length;
            uint32_t computed_crc = calc_crc32(binary_buffer_.data(), crc_data_len);
            uint32_t received_crc = 0;
            std::memcpy(&received_crc, binary_buffer_.data() + crc_data_len, 4);

            if (computed_crc != received_crc) {
                // CRC 失败，跳过当前同步头，继续搜索
                binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.begin() + 3);
                continue;
            }

            // CRC 通过，解析 BESTPOS 消息
            if (header.msg_id == BESTPOS_MSG_ID &&
                header.msg_length >= sizeof(BestPosBody))
            {
                BestPosBody bestpos;
                std::memcpy(&bestpos, binary_buffer_.data() + header.header_length, sizeof(BestPosBody));

                GpsData gps;
                gps.latitude       = bestpos.lat;
                gps.longitude      = bestpos.lon;
                gps.altitude       = bestpos.hgt;
                gps.undulation     = bestpos.undulation;
                gps.sol_status     = bestpos.sol_status;
                gps.pos_type       = bestpos.pos_type;
                gps.num_satellites = static_cast<int>(bestpos.num_svs);
                gps.num_soln_svs   = static_cast<int>(bestpos.num_soln_svs);
                gps.lat_sigma      = bestpos.lat_sigma;
                gps.lon_sigma      = bestpos.lon_sigma;
                gps.hgt_sigma      = bestpos.hgt_sigma;
                gps.diff_age       = bestpos.diff_age;
                gps.sol_age        = bestpos.sol_age;
                gps.valid          = (bestpos.sol_status == 0);  // 0 = SOL_COMPUTED (表9-48)

                // pos_type → fix_quality 映射 (参考表 9-47 位置或速度类型)
                switch (bestpos.pos_type) {
                    case 48: case 49: case 50:              // L1_INT / WIDE_INT / NARROW_INT
                    case 56:                                // INS_RTKFIXED
                        gps.fix_quality = 4; break;         // RTK 固定解
                    case 32: case 33: case 34:              // L1_FLOAT / IONOFREE_FLOAT / NARROW_FLOAT
                    case 55:                                // INS_RTKFLOAT
                        gps.fix_quality = 5; break;         // RTK 浮点解
                    case 17: case 18:                       // PSRDIFF / SBAS
                    case 54:                                // INS_PSRDIFF
                        gps.fix_quality = 2; break;         // 差分定位
                    case 1:                                 // FIXEDPOS
                    case 16:                                // SINGLE
                    case 52: case 53:                       // INS / INS_PSRSP
                        gps.fix_quality = 1; break;         // 单点定位
                    default:
                        gps.fix_quality = 0; break;         // 无效
                }

                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    // 仅更新位置相关字段, 不覆盖 heading 字段!
                    current_gps_data_.latitude       = gps.latitude;
                    current_gps_data_.longitude      = gps.longitude;
                    current_gps_data_.altitude       = gps.altitude;
                    current_gps_data_.undulation     = gps.undulation;
                    current_gps_data_.sol_status     = gps.sol_status;
                    current_gps_data_.pos_type       = gps.pos_type;
                    current_gps_data_.num_satellites  = gps.num_satellites;
                    current_gps_data_.num_soln_svs   = gps.num_soln_svs;
                    current_gps_data_.lat_sigma      = gps.lat_sigma;
                    current_gps_data_.lon_sigma      = gps.lon_sigma;
                    current_gps_data_.hgt_sigma      = gps.hgt_sigma;
                    current_gps_data_.diff_age       = gps.diff_age;
                    current_gps_data_.sol_age        = gps.sol_age;
                    current_gps_data_.fix_quality    = gps.fix_quality;
                    current_gps_data_.valid          = gps.valid;
                    bestpos_msg_count_++;
                }

                // RCLCPP_INFO(this->get_logger(),
                //     "BESTPOS: lat=%.9f lon=%.9f hgt=%.4f undulation=%.4f "
                //     "sol_status=%u pos_type=%u svs=%u/%u ext_sol=0x%02X",
                //     gps.latitude, gps.longitude, gps.altitude, gps.undulation,
                //     gps.sol_status, gps.pos_type,
                //     gps.num_soln_svs, gps.num_satellites,
                //     bestpos.ext_sol_stat);
            }

            // ===== 解析 HEADING 消息 (Message ID=971) =====
            if (header.msg_id == HEADING_MSG_ID &&
                header.msg_length >= sizeof(HeadingBody))
            {
                HeadingBody hdg;
                std::memcpy(&hdg, binary_buffer_.data() + header.header_length, sizeof(HeadingBody));

                // RCLCPP_INFO(this->get_logger(),
                //     "HEADING 原始: heading=%.2f° pitch=%.2f° baseline=%.3fm "
                //     "σ(hdg)=%.3f° sol_status=%u pos_type=%u",
                //     hdg.heading, hdg.pitch, hdg.length,
                //     hdg.hdg_std_dev, hdg.sol_status, hdg.pos_type);

                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    current_gps_data_.heading_deg     = hdg.heading;       // 0~360°, 正北顺时针
                    current_gps_data_.pitch_deg       = hdg.pitch;
                    current_gps_data_.baseline_length  = hdg.length;
                    current_gps_data_.hdg_std_dev     = hdg.hdg_std_dev;
                    current_gps_data_.hdg_sol_status  = hdg.sol_status;
                    current_gps_data_.hdg_pos_type    = hdg.pos_type;
                    current_gps_data_.heading_valid   = (hdg.sol_status == 0);  // SOL_COMPUTED
                    heading_msg_count_++;
                }
            }

            // 消费当前消息，继续处理后续数据
            binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.begin() + total_msg_len);
        }

        // 防止缓冲区无限增长 (超 8 KB 时保留末尾 1 KB)
        if (binary_buffer_.size() > 8192)
            binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.end() - 1024);
    }
}

void RtkNode::timerCallback()
{
    publishGpsData();
}

void RtkNode::convertToLocalEnu(double lat, double lon,
                                 double& x_east, double& y_north) const
{
    // 简化平面近似: 将经纬度偏移转换为 ENU 局部坐标 (米)
    // x_east  → 东向偏移 (经度差)
    // y_north → 北向偏移 (纬度差)
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kMetersPerDegreeLat = 111320.0;
    const double meters_per_degree_lon =
        111320.0 * std::cos(ref_lat_ * kDegToRad);

    x_east  = (lon - ref_lon_) * meters_per_degree_lon;
    y_north = (lat - ref_lat_) * kMetersPerDegreeLat;
}

void RtkNode::publishGpsData()
{
    // 持锁仅拷贝数据, 释放后再做 publish (序列化耗时较长, 持锁会阻塞串口读线程)
    GpsData snap;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        snap = current_gps_data_;
    }

    auto now = this->now();
    
    // -------- NavSatFix --------
    auto nav_msg = sensor_msgs::msg::NavSatFix();
    nav_msg.header.stamp    = now;
    nav_msg.header.frame_id = "gps";
    
    nav_msg.latitude  = snap.latitude;
    nav_msg.longitude = snap.longitude;
    nav_msg.altitude  = snap.altitude;
    
    // pos_type → NavSatStatus 映射 (参考表 9-47)
    switch (snap.pos_type) {
        case 48: case 49: case 50: case 56:         // L1_INT / WIDE_INT / NARROW_INT / INS_RTKFIXED
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            break;
        case 32: case 33: case 34: case 55:         // L1_FLOAT / IONOFREE_FLOAT / NARROW_FLOAT / INS_RTKFLOAT
        case 17: case 18: case 54:                  // PSRDIFF / SBAS / INS_PSRDIFF
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
            break;
        case 1: case 16: case 52: case 53:          // FIXEDPOS / SINGLE / INS / INS_PSRSP
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            break;
        default:
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    
    nav_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS     |
                             sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS  |
                             sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO  |
                             sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    
    // 使用 BESTPOS sigma 直接填充对角协方差 (单位: m²)
    double cov_lat = static_cast<double>(snap.lat_sigma) * snap.lat_sigma;
    double cov_lon = static_cast<double>(snap.lon_sigma) * snap.lon_sigma;
    double cov_hgt = static_cast<double>(snap.hgt_sigma) * snap.hgt_sigma;
    nav_msg.position_covariance[0] = cov_lat;
    nav_msg.position_covariance[4] = cov_lon;
    nav_msg.position_covariance[8] = cov_hgt;
    nav_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    
    nav_sat_pub_->publish(nav_msg);
    
    // -------- TwistStamped (BESTPOS 不含速度信息，置零) --------
    auto vel_msg = geometry_msgs::msg::TwistStamped();
    vel_msg.header.stamp    = now;
    vel_msg.header.frame_id = "gps";
    vel_msg.twist.linear.x  = 0.0;
    vel_msg.twist.linear.y  = 0.0;
    vel_msg.twist.linear.z  = 0.0;
    velocity_pub_->publish(vel_msg);
    
    // -------- Odometry (基于济南参考点的 ENU 局部坐标) --------
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp    = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link";

    double x_east = 0.0, y_north = 0.0;
    if (snap.valid) {
        convertToLocalEnu(snap.latitude,
                          snap.longitude,
                          x_east, y_north);
    }

    odom_msg.pose.pose.position.x = x_east;    // 东向 (m)
    odom_msg.pose.pose.position.y = y_north;    // 北向 (m)
    odom_msg.pose.pose.position.z = snap.altitude;

    // RTK HEADING 航向: 正北顺时针 → ENU 约定 (yaw=0 朝东, 逆时针为正)
    // 1) 先补偿天线安装偏移: heading_corrected = heading_raw + offset
    // 2) 再转 ENU: yaw_enu = π/2 - heading_corrected_rad
    if (snap.heading_valid) {
        double heading_corrected = snap.heading_deg + antenna_heading_offset_deg_;
        double heading_rad = heading_corrected * M_PI / 180.0;
        double yaw_enu = M_PI / 2.0 - heading_rad;
        // 归一化到 [-π, π]
        while (yaw_enu >  M_PI) yaw_enu -= 2.0 * M_PI;
        while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;

        // 填充 odom 四元数 (仅绕 Z 轴旋转)
        odom_msg.pose.pose.orientation.w = std::cos(yaw_enu / 2.0);
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(yaw_enu / 2.0);

        // 航向协方差：用标准偏差换算 (deg→rad)²
        double hdg_cov = (snap.hdg_std_dev * M_PI / 180.0);
        hdg_cov *= hdg_cov;
        odom_msg.pose.covariance[35] = hdg_cov;   // yaw 协方差

        // 发布 heading 话题 (ENU yaw, 弧度)
        auto hdg_msg = std_msgs::msg::Float64();
        hdg_msg.data = yaw_enu;
        heading_pub_->publish(hdg_msg);
    } else {
        odom_msg.pose.pose.orientation.w = 1.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
    }

    // 位置协方差 (对角, 单位 m²)
    odom_msg.pose.covariance[0]  = cov_lat;   // x
    odom_msg.pose.covariance[7]  = cov_lon;   // y
    odom_msg.pose.covariance[14] = cov_hgt;   // z
    // roll/pitch 协方差设为极大 (无测量)
    odom_msg.pose.covariance[21] = 9999.0;
    odom_msg.pose.covariance[28] = 9999.0;
    // yaw 协方差: 有 heading 时已在上方填充, 无 heading 时设为极大
    if (!snap.heading_valid) {
        odom_msg.pose.covariance[35] = 9999.0;
    }

    odom_pub_->publish(odom_msg);
    outdoor_odom_pub_->publish(odom_msg);
    
    // 降频日志 (每 5 次输出一次)
    static int log_counter = 0;
    if (++log_counter >= 5) {
        log_counter = 0;
        RCLCPP_INFO(this->get_logger(),
            "BESTPOS: lat=%.9f lon=%.9f hgt=%.4fm "
            "| sol=%u pos_type=%u svs=%u/%u "
            "| σ(lat=%.4f lon=%.4f hgt=%.4f)m diff_age=%.1fs sol_age=%.1fs",
            snap.latitude, snap.longitude, snap.altitude,
            snap.sol_status, snap.pos_type,
            snap.num_soln_svs, snap.num_satellites,
            snap.lat_sigma, snap.lon_sigma, snap.hgt_sigma,
            snap.diff_age, snap.sol_age);

        if (snap.heading_valid) {
            double hdg_corrected = snap.heading_deg + antenna_heading_offset_deg_;
            double yaw_enu = M_PI / 2.0 - hdg_corrected * M_PI / 180.0;
            while (yaw_enu >  M_PI) yaw_enu -= 2.0 * M_PI;
            while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;
            RCLCPP_INFO(this->get_logger(),
                "HEADING: raw=%.2f° +offset=%.1f° → corrected=%.2f° → ENU_yaw=%.4f rad (%.2f°)"
                " | pitch=%.2f° | baseline=%.3fm | σ(hdg=%.3f°) | sol=%u pos_type=%u",
                snap.heading_deg, antenna_heading_offset_deg_, hdg_corrected,
                yaw_enu, yaw_enu * 180.0 / M_PI,
                snap.pitch_deg,
                snap.baseline_length, snap.hdg_std_dev,
                snap.hdg_sol_status, snap.hdg_pos_type);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "HEADING: 无有效航向 (sol_status=%u) | HEADING消息总收到=%u, BESTPOS总收到=%u",
                snap.hdg_sol_status,
                heading_msg_count_, bestpos_msg_count_);
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

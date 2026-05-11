#include "rtk/rtk_node.hpp"

namespace rtk
{

// ==================== NMEA 工具函数 ====================

// 校验 NMEA checksum: XOR $...*XX 中所有字节
bool RtkNode::VerifyNmeaChecksum(const std::string& sentence) const
{
    size_t star = sentence.rfind('*');
    if (star == std::string::npos || star + 2 >= sentence.size()) return false;
    uint8_t computed = 0;
    for (size_t i = 1; i < star; i++)  // 跳过 '$'
        computed ^= static_cast<uint8_t>(sentence[i]);
    try {
        uint8_t received = static_cast<uint8_t>(
            std::stoul(sentence.substr(star + 1, 2), nullptr, 16));
        return computed == received;
    } catch (...) {
        return false;
    }
}

// 切割 NMEA 语句：去掉 *XX checksum 为果后按逗号分割
std::vector<std::string> RtkNode::SplitNmea(const std::string& sentence) const
{
    std::string s = sentence;
    size_t star = s.find('*');
    if (star != std::string::npos) s = s.substr(0, star);
    std::vector<std::string> fields;
    std::stringstream ss(s);
    std::string field;
    while (std::getline(ss, field, ','))
        fields.push_back(field);
    return fields;
}

// 解析 NMEA 纬度/经度 (DDMM.MMMMM 或 DDDMM.MMMMM)
double RtkNode::ParseNmeaLatLon(const std::string& value, const std::string& dir) const
{
    if (value.empty()) return 0.0;
    double v        = std::stod(value);
    double degrees  = std::floor(v / 100.0);
    double minutes  = v - degrees * 100.0;
    double result   = degrees + minutes / 60.0;
    if (dir == "S" || dir == "W") result = -result;
    return result;
}

// ==================== NMEA 语句解析器 ====================

// $GNGGA / $GPGGA —— 位置、定位质量、卫星数、HDOP、高度、大地水准面差距
void RtkNode::ParseGga(const std::vector<std::string>& fields)
{
    // idx: 0=$GNGGA, 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W,
    //      6=fix, 7=svs, 8=hdop, 9=alt, 10=M, 11=geoid, 12=M, 13=age, 14=stn*cs
    if (fields.size() < 10) return;
    int fix_q = fields[6].empty() ? 0 : std::stoi(fields[6]);

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_gps_data_.fix_quality    = fix_q;
    current_gps_data_.valid          = (fix_q > 0);
    current_gps_data_.num_satellites = fields[7].empty() ? 0 : std::stoi(fields[7]);
    current_gps_data_.hdop           = fields[8].empty() ? 99.9f : std::stof(fields[8]);
    current_gps_data_.altitude       = fields[9].empty() ? 0.0 : std::stod(fields[9]);
    if (fields.size() > 11 && !fields[11].empty())
        current_gps_data_.undulation = std::stof(fields[11]);
    if (fields.size() > 13 && !fields[13].empty())
        current_gps_data_.diff_age   = std::stof(fields[13]);
    if (!fields[2].empty() && !fields[3].empty() &&
        !fields[4].empty() && !fields[5].empty()) {
        current_gps_data_.latitude  = ParseNmeaLatLon(fields[2], fields[3]);
        current_gps_data_.longitude = ParseNmeaLatLon(fields[4], fields[5]);
    }
    // 无 $GST 时用 HDOP×5 估算 sigma；$GST 到来后会覆盖
    float sigma_xy = current_gps_data_.hdop * 5.0f;
    current_gps_data_.lat_sigma = sigma_xy;
    current_gps_data_.lon_sigma = sigma_xy;
    gga_msg_count_++;
}

// $GNHPR —— 双天线航向/俯仰/横滚 (新协议: 东向为0, 顺时针 0=E,90=S,180=W,270=N)
void RtkNode::ParseHpr(const std::vector<std::string>& fields)
{
    // $GNHPR,time,heading,pitch,roll,status,svs,hdop,baseline*cs
    // idx: 0=$GNHPR, 1=time, 2=heading(0~360°,顺时针自正东), 3=pitch, 4=roll,
    //      5=status, 6=svs, 7=hdop, 8=baseline*cs
    if (fields.size() < 3 || fields[2].empty()) return;
    float heading = std::stof(fields[2]);
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_gps_data_.heading_deg   = heading;
    current_gps_data_.hdg_std_dev   = 0.5f;  // HPR 无标准差字段，使用固定估计值 0.5°
    current_gps_data_.heading_valid = true;
    hdt_msg_count_++;
}

// $GNGST / $GPGST —— 位置精度统计 (sigma)
void RtkNode::ParseGst(const std::vector<std::string>& fields)
{
    // idx: 0=$GNGST, 1=time, 2=rms, 3=semi-major, 4=semi-minor,
    //      5=orient, 6=lat-sig, 7=lon-sig, 8=hgt-sig*cs
    if (fields.size() < 9) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!fields[6].empty()) current_gps_data_.lat_sigma = std::stof(fields[6]);
    if (!fields[7].empty()) current_gps_data_.lon_sigma = std::stof(fields[7]);
    if (!fields[8].empty()) current_gps_data_.hgt_sigma = std::stof(fields[8]);
}

// $GNRMC / $GPRMC —— 地速 (knots → m/s)
void RtkNode::ParseRmc(const std::vector<std::string>& fields)
{
    // idx: 0=$GNRMC, 1=time, 2=status(A/V), 3=lat, 4=N/S, 5=lon, 6=E/W,
    //      7=speed_knots, 8=cog, 9=date ...
    if (fields.size() < 8 || fields[2] != "A") return;  // A=有效
    if (!fields[7].empty()) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_gps_data_.speed = std::stof(fields[7]) * 0.5144444;  // knots → m/s
    }
}

// 校验 + 分发 NMEA 行
void RtkNode::ProcessNmeaLine(const std::string& line)
{
    if (line.empty() || line[0] != '$') return;
    if (!VerifyNmeaChecksum(line)) {
        RCLCPP_DEBUG(this->get_logger(), "NMEA checksum 失败: %s", line.c_str());
        return;
    }
    auto fields = SplitNmea(line);
    if (fields.empty()) return;
    const std::string& id = fields[0];
    if (id.size() >= 3) {
        const std::string suffix = id.substr(id.size() - 3);
        if      (suffix == "GGA" && fields.size() >= 10) ParseGga(fields);
        else if (suffix == "HPR" && fields.size() >= 3)  ParseHpr(fields);
        else if (suffix == "GST" && fields.size() >= 9)  ParseGst(fields);
        else if (suffix == "RMC" && fields.size() >= 8)  ParseRmc(fields);
        else if (suffix == "HDT" && fields.size() >= 2)  ParseHpr(fields);  // 兼容 $GNHDT
    }
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
    this->declare_parameter<std::string>("imu_topic", "/vru/imu_raw");
    this->declare_parameter<std::string>("chassis_feedback_topic", "/chassis/feedback");
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
    std::string imu_topic              = this->get_parameter("imu_topic").as_string();
    std::string chassis_feedback_topic  = this->get_parameter("chassis_feedback_topic").as_string();
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

    // 订阅 VRU IMU 用于 EKF 预测 (陀螺仪 gz → 偏航角速率)
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 50,
        std::bind(&RtkNode::ImuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "IMU 融合话题: %s", imu_topic.c_str());

    // 订阅底盘速度反馈 (轮速替换 GPS 差分 vx)
    chassis_feedback_sub_ = this->create_subscription<std_msgs::msg::String>(
        chassis_feedback_topic, 10,
        std::bind(&RtkNode::ChassisFeedbackCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "底盘反馈话题: %s", chassis_feedback_topic.c_str());
    
    // 预分配 NMEA 行缓冲区
    nmea_line_buf_.reserve(512);
    
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
    
    RCLCPP_INFO(this->get_logger(), "RTK Node 初始化完成，等待 NMEA 0183 语句 ($GNGGA / $GNHDT / $GNGST) ...");
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

// ==================== EKF 实现 ====================

void RtkNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(ekf_mutex_);
    if (!ekf_.initialized) return;

    rclcpp::Time stamp(msg->header.stamp);
    double dt = (stamp - ekf_.last_predict_stamp).seconds();
    if (dt <= 0.0 || dt > 0.5) {
        ekf_.last_predict_stamp = stamp;
        return;
    }
    EkfPredict(msg->angular_velocity.z, dt);
    ekf_.last_predict_stamp = stamp;
}

// 预测步骤：常速度模型 + 陀螺仪偏航率
void RtkNode::EkfPredict(double gz, double dt)
{
    double cx = std::cos(ekf_.yaw);
    double sx = std::sin(ekf_.yaw);
    double vx = ekf_.vx;

    // 状态预测
    ekf_.x   += vx * cx * dt;
    ekf_.y   += vx * sx * dt;
    ekf_.yaw += gz * dt;
    while (ekf_.yaw >  M_PI) ekf_.yaw -= 2.0 * M_PI;
    while (ekf_.yaw < -M_PI) ekf_.yaw += 2.0 * M_PI;
    // vx 保持不变 (常速度模型)

    // Jacobian F = df/dX (行主序 4x4)
    double F[16] = {
        1, 0, -vx * sx * dt,  cx * dt,
        0, 1,  vx * cx * dt,  sx * dt,
        0, 0,  1,             0,
        0, 0,  0,             1
    };

    // 过程噪声 Q
    const double q_pos = 0.05 * dt;
    const double q_yaw = 0.01 * dt;
    const double q_vx  = 0.5  * dt;

    // P = F*P*F^T + Q
    double FP[16] = {0}, newP[16] = {0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                FP[i*4+j] += F[i*4+k] * ekf_.P[k*4+j];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                newP[i*4+j] += FP[i*4+k] * F[j*4+k];  // F[j*4+k] = F^T[k*4+j]
    newP[0]  += q_pos;
    newP[5]  += q_pos;
    newP[10] += q_yaw;
    newP[15] += q_vx;
    std::memcpy(ekf_.P, newP, sizeof(newP));
}

// GPS 位置测量更新 (H = [1 0 0 0; 0 1 0 0])
void RtkNode::EkfUpdatePos(double x_gps, double y_gps, double r)
{
    double innov_x = x_gps - ekf_.x;
    double innov_y = y_gps - ekf_.y;

    // S = H*P*H^T + R (2x2)
    double S[4] = {ekf_.P[0] + r, ekf_.P[1],
                   ekf_.P[4],     ekf_.P[5] + r};
    double det = S[0]*S[3] - S[1]*S[2];
    if (std::fabs(det) < 1e-9) return;
    double Si[4] = {S[3]/det, -S[1]/det, -S[2]/det, S[0]/det};

    // K = P*H^T * Si  (4x2): P*H^T = 前两列 P
    double K[8];
    for (int i = 0; i < 4; i++) {
        K[i*2]   = ekf_.P[i*4+0]*Si[0] + ekf_.P[i*4+1]*Si[2];
        K[i*2+1] = ekf_.P[i*4+0]*Si[1] + ekf_.P[i*4+1]*Si[3];
    }

    // 状态更新
    ekf_.x   += K[0]*innov_x + K[1]*innov_y;
    ekf_.y   += K[2]*innov_x + K[3]*innov_y;
    ekf_.yaw += K[4]*innov_x + K[5]*innov_y;
    ekf_.vx  += K[6]*innov_x + K[7]*innov_y;

    // P = (I - K*H)*P = P - K*(P[row0]; P[row1])
    double r0[4], r1[4];
    for (int j = 0; j < 4; j++) { r0[j] = ekf_.P[j]; r1[j] = ekf_.P[4+j]; }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            ekf_.P[i*4+j] -= K[i*2]*r0[j] + K[i*2+1]*r1[j];

    // 对称化防数值漂移
    for (int i = 0; i < 4; i++) for (int j = i+1; j < 4; j++)
        ekf_.P[i*4+j] = ekf_.P[j*4+i] = 0.5*(ekf_.P[i*4+j] + ekf_.P[j*4+i]);
    for (int i = 0; i < 4; i++) if (ekf_.P[i*4+i] < 1e-6) ekf_.P[i*4+i] = 1e-6;
}

// GPS 航向测量更新 (H = [0 0 1 0])
void RtkNode::EkfUpdateYaw(double yaw_gps, double r_yaw)
{
    double innov = yaw_gps - ekf_.yaw;
    while (innov >  M_PI) innov -= 2.0 * M_PI;
    while (innov < -M_PI) innov += 2.0 * M_PI;

    double S = ekf_.P[10] + r_yaw;
    if (std::fabs(S) < 1e-9) return;

    // K = P[:,2] / S  (4x1)
    double K[4] = {ekf_.P[2]/S, ekf_.P[6]/S, ekf_.P[10]/S, ekf_.P[14]/S};

    ekf_.x   += K[0]*innov;
    ekf_.y   += K[1]*innov;
    ekf_.yaw += K[2]*innov;
    ekf_.vx  += K[3]*innov;

    // P = P - K*(P[row2])
    double r2[4];
    for (int j = 0; j < 4; j++) r2[j] = ekf_.P[2*4+j];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            ekf_.P[i*4+j] -= K[i] * r2[j];

    for (int i = 0; i < 4; i++) for (int j = i+1; j < 4; j++)
        ekf_.P[i*4+j] = ekf_.P[j*4+i] = 0.5*(ekf_.P[i*4+j] + ekf_.P[j*4+i]);
    for (int i = 0; i < 4; i++) if (ekf_.P[i*4+i] < 1e-6) ekf_.P[i*4+i] = 1e-6;
}

// 底盘速度反馈回调 — 直接写入 chassis_vx_，EKF 每次 GPS 更新时注入
void RtkNode::ChassisFeedbackCallback(const std_msgs::msg::String::SharedPtr msg)
{
    try {
        auto j = nlohmann::json::parse(msg->data);
        chassis_vx_ = std::abs(j.value("vx", 0.0));  // 取绝对值, 前向速度恒正
    } catch (...) {}
}

void RtkNode::serialReadThread()
{
    char read_buf[512];

    while (running_) {
        if (serial_fd_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        ssize_t bytes_read = read(serial_fd_, read_buf, sizeof(read_buf));

        if (bytes_read > 0) {
            nmea_line_buf_.append(read_buf, static_cast<size_t>(bytes_read));
        } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN(this->get_logger(), "串口读取错误: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 提取完整的 NMEA 行（以 \n 结尾）
        size_t pos;
        while ((pos = nmea_line_buf_.find('\n')) != std::string::npos) {
            std::string line = nmea_line_buf_.substr(0, pos);
            nmea_line_buf_.erase(0, pos + 1);
            // 去掉尾部 \r
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) {
                try { ProcessNmeaLine(line); }
                catch (const std::exception& e) {
                    RCLCPP_DEBUG(this->get_logger(), "NMEA 解析异常: %s", e.what());
                }
            }
        }

        // 防止缓冲区无限增长
        if (nmea_line_buf_.size() > 4096) nmea_line_buf_.clear();
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
    
    // fix_quality → NavSatStatus 映射 (NMEA GGA field 6)
    switch (snap.fix_quality) {
        case 4:  // RTK 固定解
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            break;
        case 5:  // RTK 浮点解
        case 2:  // 差分定位
            nav_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
            break;
        case 1:  // 单点定位
        case 6:  // 航位推算
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
    
    // -------- TwistStamped (RMC 速度，没有 RMC 时置零) --------
    auto vel_msg = geometry_msgs::msg::TwistStamped();
    vel_msg.header.stamp    = now;
    vel_msg.header.frame_id = "gps";
    vel_msg.twist.linear.x  = snap.speed;
    vel_msg.twist.linear.y  = 0.0;
    vel_msg.twist.linear.z  = 0.0;
    velocity_pub_->publish(vel_msg);
    
    // -------- ENU 位置计算 --------
    double x_east = 0.0, y_north = 0.0;
    if (snap.valid) {
        convertToLocalEnu(snap.latitude, snap.longitude, x_east, y_north);
    }

    // -------- ENU 航向计算 --------
    double yaw_enu = 0.0;
    bool   hdg_ok  = false;
    double hdg_cov = 9999.0;
    if (snap.heading_valid) {
        double heading_corrected = snap.heading_deg + antenna_heading_offset_deg_;
        double heading_rad = heading_corrected * M_PI / 180.0;
        // 设备约定 (新协议): 顺时针自正东 (0=E, 90=S, 180=W, 270=N)
        // ENU yaw 逆时针自正东: yaw = -heading_rad
        yaw_enu = -heading_rad;
        while (yaw_enu >  M_PI) yaw_enu -= 2.0 * M_PI;
        while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;
        hdg_ok = true;
        double s = snap.hdg_std_dev * M_PI / 180.0;
        hdg_cov = s * s;
        auto hdg_msg = std_msgs::msg::Float64();
        hdg_msg.data = yaw_enu;
        heading_pub_->publish(hdg_msg);
    }

    // -------- EKF GPS+IMU 融合 --------
    {
        std::lock_guard<std::mutex> lk(ekf_mutex_);
        if (!ekf_.initialized && snap.valid) {
            // 首次有效 GPS，初始化 EKF
            ekf_.x    = x_east;
            ekf_.y    = y_north;
            ekf_.yaw  = hdg_ok ? yaw_enu : 0.0;
            ekf_.vx   = 0.0;
            double r  = std::max(static_cast<double>(snap.lat_sigma), 0.3);
            double p0[16] = {r*r,0,0,0, 0,r*r,0,0,
                             0,0,(hdg_ok ? hdg_cov*4 : 1.0),0,
                             0,0,0,1.0};
            std::memcpy(ekf_.P, p0, sizeof(p0));
            ekf_.vx                 = chassis_vx_;
            ekf_.last_predict_stamp = now;
            ekf_.initialized        = true;
        } else if (ekf_.initialized && snap.valid) {
            // GPS 位置测量更新
            double r_xy = std::max(static_cast<double>(snap.lat_sigma), 0.15);
            EkfUpdatePos(x_east, y_north, r_xy * r_xy);

            // 注入底盘轮速 (低方差直接写入, 不需要 GPS 差分)
            ekf_.vx = chassis_vx_;
            if (ekf_.P[15] > 0.01) ekf_.P[15] = 0.01;  // 收紧 vx 协方差

            // GPS 航向测量更新
            if (hdg_ok) EkfUpdateYaw(yaw_enu, hdg_cov);
        }

        // 用 EKF 状态替换原始 GPS 值输出
        if (ekf_.initialized) {
            x_east  = ekf_.x;
            y_north = ekf_.y;
            if (!hdg_ok) yaw_enu = ekf_.yaw;
        }
    }

    // -------- Odometry (EKF 融合后输出) --------
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp    = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link";

    odom_msg.pose.pose.position.x = x_east;
    odom_msg.pose.pose.position.y = y_north;
    odom_msg.pose.pose.position.z = snap.altitude;

    odom_msg.pose.pose.orientation.w = std::cos(yaw_enu / 2.0);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(yaw_enu / 2.0);

    // 协方差 (原始 GPS sigma, 仅供参考)
    odom_msg.pose.covariance[0]  = cov_lat;
    odom_msg.pose.covariance[7]  = cov_lon;
    odom_msg.pose.covariance[14] = cov_hgt;
    odom_msg.pose.covariance[21] = 9999.0;
    odom_msg.pose.covariance[28] = 9999.0;
    odom_msg.pose.covariance[35] = hdg_ok ? hdg_cov : 9999.0;

    odom_pub_->publish(odom_msg);
    outdoor_odom_pub_->publish(odom_msg);
    
    // 降频日志 (每 5 次输出一次)
    static int log_counter = 0;
    if (++log_counter >= 5) {
        log_counter = 0;
        RCLCPP_INFO(this->get_logger(),
            "GGA: lat=%.9f lon=%.9f hgt=%.4fm "
            "| fix=%d svs=%d hdop=%.2f "
            "| σ(lat=%.4f lon=%.4f hgt=%.4f)m diff_age=%.1fs",
            snap.latitude, snap.longitude, snap.altitude,
            snap.fix_quality, snap.num_satellites, snap.hdop,
            snap.lat_sigma, snap.lon_sigma, snap.hgt_sigma,
            snap.diff_age);

        if (snap.heading_valid) {
            double hdg_corrected = snap.heading_deg + antenna_heading_offset_deg_;
            double yaw_enu_log = -hdg_corrected * M_PI / 180.0;
            while (yaw_enu_log >  M_PI) yaw_enu_log -= 2.0 * M_PI;
            while (yaw_enu_log < -M_PI) yaw_enu_log += 2.0 * M_PI;
            RCLCPP_INFO(this->get_logger(),
                "HDT: raw=%.2f° +offset=%.1f° → corrected=%.2f° → ENU_yaw=%.4f rad (%.2f°)"
                " | σ(hdg=%.3f°) | HDT总数=%u",
                snap.heading_deg, antenna_heading_offset_deg_, hdg_corrected,
                yaw_enu_log, yaw_enu_log * 180.0 / M_PI,
                snap.hdg_std_dev, hdt_msg_count_);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "HEADING: 无有效航向 | HDT语句总收到=%u, GGA语句总收到=%u",
                hdt_msg_count_, gga_msg_count_);
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

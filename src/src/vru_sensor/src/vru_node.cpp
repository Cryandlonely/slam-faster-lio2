#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>

/**
 * VRU620/621PRC-232 串口驱动节点
 *
 * 报文格式（手册 5.2 节）：
 *   [0x7A][0x7B][字节数][功能字节][数据体...][校验字节][0xBB]
 *
 * 功能字节：
 *   0xB0 — 原始 ADC（加速度+角速度，各 WORD）
 *   0xB1 — 解算姿态角（yaw/pitch/roll/tilt 各 3 字节 + 解算速率 2 字节）
 *
 * 发布话题：
 *   /vru/attitude  geometry_msgs/Vector3Stamped  (x=pitch, y=roll, z=yaw)  单位：度
 *   /vru/tilt      std_msgs/Float64               倾斜角，单位：度
 *   /vru/imu_raw   sensor_msgs/Imu                加速度+角速度（原始 ADC 换算后）
 */

#include <std_msgs/msg/float64.hpp>

static speed_t baud_from_int(int baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B115200;
    }
}

// 打开串口并配置波特率
static int open_serial(const std::string & port, int baud)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return fd;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    speed_t spd = baud_from_int(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 位数据
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);            // 无奇偶校验
    tty.c_cflag &= ~CSTOPB;                       // 1 停止位
    tty.c_cflag &= ~CRTSCTS;                      // 无硬件流控
    tty.c_iflag = IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;  // 0.1s 超时

    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

// 从 3 字节有符号数（首字节最高位为符号位）解码角度值
// 分辨率 0.0001 deg，返回单位：度
static double decode_angle_3byte(const uint8_t * p)
{
    bool negative = (p[0] & 0x80) != 0;
    uint32_t raw = ((uint32_t)(p[0] & 0x7F) << 16) |
                   ((uint32_t)p[1] << 8) |
                    (uint32_t)p[2];
    double val = raw * 0.0001;
    return negative ? -val : val;
}

// 从 2 字节有符号数（首字节最高位为符号位）解码 ADC 值，返回物理量
static double decode_word_signed(const uint8_t * p, double resolution)
{
    bool negative = (p[0] & 0x80) != 0;
    uint16_t raw = ((uint16_t)(p[0] & 0x7F) << 8) | p[1];
    double val = raw * resolution;
    return negative ? -val : val;
}

// 计算异或校验（除起始字节和结束字节之外的所有字节）
static uint8_t calc_checksum(const std::vector<uint8_t> & buf, size_t start, size_t end)
{
    uint8_t xor_val = 0;
    for (size_t i = start; i < end; ++i) xor_val ^= buf[i];
    return xor_val;
}

// -----------------------------------------------------------------------

class VruNode : public rclcpp::Node
{
public:
    VruNode() : Node("vru_node")
    {
        declare_parameter("port", std::string("/dev/ttyUSB0"));
        declare_parameter("baud", 115200);
        declare_parameter("frame_id", std::string("vru"));
        // 加速度量程：0=±2g 1=±4g 2=±8g 3=±16g
        declare_parameter("accel_range", 0);
        // 角速度量程：0=±250 1=±500 2=±1000 3=±2000 dps
        declare_parameter("gyro_range", 0);

        port_     = get_parameter("port").as_string();
        baud_     = get_parameter("baud").as_int();
        frame_id_ = get_parameter("frame_id").as_string();
        int ar    = get_parameter("accel_range").as_int();
        int gr    = get_parameter("gyro_range").as_int();

        // 加速度分辨率（g/LSB）
        const double accel_res_table[] = {1.0/16384, 1.0/8192, 1.0/4096, 1.0/2048};
        // 角速度分辨率（dps/LSB）
        const double gyro_res_table[]  = {125.0/16384, 125.0/8192, 125.0/4096, 125.0/2048};
        accel_res_ = accel_res_table[ar < 4 ? ar : 0];
        gyro_res_  = gyro_res_table [gr < 4 ? gr : 0];

        pub_attitude_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/vru/attitude", 10);
        pub_tilt_     = create_publisher<std_msgs::msg::Float64>("/vru/tilt", 10);
        pub_imu_      = create_publisher<sensor_msgs::msg::Imu>("/vru/imu_raw", 10);

        fd_ = open_serial(port_, baud_);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "无法打开串口 %s: %s", port_.c_str(), strerror(errno));
        } else {
            RCLCPP_INFO(get_logger(), "串口 %s 已打开，波特率 %d", port_.c_str(), baud_);
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&VruNode::read_loop, this));
    }

    ~VruNode() { if (fd_ >= 0) close(fd_); }

private:
    void read_loop()
    {
        if (fd_ < 0) return;

        uint8_t tmp[256];
        ssize_t n = read(fd_, tmp, sizeof(tmp));
        if (n <= 0) return;

        for (ssize_t i = 0; i < n; ++i) buf_.push_back(tmp[i]);

        parse_frames();
    }

    void parse_frames()
    {
        // 扫描帧头 0x7A 0x7B
        while (buf_.size() >= 2) {
            if (buf_[0] != 0x7A || buf_[1] != 0x7B) {
                buf_.erase(buf_.begin());
                continue;
            }
            // 至少需要：2(start)+1(cnt)+1(func)+n(data)+1(crc)+1(end)
            if (buf_.size() < 5) break;

            // 字节数字段：从 func 字节到 end 字节（含 crc+end，不含 start）
            uint8_t byte_count = buf_[2];
            // 完整帧长度 = 2(start) + byte_count
            size_t frame_len = 2u + byte_count;
            if (buf_.size() < frame_len) break;

            // 校验：对 buf_[2] .. buf_[frame_len-2] 求异或，与 buf_[frame_len-2] 比较
            // 注：校验字节在倒数第2位，结束字节 0xBB 在最后
            if (buf_[frame_len - 1] != 0xBB) {
                // 结束字节不对，丢弃帧头重新同步
                buf_.erase(buf_.begin());
                continue;
            }

            // checksum 覆盖范围：buf_[2] .. buf_[frame_len-3]（含字节数和功能字节）
            uint8_t crc = calc_checksum(buf_, 2, frame_len - 2);
            if (crc != buf_[frame_len - 2]) {
                RCLCPP_WARN(get_logger(), "校验失败, 丢弃帧");
                buf_.erase(buf_.begin());
                continue;
            }

            uint8_t func = buf_[3];
            // 数据体起止：buf_[4] .. buf_[frame_len-3]
            std::vector<uint8_t> data(buf_.begin() + 4, buf_.begin() + frame_len - 2);

            if (func == 0xB1) {
                handle_attitude(data);
            } else if (func == 0xB0) {
                handle_adc(data);
            }

            buf_.erase(buf_.begin(), buf_.begin() + frame_len);
        }

        // 防止缓冲区无限增长
        if (buf_.size() > 1024) buf_.erase(buf_.begin(), buf_.begin() + 512);
    }

    // 功能字节 0xB1：解算姿态角
    void handle_attitude(const std::vector<uint8_t> & data)
    {
        // VRU620 无 yaw，数据体从 pitch 开始（12 字节）
        // VRU621 有 yaw，数据体 15 字节（yaw 3 + pitch 3 + roll 3 + tilt 3 + rate 2 = 14... 手册示例帧 0x12=18 含 CRC+END）
        // 根据数据长度判断：有 yaw 则 data.size() >= 14，无 yaw 则 >= 11
        // 手册表13：0=yaw(3) 3=pitch(3) 6=roll(3) 9=tilt(3) 12=rate(2) → 14 字节
        //           VRU620  0=pitch(3) 3=roll(3) 6=tilt(3) 9=rate(2)  → 11 字节

        auto stamp = now();
        bool has_yaw = (data.size() >= 14);
        size_t offset = 0;

        double yaw = 0.0;
        if (has_yaw) {
            yaw    = decode_angle_3byte(data.data() + offset); offset += 3;
        }
        if (data.size() < offset + 11) return;  // 数据不足

        double pitch = decode_angle_3byte(data.data() + offset); offset += 3;
        double roll  = decode_angle_3byte(data.data() + offset); offset += 3;

        // 倾斜角：原始值为弧度，乘以 57.3 转角度（手册 5.2.4）
        uint32_t tilt_raw_u = ((uint32_t)(data[offset] & 0x7F) << 16) |
                               ((uint32_t)data[offset+1] << 8) |
                                (uint32_t)data[offset+2];
        bool tilt_neg = (data[offset] & 0x80) != 0;
        // 倾斜角也使用 0.0001 分辨率单位（弧度），再 *57.3 → 度
        double tilt_rad = tilt_raw_u * 0.0001;
        double tilt_deg = (tilt_neg ? -tilt_rad : tilt_rad) * 57.3;
        offset += 3;

        // 发布 attitude
        geometry_msgs::msg::Vector3Stamped att_msg;
        att_msg.header.stamp    = stamp;
        att_msg.header.frame_id = frame_id_;
        att_msg.vector.x = pitch;
        att_msg.vector.y = roll;
        att_msg.vector.z = yaw;
        pub_attitude_->publish(att_msg);

        // 发布 tilt
        std_msgs::msg::Float64 tilt_msg;
        tilt_msg.data = tilt_deg;
        pub_tilt_->publish(tilt_msg);

        RCLCPP_INFO(get_logger(),
            "[attitude] pitch=%.4f°  roll=%.4f°  yaw=%.4f°  tilt=%.4f°",
            pitch, roll, yaw, tilt_deg);
    }

    // 功能字节 0xB0：原始 ADC（加速度+角速度）
    void handle_adc(const std::vector<uint8_t> & data)
    {
        if (data.size() < 12) return;

        // 每个量 2 字节 WORD，顺序：ax ay az gx gy gz
        double ax = decode_word_signed(data.data() + 0,  accel_res_) * 9.80665;  // m/s²
        double ay = decode_word_signed(data.data() + 2,  accel_res_) * 9.80665;
        double az = decode_word_signed(data.data() + 4,  accel_res_) * 9.80665;
        double gx = decode_word_signed(data.data() + 6,  gyro_res_)  * M_PI / 180.0;  // rad/s
        double gy = decode_word_signed(data.data() + 8,  gyro_res_)  * M_PI / 180.0;
        double gz = decode_word_signed(data.data() + 10, gyro_res_)  * M_PI / 180.0;

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp    = now();
        imu_msg.header.frame_id = frame_id_;
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;
        // 协方差设为 -1 表示未知
        imu_msg.orientation_covariance[0] = -1;
        pub_imu_->publish(imu_msg);

        RCLCPP_INFO(get_logger(),
            "[imu_raw] ax=%.4f ay=%.4f az=%.4f m/s²  gx=%.4f gy=%.4f gz=%.4f rad/s",
            ax, ay, az, gx, gy, gz);
    }

    int fd_{-1};
    std::vector<uint8_t> buf_;
    std::string port_;
    int baud_;
    std::string frame_id_;
    double accel_res_{1.0/16384};
    double gyro_res_{125.0/16384};

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_attitude_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             pub_tilt_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VruNode>());
    rclcpp::shutdown();
    return 0;
}

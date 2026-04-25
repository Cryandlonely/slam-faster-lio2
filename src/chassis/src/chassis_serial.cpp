#include "chassis/chassis_serial.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>

// ---------------------------------------------------------------------------
// 构造 / 析构
// ---------------------------------------------------------------------------

ChassisSerial::ChassisSerial(const std::string& port) : port_(port) {}

ChassisSerial::~ChassisSerial() {
    stopReceiving();
    close();
}

// ---------------------------------------------------------------------------
// 打开 / 关闭
// ---------------------------------------------------------------------------

bool ChassisSerial::open() {
    if (isOpen()) return true;

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::cerr << "[ChassisSerial] 打开串口失败 (" << port_
                  << "): " << std::strerror(errno) << "\n";
        return false;
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "[ChassisSerial] tcgetattr 失败: " << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0;  // 非阻塞: 有数据就返回, 无数据等到 VTIME 超时
    tty.c_cc[VTIME] = 1;  // 0.1s 超时, 保证 recv_running_ 能被检查

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[ChassisSerial] tcsetattr 失败: " << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void ChassisSerial::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool ChassisSerial::isOpen() const {
    return fd_ >= 0;
}

// ---------------------------------------------------------------------------
// 速度控制
// ---------------------------------------------------------------------------

bool ChassisSerial::setVelocity(int16_t vx_mm_s, int16_t vy_mm_s, int16_t vz_mrad_s) {
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_vx_ = vx_mm_s;
        cmd_vy_ = vy_mm_s;
        cmd_vz_ = vz_mrad_s;
    }
    return sendCommand();
}

bool ChassisSerial::stop() {
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_vx_ = 0;
        cmd_vy_ = 0;
        cmd_vz_ = 0;
    }
    return sendCommand();
}

bool ChassisSerial::sendCommand() {
    if (!isOpen()) {
        std::cerr << "[ChassisSerial] sendCommand: 串口未打开\n";
        return false;
    }

    int16_t vx, vy, vz;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        vx = cmd_vx_;
        vy = cmd_vy_;
        vz = cmd_vz_;
    }

    const auto uvx = static_cast<uint16_t>(vx);
    const auto uvy = static_cast<uint16_t>(vy);
    const auto uvz = static_cast<uint16_t>(vz);

    uint8_t frame[SEND_FRAME_LEN];
    frame[0] = 0x7B;                                        // 帧头
    frame[1] = 0x00;                                        // 预留
    frame[2] = 0x00;                                        // 预留
    frame[3] = static_cast<uint8_t>((uvx >> 8) & 0xFF);    // vx 高字节
    frame[4] = static_cast<uint8_t>( uvx        & 0xFF);    // vx 低字节
    frame[5] = static_cast<uint8_t>((uvy >> 8) & 0xFF);    // vy 高字节
    frame[6] = static_cast<uint8_t>( uvy        & 0xFF);    // vy 低字节
    frame[7] = static_cast<uint8_t>((uvz >> 8) & 0xFF);    // vz 高字节
    frame[8] = static_cast<uint8_t>( uvz        & 0xFF);    // vz 低字节

    // BCC = XOR of frame[0..8]
    uint8_t bcc = 0;
    for (int i = 0; i < 9; ++i) bcc ^= frame[i];
    frame[9]  = bcc;
    frame[10] = 0x7D;   // 帧尾

    ssize_t n = ::write(fd_, frame, SEND_FRAME_LEN);
    if (n != static_cast<ssize_t>(SEND_FRAME_LEN)) {
        std::cerr << "[ChassisSerial] write() 失败: " << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// 反馈解析
// ---------------------------------------------------------------------------

static inline int16_t to_int16(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

ChassisSerial::FeedbackData ChassisSerial::parseFeedback(const uint8_t* buf) {
    FeedbackData d;

    d.motor_stopped = (buf[1] != 0);

    d.vx = static_cast<float>(to_int16(buf[2],  buf[3]))  * 0.001f;
    d.vy = static_cast<float>(to_int16(buf[4],  buf[5]))  * 0.001f;
    d.vz = static_cast<float>(to_int16(buf[6],  buf[7]))  * 0.001f;

    d.acc_x  = static_cast<float>(to_int16(buf[8],  buf[9]))  / ACC_SCALE;
    d.acc_y  = static_cast<float>(to_int16(buf[10], buf[11])) / ACC_SCALE;
    d.acc_z  = static_cast<float>(to_int16(buf[12], buf[13])) / ACC_SCALE;

    d.gyro_x = static_cast<float>(to_int16(buf[14], buf[15])) / GYRO_SCALE;
    d.gyro_y = static_cast<float>(to_int16(buf[16], buf[17])) / GYRO_SCALE;
    d.gyro_z = static_cast<float>(to_int16(buf[18], buf[19])) / GYRO_SCALE;

    uint16_t raw_mv = (static_cast<uint16_t>(buf[20]) << 8) | buf[21];
    d.battery_voltage = static_cast<float>(raw_mv) / 1000.0f;

    d.valid = true;
    return d;
}

// ---------------------------------------------------------------------------
// 反馈接口
// ---------------------------------------------------------------------------

ChassisSerial::FeedbackData ChassisSerial::getLatestFeedback() const {
    std::lock_guard<std::mutex> lk(fb_mutex_);
    return latest_feedback_;
}

void ChassisSerial::setFeedbackCallback(FeedbackCallback callback) {
    std::lock_guard<std::mutex> lk(fb_mutex_);
    feedback_cb_ = std::move(callback);
}

// ---------------------------------------------------------------------------
// 后台接收线程
// ---------------------------------------------------------------------------

void ChassisSerial::receiveLoop() {
    struct pollfd pfd{};
    pfd.fd     = fd_;
    pfd.events = POLLIN;

    uint8_t buf[RECV_FRAME_LEN];
    int  buf_pos = 0;
    bool synced  = false;

    while (recv_running_) {
        int ret = ::poll(&pfd, 1, 100);  // 100ms 超时便于检查 recv_running_
        if (ret < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[ChassisSerial] poll() 错误: " << std::strerror(errno) << "\n";
            break;
        }
        if (ret == 0) continue;

        if (!(pfd.revents & POLLIN)) continue;

        uint8_t b;
        ssize_t n = ::read(fd_, &b, 1);
        if (n <= 0) continue;

        if (!synced) {
            // 等待帧头
            if (b == 0x7B) {
                buf[0]  = 0x7B;
                buf_pos = 1;
                synced  = true;
            }
            continue;
        }

        buf[buf_pos++] = b;

        if (buf_pos == RECV_FRAME_LEN) {
            synced  = false;
            buf_pos = 0;

            // 校验帧尾
            if (buf[RECV_FRAME_LEN - 1] != 0x7D) {
                std::cerr << "[ChassisSerial] 帧尾错误: 0x"
                          << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(buf[RECV_FRAME_LEN - 1])
                          << std::dec << "\n";
                continue;
            }

            // 校验 BCC (前 22 字节异或)
            uint8_t bcc = 0;
            for (int i = 0; i < 22; ++i) bcc ^= buf[i];
            if (bcc != buf[22]) {
                std::cerr << "[ChassisSerial] BCC 校验失败: 计算=0x"
                          << std::hex << static_cast<int>(bcc)
                          << " 期望=0x" << static_cast<int>(buf[22])
                          << std::dec << "\n";
                continue;
            }

            FeedbackData parsed = parseFeedback(buf);
            FeedbackCallback cb;
            {
                std::lock_guard<std::mutex> lk(fb_mutex_);
                latest_feedback_ = parsed;
                cb = feedback_cb_;
            }
            if (cb) cb(parsed);
        }
    }
}

bool ChassisSerial::startReceiving() {
    if (!isOpen()) {
        std::cerr << "[ChassisSerial] startReceiving: 串口未打开\n";
        return false;
    }
    if (recv_running_) return true;

    recv_running_ = true;
    recv_thread_  = std::thread(&ChassisSerial::receiveLoop, this);
    return true;
}

void ChassisSerial::stopReceiving() {
    recv_running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

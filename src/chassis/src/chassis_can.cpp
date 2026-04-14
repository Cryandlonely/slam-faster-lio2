#include "chassis/chassis_can.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>

// ---------------------------------------------------------------------------
// 构造 / 析构
// ---------------------------------------------------------------------------

ChassisCAN::ChassisCAN(const std::string& interface_name)
    : if_name_(interface_name) {}

ChassisCAN::~ChassisCAN() {
    stopReceiving();
    close();
}

// ---------------------------------------------------------------------------
// 打开 / 关闭
// ---------------------------------------------------------------------------

bool ChassisCAN::open() {
    if (isOpen()) return true;

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "[ChassisCAN] socket() 失败: " << std::strerror(errno) << "\n";
        return false;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "[ChassisCAN] ioctl(SIOCGIFINDEX) 失败 (" << if_name_
                  << "): " << std::strerror(errno) << "\n";
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[ChassisCAN] bind() 失败: " << std::strerror(errno) << "\n";
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    return true;
}

void ChassisCAN::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool ChassisCAN::isOpen() const {
    return socket_fd_ >= 0;
}

// ---------------------------------------------------------------------------
// 速度控制
// ---------------------------------------------------------------------------

bool ChassisCAN::setVelocity(int16_t vx_mm_s, int16_t vy_mm_s, int16_t vz_mrad_s) {
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_vx_ = vx_mm_s;
        cmd_vy_ = vy_mm_s;
        cmd_vz_ = vz_mrad_s;
    }
    return sendCommand();
}

bool ChassisCAN::stop() {
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_vx_ = 0;
        cmd_vy_ = 0;
        cmd_vz_ = 0;
    }
    return sendCommand();
}

bool ChassisCAN::sendCommand() {
    if (!isOpen()) {
        std::cerr << "[ChassisCAN] sendCommand: 未打开 CAN 接口\n";
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

    uint8_t payload[8] = {
        static_cast<uint8_t>((uvx >> 8) & 0xFF),
        static_cast<uint8_t>( uvx       & 0xFF),
        static_cast<uint8_t>((uvy >> 8) & 0xFF),
        static_cast<uint8_t>( uvy       & 0xFF),
        static_cast<uint8_t>((uvz >> 8) & 0xFF),
        static_cast<uint8_t>( uvz       & 0xFF),
        0x00,
        0x00
    };

    return sendFrame(CTRL_ID, payload, 8);
}

// ---------------------------------------------------------------------------
// 底层 CAN 帧发送
// ---------------------------------------------------------------------------

bool ChassisCAN::sendFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    struct can_frame frame{};
    frame.can_id  = can_id;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
        std::cerr << "[ChassisCAN] write() 失败: " << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// 反馈帧拼接与解析
// ---------------------------------------------------------------------------

static inline int16_t to_int16(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

bool ChassisCAN::assembleAndParseFeedback(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (dlc != 8) return false;

    int    group  = -1;
    size_t offset = 0;
    switch (can_id) {
        case FB_ID_1: group = 0; offset =  0; break;
        case FB_ID_2: group = 1; offset =  8; break;
        case FB_ID_3: group = 2; offset = 16; break;
        default:      return false;
    }

    FeedbackData parsed;
    FeedbackCallback cb;
    bool complete = false;

    {
        std::lock_guard<std::mutex> lk(fb_mutex_);
        std::memcpy(fb_buf_ + offset, data, 8);
        fb_received_[group] = true;

        if (fb_received_[0] && fb_received_[1] && fb_received_[2]) {
            if (fb_buf_[0] == 0x7B && fb_buf_[23] == 0x7D) {
                uint8_t bcc = 0;
                for (int i = 0; i < 22; ++i) bcc ^= fb_buf_[i];

                if (bcc == fb_buf_[22]) {
                    parsed           = parseFeedback(fb_buf_);
                    latest_feedback_ = parsed;
                    complete         = true;
                } else {
                    std::cerr << "[ChassisCAN] BCC 校验失败\n";
                }
            }
            std::fill(fb_received_, fb_received_ + 3, false);
        }

        cb = feedback_cb_;
    }

    if (complete && cb) cb(parsed);
    return complete;
}

ChassisCAN::FeedbackData ChassisCAN::parseFeedback(const uint8_t* buf) {
    FeedbackData d;

    d.motor_stopped = (buf[1] != 0);

    d.vx = static_cast<float>(to_int16(buf[2],  buf[3]))  * 0.001f;
    d.vy = static_cast<float>(to_int16(buf[4],  buf[5]))  * 0.001f;
    d.vz = static_cast<float>(to_int16(buf[6],  buf[7]))  * 0.001f;

    d.acc_x = static_cast<float>(to_int16(buf[8],  buf[9]))  / ACC_SCALE;
    d.acc_y = static_cast<float>(to_int16(buf[10], buf[11])) / ACC_SCALE;
    d.acc_z = static_cast<float>(to_int16(buf[12], buf[13])) / ACC_SCALE;

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

ChassisCAN::FeedbackData ChassisCAN::getLatestFeedback() const {
    std::lock_guard<std::mutex> lk(fb_mutex_);
    return latest_feedback_;
}

void ChassisCAN::setFeedbackCallback(FeedbackCallback callback) {
    std::lock_guard<std::mutex> lk(fb_mutex_);
    feedback_cb_ = std::move(callback);
}

// ---------------------------------------------------------------------------
// 后台接收线程
// ---------------------------------------------------------------------------

void ChassisCAN::receiveLoop() {
    struct pollfd pfd{};
    pfd.fd     = socket_fd_;
    pfd.events = POLLIN;

    while (recv_running_) {
        int ret = ::poll(&pfd, 1, 50);
        if (ret < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[ChassisCAN] poll() 错误: " << std::strerror(errno) << "\n";
            break;
        }
        if (ret == 0) continue;

        if (pfd.revents & POLLIN) {
            struct can_frame frame{};
            ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
            if (n == static_cast<ssize_t>(sizeof(frame))) {
                assembleAndParseFeedback(frame.can_id, frame.data, frame.can_dlc);
            }
        }
    }
}

bool ChassisCAN::startReceiving() {
    if (!isOpen()) {
        std::cerr << "[ChassisCAN] startReceiving: 未打开 CAN 接口\n";
        return false;
    }
    if (recv_running_) return true;

    recv_running_ = true;
    recv_thread_  = std::thread(&ChassisCAN::receiveLoop, this);
    return true;
}

void ChassisCAN::stopReceiving() {
    recv_running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

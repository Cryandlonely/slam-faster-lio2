#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

/**
 * 履带底盘 CAN 控制接口
 *
 * 发送控制帧: ID=0x181, DLC=8
 *   [vx_H, vx_L, vy_H, vy_L, vz_H, vz_L, 0x00, 0x00]
 *   vx/vy 单位: mm/s (int16, 二进制补码)
 *   vz 单位: 0.001 rad/s (int16, 二进制补码)
 *
 * 接收反馈帧: ID=0x101/0x102/0x103, 合并为 24 字节
 *   Byte 0      : 帧头 0x7B
 *   Byte 1      : flag_stop
 *   Byte 2-3    : X 轴速度 (short, mm/s)
 *   Byte 4-5    : Y 轴速度 (short, mm/s)
 *   Byte 6-7    : Z 轴角速度 (short, 0.001 rad/s)
 *   Byte 8-9    : 加速度计 X (short, raw)
 *   Byte 10-11  : 加速度计 Y (short, raw)
 *   Byte 12-13  : 加速度计 Z (short, raw)
 *   Byte 14-15  : 角速度计 X (short, raw)
 *   Byte 16-17  : 角速度计 Y (short, raw)
 *   Byte 18-19  : 角速度计 Z (short, raw)
 *   Byte 20-21  : 电池电压(short, mV)
 *   Byte 22     : BCC 校验(前 22 字节异或)
 *   Byte 23     : 帧尾 0x7D
 */
class ChassisCAN {
public:
    // ---- 协议常量 ----
    static constexpr uint32_t CTRL_ID    = 0x181;
    static constexpr uint32_t FB_ID_1    = 0x101;
    static constexpr uint32_t FB_ID_2    = 0x102;
    static constexpr uint32_t FB_ID_3    = 0x103;

    // ---- 比例系数 ----
    static constexpr float ACC_SCALE  = 1672.0f;    // raw / ACC_SCALE = m/s²
    static constexpr float GYRO_SCALE = 3755.0f;    // raw / GYRO_SCALE = rad/s

    // ---- 反馈数据结构 ----
    struct FeedbackData {
        bool  motor_stopped    = false;
        float vx               = 0.0f;  // m/s
        float vy               = 0.0f;  // m/s
        float vz               = 0.0f;  // rad/s
        float acc_x            = 0.0f;  // m/s²
        float acc_y            = 0.0f;  // m/s²
        float acc_z            = 0.0f;  // m/s²
        float gyro_x           = 0.0f;  // rad/s
        float gyro_y           = 0.0f;  // rad/s
        float gyro_z           = 0.0f;  // rad/s
        float battery_voltage  = 0.0f;  // V
        bool  valid            = false;
    };

    using FeedbackCallback = std::function<void(const FeedbackData&)>;

    explicit ChassisCAN(const std::string& interface_name);
    ~ChassisCAN();

    ChassisCAN(const ChassisCAN&)            = delete;
    ChassisCAN& operator=(const ChassisCAN&) = delete;

    bool open();
    void close();
    bool isOpen() const;

    bool setVelocity(int16_t vx_mm_s, int16_t vy_mm_s, int16_t vz_mrad_s);
    bool stop();

    FeedbackData getLatestFeedback() const;
    void setFeedbackCallback(FeedbackCallback callback);

    bool startReceiving();
    void stopReceiving();

private:
    bool sendCommand();
    bool sendFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
    bool assembleAndParseFeedback(uint32_t can_id, const uint8_t* data, uint8_t dlc);
    static FeedbackData parseFeedback(const uint8_t* buf);
    void receiveLoop();

    std::string if_name_;
    int         socket_fd_ = -1;

    int16_t cmd_vx_ = 0;
    int16_t cmd_vy_ = 0;
    int16_t cmd_vz_ = 0;
    mutable std::mutex cmd_mutex_;

    uint8_t    fb_buf_[24]   = {};
    bool       fb_received_[3] = {};
    FeedbackData     latest_feedback_;
    FeedbackCallback feedback_cb_;
    mutable std::mutex fb_mutex_;

    std::thread        recv_thread_;
    std::atomic<bool>  recv_running_{false};
};

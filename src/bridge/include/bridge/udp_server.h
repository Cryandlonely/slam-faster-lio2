#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace bridge {

/// 简单 UDP 服务器 (单 recvfrom 循环)
/// 协议: 每个 UDP 数据报为一条 JSON 文本 (不需要换行符)
/// 用于摇杆/手柄实时速度指令, 低延迟无连接状态
class UdpServer {
public:
    /// 收到消息回调: (消息内容, 对端IP, 对端端口)
    using MessageCallback =
        std::function<void(const std::string& msg,
                           const std::string& peer_ip,
                           uint16_t peer_port)>;

    explicit UdpServer(uint16_t port);
    ~UdpServer();

    UdpServer(const UdpServer&) = delete;
    UdpServer& operator=(const UdpServer&) = delete;

    /// 启动接收循环 (后台线程)
    bool start();

    /// 停止接收循环
    void stop();

    /// 设置收到消息时的回调
    void setMessageCallback(MessageCallback cb);

    bool isRunning() const { return running_.load(); }

private:
    void recvLoop();

    uint16_t port_;
    int      sock_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread recv_thread_;

    MessageCallback msg_cb_;
};

}  // namespace bridge

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace bridge {

/// 简单 TCP 服务器 (单线程 accept, 每客户端一线程)
/// 协议: 每条消息以 '\n' 结尾的 JSON 文本 (JSON Lines)
class TcpServer {
public:
    using MessageCallback = std::function<void(int client_fd, const std::string& msg)>;
    using ConnectCallback = std::function<void(int client_fd, bool connected)>;

    explicit TcpServer(uint16_t port);
    ~TcpServer();

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;

    /// 启动监听
    bool start();

    /// 停止服务器
    void stop();

    /// 向所有已连接客户端广播消息 (自动追加 '\n')
    void broadcast(const std::string& msg);

    /// 向指定客户端发送消息
    void sendTo(int client_fd, const std::string& msg);

    /// 设置收到消息时的回调
    void setMessageCallback(MessageCallback cb);

    /// 设置客户端连接/断开回调
    void setConnectCallback(ConnectCallback cb);

    /// 当前连接数
    size_t clientCount() const;

private:
    void acceptLoop();
    void clientLoop(int client_fd);
    void removeClient(int client_fd);

    uint16_t port_;
    int      server_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread accept_thread_;

    mutable std::mutex clients_mutex_;
    std::vector<int> client_fds_;
    std::vector<std::thread> client_threads_;

    MessageCallback msg_cb_;
    ConnectCallback conn_cb_;
    std::mutex cb_mutex_;
};

}  // namespace bridge

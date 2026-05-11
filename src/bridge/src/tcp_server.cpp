#include "bridge/tcp_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>

namespace bridge {

TcpServer::TcpServer(uint16_t port) : port_(port) {}

TcpServer::~TcpServer() {
    stop();
}

bool TcpServer::start() {
    if (running_) return true;

    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "[TcpServer] socket() 失败: " << std::strerror(errno) << "\n";
        return false;
    }

    // 允许端口重用
    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port_);

    if (::bind(server_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[TcpServer] bind() 失败 (port=" << port_ << "): "
                  << std::strerror(errno) << "\n";
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (::listen(server_fd_, 5) < 0) {
        std::cerr << "[TcpServer] listen() 失败: " << std::strerror(errno) << "\n";
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread(&TcpServer::acceptLoop, this);
    return true;
}

void TcpServer::stop() {
    running_ = false;

    if (server_fd_ >= 0) {
        ::shutdown(server_fd_, SHUT_RDWR);
        ::close(server_fd_);
        server_fd_ = -1;
    }

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    // 关闭所有客户端 (fd 关闭后 clientLoop 的 poll 会返回 POLLHUP, 线程自然退出)
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (int fd : client_fds_) {
            ::shutdown(fd, SHUT_RDWR);
            ::close(fd);
        }
        client_fds_.clear();
    }
    // client 线程已 detach, 无需 join
}

void TcpServer::broadcast(const std::string& msg) {
    std::string data = msg + "\n";
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int fd : client_fds_) {
        ::send(fd, data.c_str(), data.size(), MSG_NOSIGNAL);
    }
}

void TcpServer::sendTo(int client_fd, const std::string& msg) {
    std::string data = msg + "\n";
    ::send(client_fd, data.c_str(), data.size(), MSG_NOSIGNAL);
}

void TcpServer::setMessageCallback(MessageCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    msg_cb_ = std::move(cb);
}

void TcpServer::setConnectCallback(ConnectCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    conn_cb_ = std::move(cb);
}

size_t TcpServer::clientCount() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return client_fds_.size();
}

void TcpServer::acceptLoop() {
    struct pollfd pfd{};
    pfd.fd     = server_fd_;
    pfd.events = POLLIN;

    while (running_) {
        int ret = ::poll(&pfd, 1, 200);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret == 0) continue;

        if (pfd.revents & POLLIN) {
            struct sockaddr_in client_addr{};
            socklen_t addr_len = sizeof(client_addr);
            int client_fd = ::accept(server_fd_,
                reinterpret_cast<struct sockaddr*>(&client_addr), &addr_len);
            if (client_fd < 0) {
                if (running_) {
                    std::cerr << "[TcpServer] accept() 失败: "
                              << std::strerror(errno) << "\n";
                }
                continue;
            }

            {
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
                (void)ip_str;  // suppress unused warning
            }

            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                client_fds_.push_back(client_fd);
            }

            // 连接回调
            {
                std::lock_guard<std::mutex> lock(cb_mutex_);
                if (conn_cb_) conn_cb_(client_fd, true);
            }

            // 每客户端一线程处理接收; 使用 detach 避免 client_threads_ 无限增长
            std::thread(&TcpServer::clientLoop, this, client_fd).detach();
        }
    }
}

void TcpServer::clientLoop(int client_fd) {
    char buf[4096];
    std::string line_buf;

    struct pollfd pfd{};
    pfd.fd     = client_fd;
    pfd.events = POLLIN;

    while (running_) {
        int ret = ::poll(&pfd, 1, 200);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret == 0) continue;

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) break;

        if (pfd.revents & POLLIN) {
            ssize_t n = ::recv(client_fd, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break;  // 断开或错误

            buf[n] = '\0';
            line_buf.append(buf, static_cast<size_t>(n));

            // 按 '\n' 拆分消息
            size_t pos;
            while ((pos = line_buf.find('\n')) != std::string::npos) {
                std::string msg = line_buf.substr(0, pos);
                line_buf.erase(0, pos + 1);

                // 去除 \r
                if (!msg.empty() && msg.back() == '\r') {
                    msg.pop_back();
                }

                if (!msg.empty()) {
                    std::lock_guard<std::mutex> lock(cb_mutex_);
                    if (msg_cb_) msg_cb_(client_fd, msg);
                }
            }

            // 防止缓冲区过大 (恶意/异常数据)
            if (line_buf.size() > 65536) {
                std::cerr << "[TcpServer] 客户端 fd=" << client_fd
                          << " 缓冲区溢出, 断开\n";
                break;
            }
        }
    }

    // 断开回调
    {
        std::lock_guard<std::mutex> lock(cb_mutex_);
        if (conn_cb_) conn_cb_(client_fd, false);
    }

    removeClient(client_fd);
    ::close(client_fd);
}

void TcpServer::removeClient(int client_fd) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_fds_.erase(
        std::remove(client_fds_.begin(), client_fds_.end(), client_fd),
        client_fds_.end());
}

}  // namespace bridge

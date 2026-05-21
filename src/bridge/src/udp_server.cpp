#include "bridge/udp_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace bridge {

UdpServer::UdpServer(uint16_t port) : port_(port) {}

UdpServer::~UdpServer() {
    stop();
}

bool UdpServer::start() {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        return false;
    }

    // 允许地址复用
    int opt = 1;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 接收超时: 500ms, 便于 stop() 能及时退出
    struct timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 500000;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port_);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    running_ = true;
    recv_thread_ = std::thread(&UdpServer::recvLoop, this);
    return true;
}

void UdpServer::stop() {
    running_ = false;
    if (sock_fd_ >= 0) {
        ::shutdown(sock_fd_, SHUT_RDWR);
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

void UdpServer::setMessageCallback(MessageCallback cb) {
    msg_cb_ = std::move(cb);
}

void UdpServer::recvLoop() {
    constexpr size_t kBufSize = 4096;
    char buf[kBufSize];

    while (running_) {
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);

        ssize_t n = ::recvfrom(sock_fd_, buf, kBufSize - 1, 0,
                               reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (n <= 0) {
            // EAGAIN / EWOULDBLOCK (超时) 或 socket 已关闭, 继续循环
            continue;
        }
        buf[n] = '\0';

        if (msg_cb_) {
            char ip_str[INET_ADDRSTRLEN] = {};
            ::inet_ntop(AF_INET, &peer.sin_addr, ip_str, sizeof(ip_str));
            msg_cb_(std::string(buf, static_cast<size_t>(n)),
                    std::string(ip_str),
                    ntohs(peer.sin_port));
        }
    }
}

}  // namespace bridge

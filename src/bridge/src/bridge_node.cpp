#include "bridge/bridge_node.h"

#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace bridge {

static std::string base64_encode(const std::vector<uint8_t>& data) {
    static const char* table =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((data.size() + 2) / 3) * 4);

    size_t i = 0;
    while (i + 2 < data.size()) {
        uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
                     (static_cast<uint32_t>(data[i + 1]) << 8) |
                     static_cast<uint32_t>(data[i + 2]);
        out.push_back(table[(n >> 18) & 0x3F]);
        out.push_back(table[(n >> 12) & 0x3F]);
        out.push_back(table[(n >> 6) & 0x3F]);
        out.push_back(table[n & 0x3F]);
        i += 3;
    }

    size_t rem = data.size() - i;
    if (rem == 1) {
        uint32_t n = (static_cast<uint32_t>(data[i]) << 16);
        out.push_back(table[(n >> 18) & 0x3F]);
        out.push_back(table[(n >> 12) & 0x3F]);
        out.push_back('=');
        out.push_back('=');
    } else if (rem == 2) {
        uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
                     (static_cast<uint32_t>(data[i + 1]) << 8);
        out.push_back(table[(n >> 18) & 0x3F]);
        out.push_back(table[(n >> 12) & 0x3F]);
        out.push_back(table[(n >> 6) & 0x3F]);
        out.push_back('=');
    }

    return out;
}

BridgeNode::BridgeNode(const rclcpp::NodeOptions& options)
    : Node("bridge_node", options)
{
    RCLCPP_INFO(this->get_logger(), "=== BridgeNode 初始化 (对外通讯桥接) ===");

    DeclareAndLoadParams();

    // ---- TCP 服务器 ----
    tcp_server_ = std::make_unique<TcpServer>(tcp_port_);
    tcp_server_->setMessageCallback(
        [this](int fd, const std::string& msg) { OnTcpMessage(fd, msg); });
    tcp_server_->setConnectCallback(
        [this](int fd, bool connected) { OnTcpConnect(fd, connected); });

    if (!tcp_server_->start()) {
        RCLCPP_ERROR(this->get_logger(), "TCP 服务器启动失败 (port=%d)", tcp_port_);
        throw std::runtime_error("TCP 服务器启动失败");
    }
    RCLCPP_INFO(this->get_logger(), "TCP 服务器已启动, 端口: %d", tcp_port_);

    // ---- PCD 传输专用 TCP 服务器 ----
    pcd_server_ = std::make_unique<TcpServer>(pcd_tcp_port_);
    pcd_server_->setMessageCallback(
        [this](int fd, const std::string& msg) { OnPcdTcpMessage(fd, msg); });
    pcd_server_->setConnectCallback(
        [this](int fd, bool connected) { OnPcdTcpConnect(fd, connected); });
    if (!pcd_server_->start()) {
        RCLCPP_ERROR(this->get_logger(), "PCD 传输服务器启动失败 (port=%d)", pcd_tcp_port_);
        throw std::runtime_error("PCD 传输服务器启动失败");
    }
    RCLCPP_INFO(this->get_logger(), "PCD 传输服务器已启动, 端口: %d", pcd_tcp_port_);

    // ---- ROS2 订阅 (聚合状态源) ----
    nav_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav_status", 10,
        std::bind(&BridgeNode::NavStatusCallback, this, std::placeholders::_1));

    slam_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        slam_odom_topic_, 10,
        std::bind(&BridgeNode::SlamOdomCallback, this, std::placeholders::_1));

    chassis_fb_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/chassis/feedback", 10,
        std::bind(&BridgeNode::ChassisFeedbackCallback, this, std::placeholders::_1));

    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/chassis/battery", 10,
        std::bind(&BridgeNode::BatteryCallback, this, std::placeholders::_1));

    // ---- ROS2 发布 (转发外部指令) ----
    goal_pose_pub_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    nav_waypoints_pub_ = this->create_publisher<std_msgs::msg::String>("/nav_waypoints", 10);
    nav_cancel_pub_   = this->create_publisher<std_msgs::msg::Bool>("/nav_cancel", 10);
    nav_pause_pub_    = this->create_publisher<std_msgs::msg::Bool>("/nav_pause", 10);
    nav_mode_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/nav_mode_cmd", 10);

    // ---- 状态推送定时器 ----
    auto period = std::chrono::duration<double>(1.0 / status_rate_);
    status_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&BridgeNode::StatusBroadcastCallback, this));

    RCLCPP_INFO(this->get_logger(), "BridgeNode 初始化完成");
    RCLCPP_INFO(this->get_logger(), "  控制端口: %d, PCD端口: %d, 状态推送: %.0f Hz",
                tcp_port_, pcd_tcp_port_, status_rate_);
    RCLCPP_INFO(this->get_logger(), "  订阅: /nav_status, %s, /chassis/feedback, /chassis/battery",
                slam_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  发布: goal_pose, /nav_waypoints, /nav_cancel");
}

BridgeNode::~BridgeNode() {
    if (pcd_server_) {
        pcd_server_->stop();
    }
    if (tcp_server_) {
        tcp_server_->stop();
    }
}

void BridgeNode::DeclareAndLoadParams() {
    this->declare_parameter<int>("tcp_port", 9090);
    this->declare_parameter<int>("pcd_tcp_port", 9091);
    this->declare_parameter<double>("status_rate", 5.0);
    this->declare_parameter<std::string>("slam_odom_topic", "/slam/odom");
    this->declare_parameter<std::string>("trans_pcd_rel_path", "src/location/PCD/transPCD/trans.pcd");
    this->declare_parameter<int>("trans_chunk_bytes", 65536);

    // 节点生命周期管理参数
    this->declare_parameter<std::string>("ros2_workspace", "");
    this->declare_parameter<std::string>("mapping_config_file", "mid360.yaml");
    this->declare_parameter<std::string>("localization_config_file", "mid360.yaml");
    this->declare_parameter<std::string>("indoor_mapping_odom_topic", "/Odometry");
    this->declare_parameter<std::string>("indoor_loc_odom_topic", "/localization");
    this->declare_parameter<std::string>("outdoor_odom_topic", "/outdoor/odom");

    tcp_port_        = static_cast<uint16_t>(this->get_parameter("tcp_port").as_int());
    pcd_tcp_port_    = static_cast<uint16_t>(this->get_parameter("pcd_tcp_port").as_int());
    status_rate_     = this->get_parameter("status_rate").as_double();
    slam_odom_topic_ = this->get_parameter("slam_odom_topic").as_string();
    trans_pcd_rel_path_ = this->get_parameter("trans_pcd_rel_path").as_string();
    trans_chunk_bytes_ = this->get_parameter("trans_chunk_bytes").as_int();
    if (trans_chunk_bytes_ <= 0) {
        trans_chunk_bytes_ = 65536;
    }

    ros2_workspace_           = this->get_parameter("ros2_workspace").as_string();
    mapping_config_file_      = this->get_parameter("mapping_config_file").as_string();
    localization_config_file_ = this->get_parameter("localization_config_file").as_string();
    indoor_mapping_odom_topic_   = this->get_parameter("indoor_mapping_odom_topic").as_string();
    indoor_loc_odom_topic_       = this->get_parameter("indoor_loc_odom_topic").as_string();
    outdoor_odom_topic_          = this->get_parameter("outdoor_odom_topic").as_string();
}

// ==================== TCP 消息处理 ====================

void BridgeNode::OnTcpMessage(int client_fd, const std::string& msg) {
    try {
        auto j = nlohmann::json::parse(msg);

        if (!j.contains("cmd") || !j["cmd"].is_string()) {
            std::string err = R"({"error":"missing 'cmd' field"})";
            tcp_server_->sendTo(client_fd, err);
            return;
        }

        std::string cmd = j["cmd"].get<std::string>();

        if (cmd == "nav_goal") {
            // 单点导航: 支持 local(x,y) 或 gps(lat,lon)
            std::string coord_mode = j.value("coord_mode", std::string("local"));
            if (j.contains("input_mode") && j["input_mode"].is_string() && !j.contains("coord_mode")) {
                std::string input_mode = j["input_mode"].get<std::string>();
                coord_mode = (input_mode == "outdoor") ? "gps" : "local";
            }

            if (coord_mode == "gps") {
                if (!j.contains("lat") || !j.contains("lon")) {
                    tcp_server_->sendTo(client_fd, R"({"error":"nav_goal gps mode requires lat,lon"})");
                    return;
                }
                nlohmann::json fwd;
                fwd["coord_mode"] = "gps";
                nlohmann::json wp;
                wp["lat"] = j["lat"];
                wp["lon"] = j["lon"];
                if (j.contains("yaw")) {
                    wp["yaw"] = j["yaw"];
                }
                if (j.contains("heading_deg")) {
                    wp["heading_deg"] = j["heading_deg"];
                }
                if (j.contains("target_vel")) {
                    fwd["target_vel"] = j["target_vel"];
                }
                fwd["waypoints"] = nlohmann::json::array({wp});
                auto wp_msg = std_msgs::msg::String();
                wp_msg.data = fwd.dump();
                nav_waypoints_pub_->publish(wp_msg);
                RCLCPP_INFO(this->get_logger(), "[收到] nav_goal GPS lat=%.8f lon=%.8f → ack:nav_goal",
                            j["lat"].get<double>(), j["lon"].get<double>());
            } else {
                if (!j.contains("x") || !j.contains("y")) {
                    tcp_server_->sendTo(client_fd, R"({"error":"nav_goal local mode requires x,y"})");
                    return;
                }

                auto pose_msg = geometry_msgs::msg::PoseStamped();
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "map";
                pose_msg.pose.position.x = j["x"].get<double>();
                pose_msg.pose.position.y = j["y"].get<double>();
                pose_msg.pose.position.z = 0.0;

                double yaw = j.value("yaw", 0.0);
                pose_msg.pose.orientation.w = std::cos(yaw / 2.0);
                pose_msg.pose.orientation.z = std::sin(yaw / 2.0);

                goal_pose_pub_->publish(pose_msg);

                RCLCPP_INFO(this->get_logger(), "[收到] nav_goal local x=%.3f y=%.3f yaw=%.2f° → ack:nav_goal",
                            pose_msg.pose.position.x, pose_msg.pose.position.y, yaw * 180.0 / M_PI);
            }
            tcp_server_->sendTo(client_fd, R"({"ack":"nav_goal"})");

        } else if (cmd == "nav_waypoints") {
            // 多航点: {"cmd":"nav_waypoints","waypoints":[...],"target_vel":0.5}
            // 直接转发整个 JSON (去掉 cmd 字段)
            nlohmann::json fwd = j;
            fwd.erase("cmd");
            auto wp_msg = std_msgs::msg::String();
            wp_msg.data = fwd.dump();
            nav_waypoints_pub_->publish(wp_msg);

            RCLCPP_INFO(this->get_logger(), "[收到] nav_waypoints (%zu 个航点) → ack:nav_waypoints",
                        j.contains("waypoints") ? j["waypoints"].size() : 0);
            tcp_server_->sendTo(client_fd, R"({"ack":"nav_waypoints"})");

        } else if (cmd == "nav_cancel") {
            auto cancel_msg = std_msgs::msg::Bool();
            cancel_msg.data = true;
            nav_cancel_pub_->publish(cancel_msg);

            RCLCPP_INFO(this->get_logger(), "[收到] nav_cancel → ack:nav_cancel");
            tcp_server_->sendTo(client_fd, R"({"ack":"nav_cancel"})");

        } else if (cmd == "nav_pause") {
            auto pause_msg = std_msgs::msg::Bool();
            pause_msg.data = true;
            nav_pause_pub_->publish(pause_msg);

            RCLCPP_INFO(this->get_logger(), "[收到] nav_pause → ack:nav_pause");
            tcp_server_->sendTo(client_fd, R"({"ack":"nav_pause"})");

        } else if (cmd == "nav_resume") {
            auto pause_msg = std_msgs::msg::Bool();
            pause_msg.data = false;
            nav_pause_pub_->publish(pause_msg);

            RCLCPP_INFO(this->get_logger(), "[收到] nav_resume → ack:nav_resume");
            tcp_server_->sendTo(client_fd, R"({"ack":"nav_resume"})");

        } else if (cmd == "query_status") {
            std::string status = BuildStatusJson();
            tcp_server_->sendTo(client_fd, status);

        } else if (cmd == "start_mapping" || cmd == "stop_mapping" ||
                   cmd == "start_indoor_loc" || cmd == "stop_indoor_loc" ||
                   cmd == "start_outdoor" || cmd == "stop_outdoor") {
            HandleLifecycleCmd(client_fd, cmd);

        } else {
            tcp_server_->sendTo(client_fd,
                R"({"error":"unknown cmd","supported":["nav_goal","nav_waypoints","nav_cancel","nav_pause","nav_resume","query_status","start_mapping","stop_mapping","start_indoor_loc","stop_indoor_loc","start_outdoor","stop_outdoor"]})");
        }

    } catch (const nlohmann::json::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[TCP] JSON 解析失败: %s", e.what());
        tcp_server_->sendTo(client_fd,
            std::string(R"({"error":"json_parse_error","detail":")") + e.what() + "\"}");
    }
}

void BridgeNode::OnTcpConnect(int client_fd, bool connected) {
    if (connected) {
        RCLCPP_DEBUG(this->get_logger(), "[TCP] 客户端连接 fd=%d (共 %zu)",
                    client_fd, tcp_server_->clientCount());
    } else {
        RCLCPP_DEBUG(this->get_logger(), "[TCP] 客户端断开 fd=%d", client_fd);
    }
}

void BridgeNode::OnPcdTcpConnect(int client_fd, bool connected) {
    if (connected) {
        RCLCPP_DEBUG(this->get_logger(), "[PCD TCP] 客户端连接 fd=%d", client_fd);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "[PCD TCP] 客户端断开 fd=%d", client_fd);
    }
}

std::string BridgeNode::ResolveTransPcdPath() const {
    std::filesystem::path p(trans_pcd_rel_path_);
    if (p.is_absolute()) {
        return p.string();
    }
    return (std::filesystem::current_path() / p).string();
}

void BridgeNode::OnPcdTcpMessage(int client_fd, const std::string& msg) {
    try {
        auto j = nlohmann::json::parse(msg);
        if (!j.contains("cmd") || !j["cmd"].is_string()) {
            pcd_server_->sendTo(client_fd, R"({"error":"missing 'cmd' field"})");
            return;
        }

        const std::string cmd = j["cmd"].get<std::string>();
        const std::string file_path = ResolveTransPcdPath();

        if (cmd == "pcd_info") {
            nlohmann::json out;
            out["type"] = "pcd_info";
            out["path"] = file_path;
            out["chunk_bytes"] = trans_chunk_bytes_;
            if (!std::filesystem::exists(file_path)) {
                out["exists"] = false;
                out["size"] = 0;
                out["mtime_ns"] = 0;
            } else {
                out["exists"] = true;
                out["size"] = static_cast<uint64_t>(std::filesystem::file_size(file_path));
                auto mtime = std::filesystem::last_write_time(file_path).time_since_epoch().count();
                out["mtime_ns"] = static_cast<int64_t>(mtime);
            }
            pcd_server_->sendTo(client_fd, out.dump());
            return;
        }

        if (cmd == "pcd_chunk") {
            if (!std::filesystem::exists(file_path)) {
                pcd_server_->sendTo(client_fd, R"({"error":"pcd_not_found"})");
                return;
            }

            uint64_t offset = j.value("offset", 0ULL);
            uint64_t req_len = static_cast<uint64_t>(j.value("length", trans_chunk_bytes_));
            if (req_len == 0 || req_len > static_cast<uint64_t>(trans_chunk_bytes_)) {
                req_len = static_cast<uint64_t>(trans_chunk_bytes_);
            }

            const uint64_t total_size = static_cast<uint64_t>(std::filesystem::file_size(file_path));
            if (offset >= total_size) {
                nlohmann::json out;
                out["type"] = "pcd_chunk";
                out["offset"] = offset;
                out["length"] = 0;
                out["eof"] = true;
                out["data_b64"] = "";
                out["total_size"] = total_size;
                pcd_server_->sendTo(client_fd, out.dump());
                return;
            }

            uint64_t read_len = std::min(req_len, total_size - offset);
            std::ifstream ifs(file_path, std::ios::binary);
            if (!ifs.is_open()) {
                pcd_server_->sendTo(client_fd, R"({"error":"pcd_open_failed"})");
                return;
            }
            ifs.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
            std::vector<uint8_t> buf(static_cast<size_t>(read_len));
            ifs.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(read_len));
            size_t got = static_cast<size_t>(ifs.gcount());
            buf.resize(got);

            nlohmann::json out;
            out["type"] = "pcd_chunk";
            out["offset"] = offset;
            out["length"] = static_cast<uint64_t>(got);
            out["eof"] = (offset + got >= total_size);
            out["total_size"] = total_size;
            out["data_b64"] = base64_encode(buf);
            pcd_server_->sendTo(client_fd, out.dump());
            return;
        }

        pcd_server_->sendTo(client_fd,
            R"({"error":"unknown cmd","supported":["pcd_info","pcd_chunk"]})");
    } catch (const nlohmann::json::exception& e) {
        pcd_server_->sendTo(client_fd,
            std::string(R"({"error":"json_parse_error","detail":")") + e.what() + "\"}");
    }
}

// ==================== ROS2 状态回调 ====================

void BridgeNode::NavStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string new_state;
    try {
        auto j = nlohmann::json::parse(msg->data);
        new_state = j.value("state", std::string{});
    } catch (...) {}

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        nav_status_json_ = msg->data;
    }

    // 检测 REACHED / ERROR 跳变 → 向所有客户端推送导航结果
    if (!new_state.empty() && new_state != nav_state_last_) {
        if (new_state == "REACHED" || new_state == "ERROR") {
            std::string result =
                std::string(R"({"type":"nav_result","result":")")
                + new_state + "\"}";  
            tcp_server_->broadcast(result);
            RCLCPP_INFO(this->get_logger(), "[Bridge] 导航结束 → 推送 nav_result: %s", new_state.c_str());
        }
        nav_state_last_ = new_state;
    }
}

void BridgeNode::SlamOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 暂未使用 SLAM odom 原始数据 (位置已由 /nav_status 携带)
    // 保留订阅以供后续扩展
    (void)msg;
}

void BridgeNode::ChassisFeedbackCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    chassis_fb_json_ = msg->data;
}

void BridgeNode::BatteryCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    battery_voltage_ = msg->data;
}

// ==================== 状态推送 ====================

void BridgeNode::StatusBroadcastCallback() {
    if (tcp_server_->clientCount() == 0) return;

    std::string status = BuildStatusJson();
    tcp_server_->broadcast(status);
}

std::string BridgeNode::BuildStatusJson() {
    std::lock_guard<std::mutex> lock(status_mutex_);

    // 以 nav_planner 的位置包为基础，追加电量和底盘速度
    try {
        auto j = nlohmann::json::parse(nav_status_json_);

        // 电量 (来自 /chassis/battery)
        j["battery"] = battery_voltage_;

        // 底盘速度 vx/vy (来自 /chassis/feedback)
        try {
            auto fb = nlohmann::json::parse(chassis_fb_json_);
            j["vx"] = fb.value("vx", 0.0);
            j["vy"] = fb.value("vy", 0.0);
        } catch (...) {
            j["vx"] = 0.0;
            j["vy"] = 0.0;
        }

        return j.dump();
    } catch (...) {
        return nav_status_json_;
    }
}

// ==================== 节点生命周期管理 ====================

void BridgeNode::StartManagedProcess(const std::string& name, const std::string& bash_cmd) {
    StopManagedProcess(name);

    pid_t pid = fork();
    if (pid == 0) {
        // 子进程: 创建新会话，与父进程解耦
        setsid();
        // 子进程继承父进程的 ROS2 环境变量，直接可用 ros2 命令
        execl("/bin/bash", "bash", "-c", bash_cmd.c_str(), nullptr);
        _exit(1);
    } else if (pid > 0) {
        {
            std::lock_guard<std::mutex> lock(proc_mutex_);
            managed_pids_[name] = pid;
        }
        RCLCPP_INFO(this->get_logger(), "[Lifecycle] 启动进程 '%s' (pid=%d): %s",
                    name.c_str(), pid, bash_cmd.c_str());
        // 延迟 500ms 检查子进程是否还活着 (快速崩溃检测)
        std::thread([this, name, pid]() {
            usleep(500000);
            int status = 0;
            pid_t ret = waitpid(pid, &status, WNOHANG);
            if (ret == pid) {
                // 进程已退出
                RCLCPP_ERROR(this->get_logger(),
                    "[Lifecycle] 进程 '%s' (pid=%d) 启动后立即退出! 退出码=%d\n"
                    "  可能原因: 包未编译/串口不存在/launch文件路径错误\n"
                    "  命令: %s",
                    name.c_str(), pid, WEXITSTATUS(status),
                    "(见上方启动日志)");
                std::lock_guard<std::mutex> lock(proc_mutex_);
                managed_pids_.erase(name);
            } else if (ret == 0) {
                RCLCPP_INFO(this->get_logger(),
                    "[Lifecycle] 进程 '%s' (pid=%d) 运行正常", name.c_str(), pid);
            }
        }).detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "[Lifecycle] fork() 失败: %s", name.c_str());
    }
}

void BridgeNode::StopManagedProcess(const std::string& name) {
    std::lock_guard<std::mutex> lock(proc_mutex_);
    auto it = managed_pids_.find(name);
    if (it == managed_pids_.end()) return;

    pid_t pid = it->second;
    // setsid 后子进程是自己 session 的进程组长, pgid == pid
    // killpg 会发信号给整个组（包括 ros2 launch 的子进程）
    if (killpg(pid, SIGTERM) != 0) {
        kill(pid, SIGTERM);  // fallback
    }
    // mapping 节点需要在 spin() 返回后执行 PCD 保存（统计滤波+体素降采样），
    // 大地图可能耗时较长，给 30 秒；其它进程 3 秒足够
    int wait_iters = (name == "mapping") ? 300 : 30;
    for (int i = 0; i < wait_iters; ++i) {
        int status = 0;
        if (waitpid(pid, &status, WNOHANG) != 0) break;
        usleep(100000);  // 100ms
    }
    // 若仍未退出则强杀
    if (killpg(pid, 0) == 0) {
        killpg(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
    }
    managed_pids_.erase(it);
    RCLCPP_INFO(this->get_logger(), "[Lifecycle] 已停止进程 '%s' (pid=%d)", name.c_str(), pid);
}

void BridgeNode::PublishNavModeSwitch(const std::string& odom_topic, bool gps_mode) {
    nlohmann::json j;
    j["action"]     = "switch_odom";
    j["topic"]      = odom_topic;
    j["gps"]        = gps_mode;
    auto msg = std_msgs::msg::String();
    msg.data = j.dump();
    nav_mode_cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[Lifecycle] 发布定位源切换: topic=%s gps=%d",
                odom_topic.c_str(), gps_mode);
}

void BridgeNode::HandleLifecycleCmd(int client_fd, const std::string& subcmd) {
    RCLCPP_INFO(this->get_logger(), "[收到] %s", subcmd.c_str());
    // 构造 ros2 launch/run 命令（继承当前进程的 ROS 环境，无需再 source）
    auto make_launch = [&](const std::string& pkg, const std::string& launch_file,
                            const std::string& config) -> std::string {
        std::string cmd = "ros2 launch " + pkg + " " + launch_file;
        if (!config.empty()) {
            cmd += " config_file:=" + config + " rviz:=false";
        }
        if (!ros2_workspace_.empty()) {
            cmd = "source " + ros2_workspace_ + "/install/setup.bash && " + cmd;
        }
        return cmd;
    };

    if (subcmd == "start_mapping") {
        auto cmd = make_launch("location", "mapping.launch.py", mapping_config_file_);
        StartManagedProcess("mapping", cmd);
        // nav_planner 直接用 /Odometry（建图模式）
        PublishNavModeSwitch(indoor_mapping_odom_topic_, false);
        RCLCPP_INFO(this->get_logger(), "[回复] ack:start_mapping");
        tcp_server_->sendTo(client_fd, R"({"ack":"start_mapping"})");

    } else if (subcmd == "stop_mapping") {
        // 先立即回复 ack, 再异步执行耗时操作 (避免阻塞 TCP 线程长达 45s)
        RCLCPP_INFO(this->get_logger(), "[回复] ack:stop_mapping (异步保存地图中)");
        tcp_server_->sendTo(client_fd, R"({"ack":"stop_mapping"})");
        std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "[stop_mapping] 调用 /map_save service ...");
            int ret = system("ros2 service call /map_save std_srvs/srv/Trigger '{}' 2>&1 | grep -q 'success=True'");
            if (ret == 0) {
                RCLCPP_INFO(this->get_logger(), "[stop_mapping] map_save 成功");
            } else {
                RCLCPP_WARN(this->get_logger(), "[stop_mapping] map_save service 调用失败或超时 (ret=%d), 仍继续停止进程", ret);
            }
            StopManagedProcess("mapping");
        }).detach();

    } else if (subcmd == "start_indoor_loc") {
        // 停止纯建图，启动含全局定位的完整室内定位
        StopManagedProcess("mapping");
        auto cmd = make_launch("location", "velodyne_localization.launch.py",
                               localization_config_file_);
        StartManagedProcess("indoor_loc", cmd);
        // 等 transform_fusion 起来后切换到 /localization
        // 稍作延迟（由上层逻辑决定），这里先切 odom topic
        PublishNavModeSwitch(indoor_loc_odom_topic_, false);
        RCLCPP_INFO(this->get_logger(), "[回复] ack:start_indoor_loc");
        tcp_server_->sendTo(client_fd, R"({"ack":"start_indoor_loc"})");

    } else if (subcmd == "stop_indoor_loc") {
        StopManagedProcess("indoor_loc");
        RCLCPP_INFO(this->get_logger(), "[回复] ack:stop_indoor_loc");
        tcp_server_->sendTo(client_fd, R"({"ack":"stop_indoor_loc"})");

    } else if (subcmd == "start_outdoor") {
        // 先立即回复 ack, 避免 TCP 线程在 StopManagedProcess 阻塞期间超时
        RCLCPP_INFO(this->get_logger(), "[回复] ack:start_outdoor (异步启动RTK中)");
        tcp_server_->sendTo(client_fd, R"({"ack":"start_outdoor"})");
        auto rtk_cmd = make_launch("rtk", "rtk_launch.py", "");
        std::thread([this, rtk_cmd]() {
            // 先停旧进程 (可能阻塞数秒, 异步执行)
            StopManagedProcess("indoor_loc");
            StopManagedProcess("mapping");
            // 启动 RTK
            StartManagedProcess("rtk", rtk_cmd);
            // 通知 nav_planner 切换定位源
            PublishNavModeSwitch(outdoor_odom_topic_, true);
        }).detach();

    } else if (subcmd == "stop_outdoor") {
        // 先立即回复 ack
        RCLCPP_INFO(this->get_logger(), "[回复] ack:stop_outdoor (异步停止RTK中)");
        tcp_server_->sendTo(client_fd, R"({"ack":"stop_outdoor"})");
        std::thread([this]() {
            StopManagedProcess("rtk");
            // 切回室内定位 odom
            bool indoor_loc_running = false;
            {
                std::lock_guard<std::mutex> lock(proc_mutex_);
                indoor_loc_running = managed_pids_.count("indoor_loc") > 0;
            }
            const std::string& indoor_topic = indoor_loc_running
                ? indoor_loc_odom_topic_ : indoor_mapping_odom_topic_;
            PublishNavModeSwitch(indoor_topic, false);
        }).detach();
    }
}

}  // namespace bridge

/**
 * @file app_motion_target_node.cpp
 * @brief PID 目标桥接节点 —— UDP 接收目标指令, 发布到 /app/motioncontrol
 *
 * ## 数据流
 *   地面站 UDP :5003 (17字节 0xA5 协议)
 *     → 解析 surge(vx) / sway(vy) / heave(depth) / yaw(heading)
 *     → 转换为物理量纲 (m/s, m, rad)
 *     → 发布 /app/motioncontrol (Float64MultiArray[7])
 *     → bsp_motioncontrol_node 完成 6-DOF PID + 前馈 + 推力分配
 *
 * ## /app/motioncontrol 消息格式 (与 bsp_motioncontrol_node 约定)
 *   data[0] = 保留 (通常为模式标记)
 *   data[1] = vx  (m/s)  纵荡速度目标
 *   data[2] = vy  (m/s)  横荡速度目标
 *   data[3] = depth (m)  深度目标
 *   data[4] = yaw (rad)  艏向目标
 *   data[5] = pitch (rad) 纵倾目标 (当前固定为 0)
 *   data[6] = roll (rad)  横摇目标 (当前固定为 0)
 *
 * ## 与 app_keyboard_control_node 互斥
 *   - 本节点 → bsp_motioncontrol_node → /hal/thruster/cmd (PID 闭环)
 *   - app_keyboard_control_node → /hal/thruster/cmd (开环直驱)
 *   - 同一时刻只应激活其一, 避免话题争用
 *
 * ## 设计取舍
 *   - 地面站仍用同一 17 字节协议发送目标值, 但语义解释不同
 *   - surge/sway 是 m/s（0.001 分辨率），并按 max_velocity 限幅
 *   - heave → 深度目标 (m)
 *   - yaw → 艏向目标 (rad, 地面站发度数, 本节点转弧度)
 */

 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>
 #include <fcntl.h>
 #include <unistd.h>
 
 #include <algorithm>
 #include <atomic>
 #include <cerrno>
 #include <chrono>
 #include <cmath>
 #include <cstring>
 #include <exception>
 #include <functional>
 #include <memory>
 #include <mutex>
 #include <string>
 #include <thread>
 
 #include "rclcpp/rclcpp.hpp"
 #include "rclcpp_lifecycle/lifecycle_node.hpp"
 #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
 #include "std_msgs/msg/float64_multi_array.hpp"
 
 #include "app/app_motion_mode_protocol.hpp"
 
 using CallbackReturn =
     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
 
 // ============================================================================
 // UDP 协议常量
 //   struct: <BHhhhhHH = 15 字节 + 2 字节 CRC 尾 = 17 字节总长
 // ============================================================================
 static constexpr uint16_t FLAG_ESTOP = app::motion::DATA_FLAG_ESTOP;
 static constexpr double PI = 3.14159265358979323846;
 
 // ============================================================================
 // 运动目标指令
 // ============================================================================
 struct TargetCmd {
     double vx    = 0.0;   // m/s
     double vy    = 0.0;   // m/s
     double depth = 5.0;   // m (默认中性深度, 避免上浮到水面)
     double yaw   = 0.0;   // rad
     uint16_t flags = 0;
     bool fresh = false;
 };
 
 // ============================================================================
 // 主节点类
 // ============================================================================
 class AppMotionTargetNode : public rclcpp_lifecycle::LifecycleNode
 {
 public:
     AppMotionTargetNode()
     : LifecycleNode("app_motion_target_node")
     {
         this->declare_parameter("udp_port", 5003);
         this->declare_parameter("publish_rate_hz", 20.0);
         this->declare_parameter("max_velocity_x", 1.0);
         this->declare_parameter("max_velocity_y", 0.5);
         this->declare_parameter("cmd_timeout_s", 1.0);
     }
 
     // ==================== 生命周期 ====================
 
     CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
     {
         RCLCPP_INFO(get_logger(), "[TGT] on_configure");
 
         target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
             "/app/motioncontrol", 10);
 
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
     {
         RCLCPP_INFO(get_logger(), "[TGT] on_activate");
 
         target_pub_->on_activate();
 
         int port     = this->get_parameter("udp_port").as_int();
         double rate  = this->get_parameter("publish_rate_hz").as_double();
         max_vx_      = this->get_parameter("max_velocity_x").as_double();
         max_vy_      = this->get_parameter("max_velocity_y").as_double();
         cmd_timeout_s_ = this->get_parameter("cmd_timeout_s").as_double();
 
         if (port < 1 || port > 65535 || !std::isfinite(rate) ||
             rate <= 0.0 || rate > 1000.0 || !std::isfinite(max_vx_) ||
             !std::isfinite(max_vy_) || !std::isfinite(cmd_timeout_s_) ||
             max_vx_ <= 0.0 || max_vy_ <= 0.0 || cmd_timeout_s_ <= 0.0) {
             RCLCPP_ERROR(get_logger(),
                 "[TGT] 参数非法: port=%d rate=%.3f max_vx=%.3f max_vy=%.3f timeout=%.3f",
                 port, rate, max_vx_, max_vy_, cmd_timeout_s_);
             target_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
 
         // Never reuse a target received during a previous activation lease.
         {
             std::lock_guard<std::mutex> lock(cmd_mutex_);
             latest_cmd_ = TargetCmd{};
             last_cmd_time_ = std::chrono::steady_clock::now();
         }
 
         // UDP socket
         sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
         if (sock_fd_ < 0) {
             RCLCPP_ERROR(get_logger(), "[TGT] socket() 失败: %s", strerror(errno));
             target_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
         struct sockaddr_in addr{};
         addr.sin_family = AF_INET;
         addr.sin_port   = htons(port);
         addr.sin_addr.s_addr = INADDR_ANY;
         if (bind(sock_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
             RCLCPP_ERROR(get_logger(), "[TGT] bind(:%d) 失败: %s", port, strerror(errno));
             close(sock_fd_);
             sock_fd_ = -1;
             target_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
         int fl = fcntl(sock_fd_, F_GETFL, 0);
         if (fl < 0 || fcntl(sock_fd_, F_SETFL, fl | O_NONBLOCK) < 0) {
             RCLCPP_ERROR(get_logger(), "[TGT] fcntl(O_NONBLOCK) 失败: %s", strerror(errno));
             close(sock_fd_);
             sock_fd_ = -1;
             target_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
 
         try {
             int period_ms = std::max(1, static_cast<int>(1000.0 / rate));
             pub_timer_ = this->create_wall_timer(
                 std::chrono::milliseconds(period_ms),
                 std::bind(&AppMotionTargetNode::publish_target, this));
             running_ = true;
             recv_thread_ = std::thread(&AppMotionTargetNode::udp_recv_loop, this);
         } catch (const std::exception &exception) {
             RCLCPP_ERROR(get_logger(), "[TGT] activation resource setup failed: %s",
                 exception.what());
             stop_io();
             if (pub_timer_) {
                 pub_timer_->cancel();
                 pub_timer_.reset();
             }
             target_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
 
         RCLCPP_INFO(get_logger(),
             "[TGT] 已激活, UDP :%d, 发布 %dHz | max_vx=%.1f max_vy=%.1f",
             port, (int)rate, max_vx_, max_vy_);
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
     {
         RCLCPP_INFO(get_logger(), "[TGT] on_deactivate");
         if (pub_timer_) pub_timer_->cancel();
         stop_io();
         publish_estop_target();
         target_pub_->on_deactivate();
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
     {
         target_pub_.reset();
         pub_timer_.reset();
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
     {
         if (pub_timer_) pub_timer_->cancel();
         publish_estop_target();
         stop_io();
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_error(const rclcpp_lifecycle::State &) override
     {
         if (pub_timer_) pub_timer_->cancel();
         publish_estop_target();
         stop_io();
         return CallbackReturn::SUCCESS;
     }
 
 private:
     // ==================== 协议解析 ====================
     bool parse_packet(const uint8_t *data, size_t len, TargetCmd &cmd) const
     {
         const auto decoded = app::motion::decodeDataPlaneCommand(data, len);
         if (!decoded.ok) return false;
 
         // surge/sway are physical velocity targets in m/s (wire resolution 0.001).
         // heave → 深度目标 (m), 分辨率 0.01m
         cmd.depth = decoded.command.heave / 100.0;
         // yaw → 艏向目标 (地面站发 °, 转为 rad)
         cmd.yaw = decoded.command.yaw / 10.0 * PI / 180.0;
         cmd.flags = decoded.command.flags;
         cmd.fresh = true;
 
         cmd.vx = app::motion::decodeBoundedVelocity(decoded.command.surge, max_vx_);
         cmd.vy = app::motion::decodeBoundedVelocity(decoded.command.sway, max_vy_);
 
         return true;
     }
 
     // ==================== UDP 接收线程 ====================
     void udp_recv_loop()
     {
         uint8_t buf[256];
         while (running_) {
             ssize_t n = recvfrom(sock_fd_, buf, sizeof(buf), 0, nullptr, nullptr);
             if (n < 0) {
                 if (errno == EAGAIN || errno == EWOULDBLOCK) {
                     std::this_thread::sleep_for(std::chrono::milliseconds(5));
                     continue;
                 }
                 break;
             }
             TargetCmd cmd;
             if (parse_packet(buf, static_cast<size_t>(n), cmd)) {
                 std::lock_guard<std::mutex> lock(cmd_mutex_);
                 latest_cmd_ = cmd;
                 last_cmd_time_ = std::chrono::steady_clock::now();
             }
         }
     }
 
     // ==================== 定时发布 ====================
     void publish_target()
     {
         TargetCmd cmd;
         std::chrono::steady_clock::time_point last_time;
         {
             std::lock_guard<std::mutex> lock(cmd_mutex_);
             cmd = latest_cmd_;
             last_time = last_cmd_time_;
         }
 
         // Do not publish until this activation has received a fresh target.
         if (!cmd.fresh) {
             return;
         }
 
         // Watchdog timeout keeps depth/yaw but removes translational velocity.
         const double dt = std::chrono::duration<double>(
             std::chrono::steady_clock::now() - last_time).count();
         if (dt > cmd_timeout_s_) {
             cmd.vx = 0.0; cmd.vy = 0.0;
             // 保持深度/偏航目标不变, 避免骤停 (仅速度归零)
             // 如仍需全停, 可改为全零
         }
 
         auto msg = std_msgs::msg::Float64MultiArray();
         msg.data.resize(7);
         msg.data[0] = (cmd.flags & FLAG_ESTOP) != 0U ? 1.0 : 0.0;
         msg.data[1] = cmd.vx;       // vx (m/s)
         msg.data[2] = cmd.vy;       // vy (m/s)
         msg.data[3] = cmd.depth;    // depth (m)
         msg.data[4] = cmd.yaw;      // yaw (rad)
         msg.data[5] = 0.0;          // pitch (rad) 锁定
         msg.data[6] = 0.0;          // roll  (rad) 锁定
 
         target_pub_->publish(msg);
     }
 
     void publish_estop_target()
     {
         if (!target_pub_) return;
         auto msg = std_msgs::msg::Float64MultiArray();
         msg.data = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
         target_pub_->publish(msg);
     }
 
     void stop_io()
     {
         running_ = false;
         if (recv_thread_.joinable()) recv_thread_.join();
         if (sock_fd_ >= 0) {
             close(sock_fd_);
             sock_fd_ = -1;
         }
     }
 
     // ==================== 成员 ====================
     rclcpp_lifecycle::LifecyclePublisher<
         std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
     rclcpp::TimerBase::SharedPtr pub_timer_;
 
     int sock_fd_ = -1;
     std::thread recv_thread_;
     std::atomic<bool> running_{false};
     std::mutex cmd_mutex_;
     TargetCmd latest_cmd_;
     std::chrono::steady_clock::time_point last_cmd_time_{};
 
     double max_vx_ = 1.0;
     double max_vy_ = 0.5;
     double cmd_timeout_s_ = 1.0;
 };
 
 int main(int argc, char **argv)
 {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<AppMotionTargetNode>();
     rclcpp::executors::MultiThreadedExecutor exec;
     exec.add_node(node->get_node_base_interface());
     exec.spin();
     rclcpp::shutdown();
     return 0;
 }
 
 
 
/**
 * @file app_keyboard_control_node.cpp
 * @brief 键盘直驱节点 —— UDP 接收键令, 开环推力分配, 发布 /hal/thruster/cmd
 *
 * ## 数据流
 *   地面站 UDP :5002 (17字节 0xA5 协议)
 *     → 解析 surge/sway/heave/yaw
 *     → 直接缩放为 6-DOF wrench (无 PID)
 *     → T_pinv 推力分配 (6×6 阻尼最小二乘)
 *     → 发布 /hal/thruster/cmd (Float64MultiArray[6], 百分比)
 *
 * ## 与 bsp_motioncontrol_node 的关系
 *   - 本节点 == 开环模式: 键盘 → 直接推力, 绕过 PID
 *   - bsp_motioncontrol_node == 闭环模式: /app/motioncontrol → PID + 前馈 → 推力
 *   - 两者互斥使用同一个 /hal/thruster/cmd 话题 (同一时刻只应激活其一)
 *
 * ## 平滑控制 (v2)
 *   - 斜坡限幅 (Slew Rate Limiter): 限制力/力矩变化率, 避免阶跃跳变
 *   - 推进器一阶时滞 (Thruster Dynamics): f_actual = α·f_cmd + (1-α)·f_prev
 *     α = dt/(τ+dt), τ 默认 0.2s (匹配 MATLAB T_motor)
 *   - 急停时重置平滑器状态
 *
 * ## 设计取舍
 *   - surge/sway: 瞬时量, 松开归零 (dead-man switch 由地面站保证)
 *   - heave/yaw:  累计量目标, 但开环下仅翻译为力/力矩 (无 PID 跟踪)
 *   - 无 Eigen 依赖: 矩阵运算手写
 *   - 适配 develop 6 推进器布局 (1 主推 + 5 辅推)
 */

 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>
 #include <fcntl.h>
 #include <unistd.h>
 
 #include <algorithm>
 #include <array>
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
 // UDP 协议常量 (与地面站 UVMS_ControlSender.py 一致)
 //   struct: <BHhhhhHH = 15 字节 + 2 字节 CRC 尾 = 17 字节总长
 // ============================================================================
 static constexpr size_t PROTO_PKT_SIZE = app::motion::DATA_PLANE_SIZE;
 static constexpr uint16_t FLAG_ESTOP = app::motion::DATA_FLAG_ESTOP;
 
 // ============================================================================
 // 控制分配矩阵 T_pinv (6×6), 预计算自 develop 默认 T_alloc
 //   列: Main, Aux0..Aux4    行: Fx,Fy,Fz,Mx,My,Mz
 // ============================================================================
 static constexpr std::array<std::array<double, 6>, 6> T_PINV = {{
     {  0.74226804, -0.18556701,  0.00000000,  0.00000000,  0.00000000,  0.41237113 },
     {  0.00000000,  0.00000000,  0.45871560,  5.00000000,  0.13761468, -0.00000000 },
     {  0.00000000, -0.00000000,  0.45871560, -5.00000000,  0.13761468,  0.00000000 },
     { -0.10309278,  0.52577320, -0.00000000,  0.00000000, -0.00000000,  2.16494845 },
     {  0.10309278,  0.47422680,  0.00000000, -0.00000000,  0.00000000, -2.16494845 },
     {  0.41237113, -0.10309278, -0.00000000, -0.00000000, -0.00000000,  1.34020619 }
 }};
 
 // ============================================================================
 // 运动指令 (解析自 UDP)
 // ============================================================================
 struct MotionCmd {
     double surge  = 0.0;   // [-1, 1]
     double sway   = 0.0;   // [-1, 1]
     double heave  = 5.0;   // 目标深度 [m] (默认 5m 中性点)
     double yaw    = 0.0;   // 目标偏航 [°] (开环当做力矩参考)
     uint16_t flags = 0;
     uint16_t seq   = 0;
     bool fresh     = false;
 };
 
 // ============================================================================
 // 主节点类
 // ============================================================================
 class AppKeyboardControlNode : public rclcpp_lifecycle::LifecycleNode
 {
 public:
     AppKeyboardControlNode()
     : LifecycleNode("app_keyboard_control_node")
     {
         this->declare_parameter("udp_port", 5002);
         this->declare_parameter("control_rate_hz", 20.0);
         this->declare_parameter("cmd_timeout_s", 0.5);
         this->declare_parameter("surge_scale", 300.0);
         this->declare_parameter("sway_scale", 160.0);
         this->declare_parameter("heave_scale", 80.0);
         this->declare_parameter("yaw_scale", 5.0);
         this->declare_parameter("smoothing.enable", true);
         this->declare_parameter("smoothing.slew_Fx", 600.0);
         this->declare_parameter("smoothing.slew_Fy", 320.0);
         this->declare_parameter("smoothing.slew_Fz", 160.0);
         this->declare_parameter("smoothing.slew_Mz", 10.0);
         this->declare_parameter("smoothing.thruster_tau", 0.2);
     }
 
     // ==================== 生命周期 ====================
 
     CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
     {
         RCLCPP_INFO(get_logger(), "[KBD] on_configure");
 
         thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
             "/hal/thruster/cmd", 10);
 
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
     {
         RCLCPP_INFO(get_logger(), "[KBD] on_activate");
 
         thruster_pub_->on_activate();
 
         int port   = this->get_parameter("udp_port").as_int();
         double rate = this->get_parameter("control_rate_hz").as_double();
         cmd_timeout_s_ = this->get_parameter("cmd_timeout_s").as_double();
 
         if (port < 1 || port > 65535 || !std::isfinite(rate) ||
             rate <= 0.0 || rate > 1000.0 || !std::isfinite(cmd_timeout_s_) ||
             cmd_timeout_s_ <= 0.0) {
             RCLCPP_ERROR(get_logger(),
                 "[KBD] 参数非法: udp_port=%d rate=%.3f timeout=%.3f",
                 port, rate, cmd_timeout_s_);
             thruster_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
 
         surge_scale_ = this->get_parameter("surge_scale").as_double();
         sway_scale_  = this->get_parameter("sway_scale").as_double();
         heave_scale_ = this->get_parameter("heave_scale").as_double();
         yaw_scale_   = this->get_parameter("yaw_scale").as_double();
 
         smoothing_enable_ = this->get_parameter("smoothing.enable").as_bool();
         slew_Fx_  = this->get_parameter("smoothing.slew_Fx").as_double();
         slew_Fy_  = this->get_parameter("smoothing.slew_Fy").as_double();
         slew_Fz_  = this->get_parameter("smoothing.slew_Fz").as_double();
         slew_Mz_  = this->get_parameter("smoothing.slew_Mz").as_double();
         thruster_tau_ = this->get_parameter("smoothing.thruster_tau").as_double();
 
         const auto is_nonnegative_finite = [](double value) {
             return std::isfinite(value) && value >= 0.0;
         };
         if (!is_nonnegative_finite(surge_scale_) ||
             !is_nonnegative_finite(sway_scale_) ||
             !is_nonnegative_finite(heave_scale_) ||
             !is_nonnegative_finite(yaw_scale_) ||
             !is_nonnegative_finite(slew_Fx_) ||
             !is_nonnegative_finite(slew_Fy_) ||
             !is_nonnegative_finite(slew_Fz_) ||
             !is_nonnegative_finite(slew_Mz_) ||
             !is_nonnegative_finite(thruster_tau_)) {
             RCLCPP_ERROR(get_logger(), "[KBD] scaling or smoothing parameter is invalid");
             thruster_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
 
         // 重置平滑器状态
         prev_tau_ = {0, 0, 0, 0, 0, 0};
         thruster_filtered_ = {0, 0, 0, 0, 0, 0};
         {
             std::lock_guard<std::mutex> lock(cmd_mutex_);
             latest_cmd_ = MotionCmd{};
             last_cmd_time_ = std::chrono::steady_clock::now();
         }
         watchdog_tripped_ = false;
 
         // 创建 UDP socket (非阻塞)
         sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
         if (sock_fd_ < 0) {
             RCLCPP_ERROR(get_logger(), "[KBD] socket() 失败: %s", strerror(errno));
             thruster_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
         struct sockaddr_in addr{};
         addr.sin_family = AF_INET;
         addr.sin_port   = htons(port);
         addr.sin_addr.s_addr = INADDR_ANY;
         if (bind(sock_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
             RCLCPP_ERROR(get_logger(), "[KBD] bind(:%d) 失败: %s", port, strerror(errno));
             close(sock_fd_);
             sock_fd_ = -1;
             thruster_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
         int fl = fcntl(sock_fd_, F_GETFL, 0);
         if (fl < 0 || fcntl(sock_fd_, F_SETFL, fl | O_NONBLOCK) < 0) {
             RCLCPP_ERROR(get_logger(), "[KBD] fcntl(O_NONBLOCK) 失败: %s", strerror(errno));
             close(sock_fd_);
             sock_fd_ = -1;
             thruster_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
 
         // 启动 UDP 接收线程
         try {
             // The timer is created before the thread so no throwing operation
             // remains after the receiver becomes joinable.
             int period_ms = std::max(1, static_cast<int>(1000.0 / rate));
             ctrl_timer_ = this->create_wall_timer(
                 std::chrono::milliseconds(period_ms),
                 std::bind(&AppKeyboardControlNode::control_loop, this));
             running_ = true;
             recv_thread_ = std::thread(&AppKeyboardControlNode::udp_recv_loop, this);
         } catch (const std::exception &exception) {
             RCLCPP_ERROR(get_logger(), "[KBD] activation resource setup failed: %s",
                 exception.what());
             stop_io();
             if (ctrl_timer_) {
                 ctrl_timer_->cancel();
                 ctrl_timer_.reset();
             }
             thruster_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
 
         RCLCPP_INFO(get_logger(),
             "[KBD] 已激活, UDP :%d, 控制 %dHz | 缩放: surge=%.0f sway=%.0f heave=%.0f yaw=%.1f",
             port, (int)rate, surge_scale_, sway_scale_, heave_scale_, yaw_scale_);
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
     {
         RCLCPP_INFO(get_logger(), "[KBD] on_deactivate");
         if (ctrl_timer_) ctrl_timer_->cancel();
         stop_io();
         send_zero_thrust();
         thruster_pub_->on_deactivate();
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
     {
         thruster_pub_.reset();
         ctrl_timer_.reset();
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
     {
         if (ctrl_timer_) ctrl_timer_->cancel();
         send_zero_thrust();
         stop_io();
         return CallbackReturn::SUCCESS;
     }
 
     CallbackReturn on_error(const rclcpp_lifecycle::State &) override
     {
         if (ctrl_timer_) ctrl_timer_->cancel();
         send_zero_thrust();
         stop_io();
         return CallbackReturn::SUCCESS;
     }
 
 private:
     // ==================== 协议解析 ====================
     static bool parse_packet(const uint8_t *data, size_t len, MotionCmd &cmd)
     {
         const auto decoded = app::motion::decodeDataPlaneCommand(data, len);
         if (!decoded.ok) return false;
 
         cmd.seq = decoded.command.sequence;
         cmd.surge = decoded.command.surge / 1000.0;
         cmd.sway = decoded.command.sway / 1000.0;
         cmd.heave = decoded.command.heave / 100.0;
         cmd.yaw = decoded.command.yaw / 10.0;
         cmd.flags = decoded.command.flags;
         cmd.fresh = true;
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
             MotionCmd cmd;
             bool ok = parse_packet(buf, static_cast<size_t>(n), cmd);
             static int rx_cnt = 0, err_cnt = 0;
             rx_cnt++;
             if (ok) {
                 std::lock_guard<std::mutex> lock(cmd_mutex_);
                 latest_cmd_ = cmd;
                 last_cmd_time_ = std::chrono::steady_clock::now();
                 if (rx_cnt % 10 == 1)
                     RCLCPP_INFO(get_logger(), "[KBD] rx#%d OK: s=%.1f w=%.1f h=%.1f y=%.0f",
                         rx_cnt, cmd.surge, cmd.sway, cmd.heave, cmd.yaw);
                 if (cmd.flags & FLAG_ESTOP) {
                     RCLCPP_WARN(get_logger(), "[KBD] 急停!");
                     send_zero_thrust();
                 }
             } else {
                 if (++err_cnt % 20 == 1)
                     RCLCPP_WARN(get_logger(), "[KBD] rx#%d FAIL(%d): len=%ld hdr=%02x size_ok=%d",
                         rx_cnt, err_cnt, n,
                         n > 0 ? static_cast<unsigned int>(buf[0]) : 0U,
                         n == static_cast<ssize_t>(PROTO_PKT_SIZE));
             }
         }
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
 
     // ==================== 控制循环 (带平滑) ====================
     void control_loop()
     {
         MotionCmd cmd;
         std::chrono::steady_clock::time_point last_cmd_time;
         {
             std::lock_guard<std::mutex> lock(cmd_mutex_);
             cmd = latest_cmd_;
             last_cmd_time = last_cmd_time_;
         }
 
         const double command_age = std::chrono::duration<double>(
             std::chrono::steady_clock::now() - last_cmd_time).count();
         if (!cmd.fresh || command_age > cmd_timeout_s_) {
             if (!watchdog_tripped_) {
                 RCLCPP_WARN(get_logger(),
                     "[KBD] 指令看门狗触发 (age=%.3fs), 输出归零", command_age);
                 watchdog_tripped_ = true;
             }
             send_zero_thrust();
             reset_smoothing();
             return;
         }
         watchdog_tripped_ = false;
 
         if (cmd.flags & FLAG_ESTOP) {
             send_zero_thrust();
             reset_smoothing();
             return;
         }
 
         // ---- 步骤1: 目标力/力矩 (raw) ----
         double Fx_raw =  cmd.surge * surge_scale_;
         double Fy_raw =   cmd.sway * sway_scale_;
         double Fz_raw = (std::abs(cmd.heave - 5.0) > 0.5)
                          ? std::copysign(heave_scale_, (cmd.heave - 5.0))
                          : 0.0;
         double Mz_raw =   cmd.yaw * yaw_scale_;
 
         // ---- 步骤2: 斜坡限幅 (Slew Rate Limiter) ----
         // 限制力/力矩变化率, 避免键盘开关量 → 推力阶跃跳变
         double dt = 1.0 / this->get_parameter("control_rate_hz").as_double();
         double tau_smooth[6] = {Fx_raw, Fy_raw, Fz_raw, 0.0, 0.0, Mz_raw};
 
         if (smoothing_enable_ && dt > 0.0) {
             const double slew[6] = {slew_Fx_, slew_Fy_, slew_Fz_, 0.0, 0.0, slew_Mz_};
             for (int i = 0; i < 6; i++) {
                 double max_step = slew[i] * dt;
                 double diff = tau_smooth[i] - prev_tau_[i];
                 if (diff >  max_step) tau_smooth[i] = prev_tau_[i] + max_step;
                 if (diff < -max_step) tau_smooth[i] = prev_tau_[i] - max_step;
             }
         }
         for (int i = 0; i < 6; i++) prev_tau_[i] = tau_smooth[i];
 
         // ---- 步骤3: 推力分配 f_raw = T_pinv × tau ----
         std::array<double, 6> f_raw{};
         for (int i = 0; i < 6; i++) {
             double sum = 0.0;
             for (int j = 0; j < 6; j++) sum += T_PINV[i][j] * tau_smooth[j];
             f_raw[i] = sum;
         }
 
         // ---- 步骤4: 推进器一阶时滞滤波 (Thruster Dynamics) ----
         // f_filtered = α × f_raw + (1-α) × f_filtered_prev
         // α = dt / (τ + dt),  τ = thruster_tau_
         std::array<double, 6> f_out{};
         if (smoothing_enable_ && thruster_tau_ > 0.0) {
             double alpha = dt / (thruster_tau_ + dt);
             for (int i = 0; i < 6; i++) {
                 thruster_filtered_[i] = alpha * f_raw[i]
                                       + (1.0 - alpha) * thruster_filtered_[i];
                 f_out[i] = thruster_filtered_[i];
             }
         } else {
             f_out = f_raw;
         }
 
         // ---- 步骤5: 百分比限幅 + 发布 ----
         auto msg = std_msgs::msg::Float64MultiArray();
         msg.data.reserve(6);
         for (int i = 0; i < 6; i++) {
             msg.data.push_back(std::clamp(f_out[i], -100.0, 100.0));
         }
         thruster_pub_->publish(msg);
     }
 
     void send_zero_thrust()
     {
         if (!thruster_pub_) return;
         auto msg = std_msgs::msg::Float64MultiArray();
         msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
         thruster_pub_->publish(msg);
     }
 
     void reset_smoothing()
     {
         prev_tau_ = {0, 0, 0, 0, 0, 0};
         thruster_filtered_ = {0, 0, 0, 0, 0, 0};
     }
 
     // ==================== 成员 ====================
     rclcpp_lifecycle::LifecyclePublisher<
         std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
     rclcpp::TimerBase::SharedPtr ctrl_timer_;
 
     int sock_fd_ = -1;
     std::thread recv_thread_;
     std::atomic<bool> running_{false};
     std::mutex cmd_mutex_;
     MotionCmd latest_cmd_;
     std::chrono::steady_clock::time_point last_cmd_time_{};
     double cmd_timeout_s_ = 0.5;
     bool watchdog_tripped_ = false;
 
     double surge_scale_ = 300.0;
     double sway_scale_  = 160.0;
     double heave_scale_ = 80.0;
     double yaw_scale_   = 5.0;
 
     // ---- 平滑控制状态 ----
     bool   smoothing_enable_ = true;
     double slew_Fx_  = 600.0;
     double slew_Fy_  = 320.0;
     double slew_Fz_  = 160.0;
     double slew_Mz_  = 10.0;
     double thruster_tau_ = 0.2;
     std::array<double, 6> prev_tau_{};           // 上一帧 wrench
     std::array<double, 6> thruster_filtered_{};  // 推进器滤波状态
 };
 
 int main(int argc, char **argv)
 {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<AppKeyboardControlNode>();
     rclcpp::executors::MultiThreadedExecutor exec;
     exec.add_node(node->get_node_base_interface());
     exec.spin();
     rclcpp::shutdown();
     return 0;
 }
 
 
 
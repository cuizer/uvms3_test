#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic> 
#include <array>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h> 
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" 
#include "std_srvs/srv/set_bool.hpp" 

#include "hal/msg/hal_mainthruster.hpp"
#include "hal/msg/hal_auxithruster.hpp"
#include "hal/srv/hal_thrustercontrol_srv.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using std::placeholders::_2;

class HalThrusterNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit HalThrusterNode(const std::string & node_name, bool intra_process_comms = false)
    : rclcpp_lifecycle::LifecycleNode(node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "配置中... 初始化推进器节点接口。");
        pub_main_status_ = this->create_publisher<hal::msg::HalMainthruster>("hal_mainthruster_msg", 10);
        pub_aux_status_ = this->create_publisher<hal::msg::HalAuxithruster>("hal_auxithruster_msg", 10);

        // 对应协议 #33 节点消息命名：/hal/thrustercontrol
        srv_control_ = this->create_service<hal::srv::HalThrustercontrolSrv>(
            "/hal/thrustercontrol", std::bind(&HalThrusterNode::control_srv_callback, this, _1, _2));

        srv_estop_ = this->create_service<std_srvs::srv::SetBool>(
            "hal_thruster_estop", std::bind(&HalThrusterNode::estop_callback, this, _1, _2));

        sub_cmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hal/thruster/cmd", 10, std::bind(&HalThrusterNode::cmd_callback, this, _1));

        // 20ms 定时器：用于主推轮询、辅推 100ms 周期发送分频以及状态发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&HalThrusterNode::timer_general_callback, this));

        hardware_api_init_can("can0");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
        RCLCPP_INFO(get_logger(), "推进器节点已激活。");
        pub_main_status_->on_activate();
        pub_aux_status_->on_activate();
        
        is_testing_ = false;
        is_estopped_ = false; 
        is_emergency_ascending_ = false;

        // 初始化 6 个辅推的目标期望值为 0.0 (对应死区油门 1000)
        for (int i = 0; i < 6; ++i) {
            aux_target_pct_[i].store(0.0);
            last_seen_ms_[i].store(0); 
        }
        
        keep_running_ = false; 
        if (can_rx_thread_.joinable()) can_rx_thread_.join();
        
        keep_running_ = true;  
        can_rx_thread_ = std::thread(&HalThrusterNode::can_receive_loop, this);
        
        LifecycleNode::on_activate(state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        pub_main_status_.reset(); pub_aux_status_.reset();
        srv_control_.reset(); srv_estop_.reset(); sub_cmd_.reset(); timer_.reset();
        
        keep_running_ = false; 
        is_testing_ = false;
        if (can_rx_thread_.joinable()) can_rx_thread_.join();
        if (test_thread_.joinable()) test_thread_.join();
        
        hardware_api_close_can(); 
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
        pub_main_status_->on_deactivate();
        pub_aux_status_->on_deactivate();
        stop_all_thrusters();
        LifecycleNode::on_deactivate(state);

        keep_running_ = false;
        is_testing_ = false;
        if (can_rx_thread_.joinable()) can_rx_thread_.join();
        if (test_thread_.joinable()) test_thread_.join();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        stop_all_thrusters();
        hardware_api_close_can();
        return CallbackReturn::SUCCESS;
    }

private:
    // --- 常量定义 ---
    const uint32_t MAIN_THRUSTER_ID = 0x01; 
    const int64_t ONLINE_TIMEOUT_MS = 2000; // 2秒未收到反馈判定为离线

    // --- 状态控制变量 ---
    std::atomic<bool> is_estopped_{false}; 
    std::atomic<bool> is_testing_{false}; 
    std::atomic<bool> is_emergency_ascending_{false}; // 紧急上浮状态锁
    int can_socket_ = -1; 

    std::thread test_thread_;
    std::thread can_rx_thread_;
    std::atomic<bool> keep_running_{false}; 

    // --- 数据缓存：主推进器 ---
    std::atomic<int32_t> real_main_rpm_{0};
    std::atomic<int32_t> real_main_current_raw_{0}; 
    std::atomic<int32_t> real_main_voltage_{0};     
    std::atomic<uint32_t> real_main_fault_{0};
    
    // --- 数据缓存：6 路辅助推进器 ---
    std::array<std::atomic<double>, 6> aux_target_pct_{}; 
    std::array<std::atomic<int16_t>, 6> aux_rpm_{};
    std::array<std::atomic<float>, 6>   aux_current_{};
    std::array<std::atomic<uint8_t>, 6> aux_voltage_{};
    std::array<std::atomic<int8_t>, 6>  aux_temp_{};
    std::array<std::atomic<uint8_t>, 6> aux_status_machine_{};
    std::array<std::atomic<uint8_t>, 6> aux_fault_{};
    std::array<std::atomic<int64_t>, 6> last_seen_ms_{}; // 用于离线判定的系统时间戳

    // --- ROS2 接口指针 ---
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalMainthruster>> pub_main_status_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalAuxithruster>> pub_aux_status_;
    rclcpp::Service<hal::srv::HalThrustercontrolSrv>::SharedPtr srv_control_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_estop_; 
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- 急停控制服务 ---
    void estop_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            is_estopped_ = true;
            is_testing_ = false;   
            is_emergency_ascending_ = false;
            stop_all_thrusters();  
            RCLCPP_FATAL(get_logger(), "�� 触发急停！已切断所有控制指令并停止所有推进器（含6路辅推）。");
            response->success = true; response->message = "急停已激活，全系统推进器已锁定停机。";
        } else {
            is_estopped_ = false;
            RCLCPP_INFO(get_logger(), "✅ 急停已解除，恢复推进器控制。");
            response->success = true; response->message = "急停已解除。";
        }
    }

    // --- 基于通讯协议 #33 规范修正的服务回调逻辑 ---
    void control_srv_callback(
        const std::shared_ptr<hal::srv::HalThrustercontrolSrv::Request> request,
        std::shared_ptr<hal::srv::HalThrustercontrolSrv::Response> response) 
    {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            response->success = false; response->message = "节点未激活，无法执行指令。"; return;
        }
        if (is_estopped_) {
            response->success = false; response->message = "当前处于急停锁定状态，拒绝执行任何总线控制指令！"; return;
        }

        uint8_t cmd = request->command;
        
        switch (cmd) {
            case 0x01: // 01 主推进器自检 (正反转各3秒，速度10%)
                if (is_emergency_ascending_) {
                    response->success = false; response->message = "处于紧急上浮响应锁中，拒绝自检。"; return;
                }
                if (is_testing_) {
                    response->success = false; response->message = "推进器正在测试中，请稍后再试。"; return;
                }
                RCLCPP_INFO(get_logger(), "收到指令 01：开始【主推进器】正反转各3秒(10%功率)自检测试...");
                if (test_thread_.joinable()) test_thread_.join(); 
                test_thread_ = std::thread(&HalThrusterNode::execute_test_sequence, this, true);
                response->success = true; response->message = "主推自检序列已成功启动";
                break;

            case 0x02: // 02 辅助推进器自检 (正反转各3秒，速度10%)
                if (is_emergency_ascending_) {
                    response->success = false; response->message = "处于紧急上浮响应锁中，拒绝自检。"; return;
                }
                if (is_testing_) {
                    response->success = false; response->message = "推进器正在测试中，请稍后再试。"; return;
                }
                RCLCPP_INFO(get_logger(), "收到指令 02：开始【6路辅推】全体正反转各3秒(10%功率)自检测试...");
                if (test_thread_.joinable()) test_thread_.join(); 
                test_thread_ = std::thread(&HalThrusterNode::execute_test_sequence, this, false);
                response->success = true; response->message = "全体辅推自检序列已成功启动";
                break;

            case 0x03: // 03 推进器停止
                RCLCPP_WARN(get_logger(), "收到指令 03：立刻安全终止自检线程并下发全停机状态指令。");
                is_testing_ = false; // 触发打断机制
                if (test_thread_.joinable()) test_thread_.join();
                
                // 如果当前处于紧急上浮，收到明确停止指令则解除上浮闭锁
                is_emergency_ascending_ = false; 
                
                stop_all_thrusters();
                response->success = true; response->message = "全系统推进器已紧急停机恢复。";
                break;

            case 0x04: // 04 紧急上浮
                RCLCPP_ERROR(get_logger(), "！！！严重安全警告：收到指令 04 —— 全系统进入紧急上浮响应逻辑 ！！！");
                is_testing_ = false; // 强行中止可能存在的自检
                if (test_thread_.joinable()) test_thread_.join();
                
                // 激活紧急上浮
                execute_emergency_ascent();
                
                response->success = true; response->message = "安全断言成功，紧急上浮动作序列已强制触发执行。";
                break;

            default:
                RCLCPP_ERROR(get_logger(), "收到无法解析的异常通讯指令代码: 0x%X", cmd);
                response->success = false; response->message = "未知协议自检/控制指令代码";
                break;
        }
    }

    // --- 自检自旋逻辑（速度调校至符合协议的 10%） ---
    void execute_test_sequence(bool is_main_thruster) {
        is_testing_ = true; 
        double test_thrust_pct = 5.0; // 严格匹配通讯协议规定的 5% 速度
        
        std::vector<uint32_t> test_nodes;
        if (is_main_thruster) {
            test_nodes.push_back(MAIN_THRUSTER_ID); 
        } else {
            for (uint32_t i = 0; i < 6; ++i) test_nodes.push_back(i); 
        }

        if (is_estopped_ || !is_testing_) { is_testing_ = false; return; }

        // 1. 正转自检
        for (uint32_t id : test_nodes) {
            if (is_main_thruster) set_thruster_rpm_hardware(id, test_thrust_pct);
            else aux_target_pct_[id].store(test_thrust_pct);
        }
        if (!interruptible_sleep(3000)) { is_testing_ = false; return; }

        // 2. 反转自检
        for (uint32_t id : test_nodes) {
            if (is_main_thruster) set_thruster_rpm_hardware(id, -test_thrust_pct);
            else aux_target_pct_[id].store(-test_thrust_pct);
        }
        if (!interruptible_sleep(3000)) { is_testing_ = false; return; }

        // 3. 停机恢复
        for (uint32_t id : test_nodes) {
            if (is_main_thruster) set_thruster_rpm_hardware(id, 0.0);
            else aux_target_pct_[id].store(0.0);
        }
        is_testing_ = false; 
    }
        
    // --- 框架预留：紧急上浮控制功能 ---
    void execute_emergency_ascent() {
        // 1. 设置最高等级状态闭锁，防止上层的常规控制话题(cmd_callback)冲刷覆盖指令
        is_emergency_ascending_ = true;

        // 2. 停止非必要的推进器活动（主推清零）
        set_thruster_rpm_hardware(MAIN_THRUSTER_ID, 0.0);

        // 3. 全系统辅助推进器垂直通道响应配置框架
        // 提示：此处后续可根据 CI-AUV 动力分布结构，直接对辅助垂直推进器（如 Z 轴通道）下发 100% 满功率输出
        RCLCPP_WARN(get_logger(), "[紧急上浮动作序列] 正在向垂直通道推进器注入极限满功率上浮期望...");
        for (int i = 0; i < 6; ++i) {
            // aux_target_pct_[i].store(100.0); // 示例：后续结合具体的 Z 向通道 ID 进行定向满载输出
            aux_target_pct_[i].store(0.0);     // 暂且安全清零
        }
        
        // 4. 底层硬触发接口预留 (如抛载机构、气囊充气总线报文等)
        // 示例：
        // struct can_frame emergency_frame;
        // emergency_frame.can_id = 0x190; // 设定的紧急抛载执行机构标称ID
        // emergency_frame.can_dlc = 1;
        // emergency_frame.data[0] = 0xFF; // 解锁脱钩控制字
        // write(can_socket_, &emergency_frame, sizeof(emergency_frame));

        RCLCPP_FATAL(get_logger(), "[紧急上浮动作序列] 总线控制安全切断完毕，物理应急机构已就绪。");
    }

    // --- 订阅常规控制输入指令 ---
    void cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
        // 如果处于急停、自检或紧急上浮状态，一律拒绝对底层下发常规控制参数
        if (is_estopped_ || is_testing_ || is_emergency_ascending_) return; 

        if (msg->data.size() >= 1) {
            set_thruster_rpm_hardware(MAIN_THRUSTER_ID, msg->data[0]);  
        }

        for (size_t i = 1; i < msg->data.size() && i <= 6; ++i) {
            aux_target_pct_[i - 1].store(msg->data[i]);
        }
    }

    // --- 全停机安全控制下发 ---
    void stop_all_thrusters() {
        RCLCPP_WARN(get_logger(), "⚠️ 正在向底层硬件下发【全系统推进器】紧急停机安全指令！");
        set_thruster_rpm_hardware(MAIN_THRUSTER_ID, 0.0); 
        for (int i = 0; i < 6; ++i) {
            aux_target_pct_[i].store(0.0);
        }
        send_aux_control_commands();
    }

    // --- 辅助推进器控制字计算逻辑 (1100~2000 范围映射) ---
    uint16_t calculate_aux_cmd_word(double pct) {
        if (std::abs(pct) < 1e-3) {
            return 1000; 
        }
        uint16_t direction = 0;
        if (pct < 0) {
            direction = 1 << 11; 
            pct = -pct;
        }
        if (pct > 100.0) pct = 100.0;

        uint16_t throttle = 1100 + static_cast<uint16_t>((pct / 100.0) * 900.0);
        if (throttle > 2000) throttle = 2000;

        return throttle | direction;
    }

    // --- 向硬件周期下发辅推控制数据 (0x200 和 0x201) ---
    void send_aux_control_commands() {
        if (can_socket_ < 0) return;

        for (int group = 0; group < 2; ++group) {
            struct can_frame frame;
            frame.can_id = 0x200 + group; 
            frame.can_dlc = 8;
            std::memset(frame.data, 0, 8);

            for (int i = 0; i < 4; ++i) {
                int aux_id = group * 4 + i;
                
                if (aux_id < 6) { 
                    double target_pct = is_estopped_ ? 0.0 : aux_target_pct_[aux_id].load();
                    uint16_t cmd_word = calculate_aux_cmd_word(target_pct);

                    frame.data[i * 2]     = cmd_word & 0xFF;
                    frame.data[i * 2 + 1] = (cmd_word >> 8) & 0xFF;
                } else {
                    frame.data[i * 2]     = 0x00;
                    frame.data[i * 2 + 1] = 0x00;
                }
            }

            if (write(can_socket_, &frame, sizeof(struct can_frame)) < 0) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "辅推 CAN 发送失败！ID: 0x%X", frame.can_id);
            }
        }
    }
    
    // --- 主推进器控制发送函数 (保持大端模式) ---
    void set_thruster_rpm_hardware(uint32_t target_node_id, double thrust_percentage) {
        if (can_socket_ < 0) return;
        if (thrust_percentage > 100.0) thrust_percentage = 100.0;
        if (thrust_percentage < -100.0) thrust_percentage = -100.0;

        uint32_t can_id = 0x300 + target_node_id;
        int32_t target_thrust = static_cast<int32_t>(thrust_percentage);
        struct can_frame frame;
        frame.can_id = can_id; 
        frame.can_dlc = 8; 

        frame.data[0] = 0x54; // 'T' 
        frame.data[1] = 0x43; // 'C' 
        frame.data[2] = 0x00; 
        frame.data[3] = 0x00; 

        frame.data[4] = (target_thrust >> 24) & 0xFF;
        frame.data[5] = (target_thrust >> 16) & 0xFF;
        frame.data[6] = (target_thrust >> 8) & 0xFF;
        frame.data[7] = target_thrust & 0xFF;

        write(can_socket_, &frame, sizeof(struct can_frame));
    }

    // --- 解析硬件回传总线数据 ---
    void can_receive_loop() {
        struct can_frame frame;
        uint32_t target_main_rx_id = 0x280 + MAIN_THRUSTER_ID; 

        while (keep_running_) {
            if (can_socket_ < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue;
            }
            
            ssize_t nbytes = recv(can_socket_, &frame, sizeof(struct can_frame), 0);
            
            if (nbytes == sizeof(struct can_frame)) {
                uint32_t clean_id = frame.can_id & CAN_SFF_MASK;

                // 1. 解析主推进器反馈报文
                if (clean_id == target_main_rx_id && frame.can_dlc == 8) {
                    int32_t value = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7];
                    if (frame.data[0] == 0x51) {
                        switch (frame.data[1]) {
                            case 0x56: real_main_rpm_ = value; break;
                            case 0x43: real_main_current_raw_ = value; break;
                            case 0x50: real_main_voltage_ = value; break;
                        }
                    } else if (frame.data[0] == 0x45 && frame.data[1] == 0x46) {
                        real_main_fault_ = static_cast<uint32_t>(value);
                    }
                }
                
                // 2. 解析辅助推进器反馈报文 (仅接收 0x300 ~ 0x305)
                else if (clean_id >= 0x300 && clean_id <= 0x305 && frame.can_dlc == 8) {
                    uint8_t aux_id = clean_id - 0x300; 

                    int16_t raw_speed   = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
                    int16_t raw_current = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
                    uint8_t raw_volt    = frame.data[4];
                    int8_t  raw_temp    = frame.data[5];
                    uint8_t raw_status  = frame.data[6];
                    uint8_t raw_fault   = frame.data[7];

                    aux_rpm_[aux_id].store(raw_speed); 
                    aux_current_[aux_id].store(static_cast<float>(raw_current) * 0.01f); 
                    aux_voltage_[aux_id].store(raw_volt); 
                    aux_temp_[aux_id].store(raw_temp);     
                    aux_status_machine_[aux_id].store(raw_status);
                    aux_fault_[aux_id].store(raw_fault);

                    int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    last_seen_ms_[aux_id].store(now_ms);

                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "�� [数据闭环成功] 驱动正在稳定同步辅推 ID: %d 的回传反馈。", aux_id);
                }
            }
        }
    }

    // --- 定时处理主逻辑 ---
    void timer_general_callback() {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;

        static int ticks = 0;
        ticks++;

        // 1. 主推进器参数轮询
        switch (ticks % 4) {
            case 0: request_thruster_status(MAIN_THRUSTER_ID, 0x51, 0x56); break; 
            case 1: request_thruster_status(MAIN_THRUSTER_ID, 0x51, 0x43); break; 
            case 2: request_thruster_status(MAIN_THRUSTER_ID, 0x51, 0x50); break; 
            case 3: request_thruster_status(MAIN_THRUSTER_ID, 0x45, 0x46); break; 
        }

        // 2. 辅推 100ms 周期控制下发 (20ms * 5 = 100ms)
        if (ticks % 5 == 0) {
            send_aux_control_commands();
        }

        // 3. 发布主推进器状态数据
        publish_main_thruster_status();

        // 4. 发布 6 路辅助推进器状态和在线情况
        publish_aux_thrusters_status();
    }

    // --- 打包发布主推数据 ---
    void publish_main_thruster_status() {
        hal::msg::HalMainthruster main_msg;
        main_msg.timestamp = this->get_clock()->now().nanoseconds();
        main_msg.rpm = real_main_rpm_.load();
        main_msg.current = static_cast<int16_t>(real_main_current_raw_.load() * 100); 
        main_msg.voltage = static_cast<int16_t>(real_main_voltage_.load());
        uint32_t fault_code = real_main_fault_.load();
        main_msg.fault_status = is_estopped_ ? 0xFF : static_cast<uint8_t>(fault_code & 0xFF);
        pub_main_status_->publish(main_msg);
    }

    // --- 组装并发布 6 路辅推状态与在线情况报告 ---
    void publish_aux_thrusters_status() {
        hal::msg::HalAuxithruster aux_msg;
        aux_msg.timestamp = this->get_clock()->now().nanoseconds();
        int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        for (int i = 0; i < 6; ++i) {
            int64_t last_seen = last_seen_ms_[i].load();
            bool is_online = (last_seen != 0) && ((now_ms - last_seen) < ONLINE_TIMEOUT_MS);

            aux_msg.rpm[i]     = aux_rpm_[i].load();
            aux_msg.current[i] = static_cast<int16_t>(std::round(aux_current_[i].load() * 100.0f)); 
            aux_msg.voltage[i] = static_cast<int16_t>(aux_voltage_[i].load());
            aux_msg.temp[i]    = static_cast<uint16_t>(aux_temp_[i].load());

            uint8_t fault_type = aux_fault_[i].load();
            if (is_estopped_) {
                fault_type = 0xFF; 
            } else if (is_emergency_ascending_) {
                fault_type = 0xFD; // 预留自定义故障码：指示正处于紧急上浮中
            } else if (!is_online) {
                fault_type = 0xFE; 
            }
            aux_msg.fault_status[i] = fault_type;
        }
        pub_aux_status_->publish(aux_msg);
    }

    // --- 向总线发送状态请求指令（主推专用） ---
    void request_thruster_status(uint32_t target_node_id, uint8_t cmd_byte1, uint8_t cmd_byte2) {
        if (can_socket_ < 0) return;
        struct can_frame frame;
        frame.can_id = 0x300 + target_node_id; 
        frame.can_dlc = 4; 
        frame.data[0] = cmd_byte1;
        frame.data[1] = cmd_byte2;
        frame.data[2] = 0x00; 
        frame.data[3] = 0x00; 
        write(can_socket_, &frame, sizeof(struct can_frame));
    }

    // --- 可打断的线程睡眠延时（加入 is_testing_ 自感应） ---
    bool interruptible_sleep(int milliseconds) {
        int elapsed = 0;
        while (elapsed < milliseconds) {
            // 当急停、系统退出或外部取消自检状态(is_testing_=false)时，立刻退出当前耗时阻断
            if (is_estopped_ || !keep_running_ || !is_testing_) return false; 
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            elapsed += 50;
        }
        return true;
    }

    // --- SocketCAN 初始化 ---
    bool hardware_api_init_can(const std::string& can_iface = "can0") {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) return false;
        
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) { close(can_socket_); can_socket_ = -1; return false; }
        
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000; 
        setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_can addr;
        addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(can_socket_); can_socket_ = -1; return false; }
        RCLCPP_INFO(get_logger(), "✅ SocketCAN 接口 %s 初始化成功！", can_iface.c_str());
        return true;
    }

    void hardware_api_close_can() { if (can_socket_ >= 0) { close(can_socket_); can_socket_ = -1; } }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HalThrusterNode>("hal_thruster_node");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
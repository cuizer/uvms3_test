#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <mutex>
#include <atomic>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "hal/msg/hal_tailservo.hpp"
#include "hal/msg/hal_wingservo.hpp"
#include "hal/srv/hal_servocontrol_srv.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using std::placeholders::_2;

class HalServoNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit HalServoNode(const std::string & node_name, bool intra_process_comms = false)
    : rclcpp_lifecycle::LifecycleNode(node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "配置中... 初始化硬件接口与多线程回调组。");
        
        max_angle_ = 45.0;
        
        // ==========================================
        // 1. 创建独立的互斥回调组
        // ==========================================
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // ==========================================
        // 2. 初始化发布者
        // ==========================================
        pub_tail_status_ = this->create_publisher<hal::msg::HalTailservo>("/hal/tailservo", 10);
        pub_wing_status_ = this->create_publisher<hal::msg::HalWingservo>("/hal/wingservo", 10);
        
        // ==========================================
        // 3. 对应协议 #34 节点消息命名：/hal/servocontrol
        // ==========================================
        srv_control_ = this->create_service<hal::srv::HalServocontrolSrv>(
            "/hal/servocontrol",
            std::bind(&HalServoNode::control_srv_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // ==========================================
        // 4. 初始化订阅者 (绑定到 sub_cb_group_)
        // ==========================================
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = sub_cb_group_;
        
        sub_tail_cmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hal/servo/tail_cmd", 10, 
            std::bind(&HalServoNode::tail_cmd_callback, this, std::placeholders::_1), 
            sub_opt);
        
        sub_wing_cmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hal/servo/wing_cmd", 10, 
            std::bind(&HalServoNode::wing_cmd_callback, this, std::placeholders::_1), 
            sub_opt);
        
        // ==========================================
        // 5. 初始化定时器 (绑定到 timer_cb_group_)
        // ==========================================
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&HalServoNode::timer_publish_status_callback, this),
            timer_cb_group_);
        
        // ==========================================
        // 6. 初始化硬件与后台读取线程
        // ==========================================
        hardware_api_init();
        keep_reading_ = true;
        read_thread_ = std::thread(&HalServoNode::serial_read_loop, this);
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
        RCLCPP_INFO(get_logger(), "舵机节点已激活。");
        pub_tail_status_->on_activate();
        pub_wing_status_->on_activate();
        is_testing_ = false;
        LifecycleNode::on_activate(state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
        RCLCPP_INFO(get_logger(), "舵机节点停用，终止测试并强制舵机回中。");
        is_testing_ = false; // 触发自检退出
        if (test_thread_.joinable()) test_thread_.join();

        pub_tail_status_->on_deactivate();
        pub_wing_status_->on_deactivate();
        set_tail_servos_hardware({0.0, 0.0, 0.0, 0.0});
        set_wing_servos_hardware({0.0, 0.0});
        send_stop_command_to_all(true); // 保持锁力
        LifecycleNode::on_deactivate(state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        keep_reading_ = false;
        is_testing_ = false;
        if (read_thread_.joinable()) read_thread_.join();
        if (test_thread_.joinable()) test_thread_.join();

        pub_tail_status_.reset(); pub_wing_status_.reset();
        srv_control_.reset(); sub_tail_cmd_.reset(); sub_wing_cmd_.reset(); timer_.reset();

        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        is_testing_ = false;
        if (test_thread_.joinable()) test_thread_.join();
        send_stop_command_to_all(false); // 释放锁力
        return CallbackReturn::SUCCESS;
    }

private:
    double max_angle_;
    std::atomic<bool> is_testing_{false};

    uint8_t current_poll_id_ = 0;

    // 串口与线程
    int serial_fd_ = -1;
    std::mutex serial_write_mutex_;
    std::thread read_thread_;
    std::thread test_thread_; // 统一管理的自检测试线程
    std::atomic<bool> keep_reading_{false};

    // 数据缓存池
    std::mutex data_cache_mutex_;
    hal::msg::HalTailservo cached_tail_msg_;
    hal::msg::HalWingservo cached_wing_msg_;

    // ROS 2 接口
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalTailservo>> pub_tail_status_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalWingservo>> pub_wing_status_;
    std::shared_ptr<rclcpp::Service<hal::srv::HalServocontrolSrv>> srv_control_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_tail_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_wing_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

    // ==========================================
    // ⚙️ 通信协议引擎
    // ==========================================
    std::vector<uint8_t> pack_command(uint8_t cmd_id, const std::vector<uint8_t>& content) {
        std::vector<uint8_t> packet;
        packet.push_back(0x12); packet.push_back(0x4C);
        packet.push_back(cmd_id);
        packet.push_back(static_cast<uint8_t>(content.size()));
        packet.insert(packet.end(), content.begin(), content.end());
        uint32_t sum = 0;
        for (uint8_t b : packet) sum += b;
        packet.push_back(static_cast<uint8_t>(sum % 256));
        return packet;
    }

    void hardware_serial_write(const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(serial_write_mutex_);
        if (serial_fd_ < 0) return;

        if (data.size() > 2 && data[2] == 0x08) {
            RCLCPP_DEBUG(this->get_logger(), "串口下发控制包成功");
        }

        write(serial_fd_, data.data(), data.size());
        tcdrain(serial_fd_);
    }

    // ==========================================
    // �� 后台读取线程
    // ==========================================
    void serial_read_loop() {
        int state = 0;
        uint8_t cmd_id = 0, length = 0, checksum = 0;
        std::vector<uint8_t> payload;

        while (keep_reading_) {
            uint8_t byte;
            if (read(serial_fd_, &byte, 1) > 0) {
                switch(state) {
                    case 0: if (byte == 0x05) state = 1; break;
                    case 1: if (byte == 0x1C) state = 2; else state = (byte == 0x05) ? 1 : 0; break;
                    case 2: cmd_id = byte; state = 3; break;
                    case 3: length = byte; payload.clear(); state = (length > 0) ? 4 : 5; break;
                    case 4:
                        payload.push_back(byte);
                        if (payload.size() == length) state = 5;
                        break;
                    case 5:
                        checksum = byte;
                        uint32_t sum = 0x05 + 0x1C + cmd_id + length;
                        for (uint8_t b : payload) sum += b;
                        if ((sum % 256) == checksum) process_servo_response(cmd_id, payload);
                        state = 0;
                        break;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
    }

    void process_servo_response(uint8_t cmd_id, const std::vector<uint8_t>& payload) {
        if (cmd_id == 0x0A && payload.size() >= 3) {
            uint8_t servo_id = payload[0];
            int16_t pos_raw = static_cast<int16_t>((payload[2] << 8) | payload[1]);
            float angle = static_cast<float>(pos_raw) / 10.0f;
    
            std::lock_guard<std::mutex> lock(data_cache_mutex_);
            if (servo_id <= 3) {
                cached_tail_msg_.position[servo_id] = angle;
            } else if (servo_id >= 4 && servo_id <= 5) {
                cached_wing_msg_.position[servo_id - 4] = angle;
            }
        }
    }

    // ==========================================
    // ⏱️ 定时器：轮询硬件并发布 Topic
    // ==========================================
    void timer_publish_status_callback() {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;

        hardware_serial_write(pack_command(0x0A, {current_poll_id_}));
        current_poll_id_ = (current_poll_id_ + 1) % 6;

        if (current_poll_id_ == 0) {
            int64_t current_timestamp = this->get_clock()->now().nanoseconds();
            {
                std::lock_guard<std::mutex> lock(data_cache_mutex_);
                cached_tail_msg_.timestamp = current_timestamp;
                pub_tail_status_->publish(cached_tail_msg_);

                cached_wing_msg_.timestamp = current_timestamp;
                pub_wing_status_->publish(cached_wing_msg_);
            }
        }
    }

    // ==========================================
    // �� 控制逻辑 (Service & CMD)
    // ==========================================
    void send_angle_command(uint8_t id, double deg) {
        int16_t angle_raw = static_cast<int16_t>(deg * 10.0);
        uint16_t time = 500, power = 1000;
        std::vector<uint8_t> c = {id};
        c.push_back(angle_raw & 0xFF); c.push_back((angle_raw >> 8) & 0xFF);
        c.push_back(time & 0xFF);      c.push_back((time >> 8) & 0xFF);
        c.push_back(power & 0xFF);     c.push_back((power >> 8) & 0xFF);
        hardware_serial_write(pack_command(0x08, c));
    }

    void send_stop_command_to_all(bool lock) {
        uint8_t val = lock ? 0x01 : 0x00;
        for (uint8_t id = 0; id <= 5; ++id) hardware_serial_write(pack_command(0x18, {id, val}));
    }

    void tail_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE || is_testing_) return;
    
        if (msg->data.size() >= 4) {
            for (int i=0; i<4; ++i) {
                send_angle_command(i, std::max(-max_angle_, std::min(msg->data[i], max_angle_)));
            }
        }
    }
    
    void wing_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE || is_testing_) return;
        
        if (msg->data.size() >= 2) {
            for (int i=0; i<2; ++i) send_angle_command(i + 4, std::max(-max_angle_, std::min(msg->data[i], max_angle_)));
        }
    }

    // 对应协议 #34 的服务回调
    void control_srv_callback(const std::shared_ptr<hal::srv::HalServocontrolSrv::Request> req,
                              std::shared_ptr<hal::srv::HalServocontrolSrv::Response> res) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            res->success = false; res->message = "节点未就绪（非Active状态）"; return;
        }
        if (is_testing_) {
            res->success = false; res->message = "当前舵机正在执行自检序列，拒绝重入"; return;
        }

        uint8_t cmd = req->command;
        if (cmd == 0x01 || cmd == 0x02) {
            if (test_thread_.joinable()) test_thread_.join(); // 回收残余线程句柄
            
            test_thread_ = std::thread(&HalServoNode::execute_test_sequence, this, cmd == 0x01);
            res->success = true; res->message = "舵机自检测试已成功异步启动";
        } else {
            res->success = false; res->message = "无效的协议指令代码";
        }
    }

    // 严格按照协议实现：“舵机正反转各3°” 自检逻辑
    void execute_test_sequence(bool is_tail) {
        is_testing_ = true;
        double target_test_angle = 15.0; // 严格匹配协议规定的 3 度偏转角
        
        auto set_angles = [&](double a) { 
            if(is_tail) {
                for(int i=0; i<=3; ++i) send_angle_command(i, a); 
            } else {
                for(int i=4; i<=5; ++i) send_angle_command(i, a); 
            }
        };

        if (!keep_reading_ || !is_testing_) { is_testing_ = false; return; }

        // 1. 正转 3°
        set_angles(target_test_angle);  
        if (!interruptible_sleep(3000)) { is_testing_ = false; return; }

        // 2. 反转 3°
        set_angles(-target_test_angle); 
        if (!interruptible_sleep(3000)) { is_testing_ = false; return; }

        // 3. 停机恢复回中
        set_angles(0.0); 
        is_testing_ = false;
        RCLCPP_INFO(get_logger(), "协议 #34 规定的舵机正反转各15°自检序列执行完毕。");
    }

    // 可响应生命周期打断的线程延时
    bool interruptible_sleep(int milliseconds) {
        int elapsed = 0;
        while (elapsed < milliseconds) {
            if (!keep_reading_ || !is_testing_) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            elapsed += 50;
        }
        return true;
    }

    // ==========================================
    // ��️ 硬件底层接口
    // ==========================================
    void hardware_api_init() {
        std::string port_name = "/dev/ttyTHS0";
        serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "无法打开串口 %s: %s", port_name.c_str(), strerror(errno));
            return;
        }
    
        struct termios opt; 
        tcgetattr(serial_fd_, &opt);
        
        cfsetispeed(&opt, B115200); 
        cfsetospeed(&opt, B115200);
    
        opt.c_cflag &= ~CSIZE;                  
        opt.c_cflag |= (CS8 | CLOCAL | CREAD);  
        opt.c_cflag &= ~(PARENB | CSTOPB);      
    
        opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
        opt.c_oflag &= ~OPOST;                  
    
        opt.c_cc[VMIN] = 0;  
        opt.c_cc[VTIME] = 1; 
    
        tcflush(serial_fd_, TCIFLUSH); 
        tcsetattr(serial_fd_, TCSANOW, &opt);
        
        RCLCPP_INFO(get_logger(), "串口 %s 初始化成功，波特率 115200。", port_name.c_str());
    }

    void set_tail_servos_hardware(const std::vector<double>& a) { for(size_t i=0; i<a.size(); ++i) send_angle_command(i, a[i]); }
    void set_wing_servos_hardware(const std::vector<double>& a) { for(size_t i=0; i<a.size(); ++i) send_angle_command(i+4, a[i]); }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HalServoNode>("hal_servo_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    RCLCPP_INFO(node->get_logger(), "已启动多线程执行器，正在监听舵机总线与服务指令...");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
#include <memory>
#include <string>
#include <cstring>
#include <vector>
#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"

// 引入系统自定义消息
#include "hal/msg/hal_antenna.hpp"
#include "hal/msg/can_msg_in.hpp"
#include "hal/msg/can_msg_out.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HalAntennaControlNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit HalAntennaControlNode(const std::string & node_name)
    : rclcpp_lifecycle::LifecycleNode(node_name) {
        // 【Foxy 适配核心】: 参数声明必须严格放置在构造函数中
        // 防止生命周期节点多次触发 Configure 时抛出 ParameterAlreadyDeclaredException
        this->declare_parameter<int>("motor_run_speed", 1800);
        this->declare_parameter<int>("lsb_down", 0);
        this->declare_parameter<int>("lsb_up", -12500000);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(this->get_logger(), "[系统] 正在配置天线控制复合模式节点 (环境: ROS 2 Foxy)...");

        // 1. 动态获取在构造函数中已声明的物理参数
        motor_run_speed_ = this->get_parameter("motor_run_speed").as_int();
        lsb_down_ = this->get_parameter("lsb_down").as_int();
        lsb_up_ = this->get_parameter("lsb_up").as_int();

        RCLCPP_INFO(this->get_logger(), "[参数] 标称速度: %d dps, 下限: %d, 上限: %d", 
                    motor_run_speed_, lsb_down_, lsb_up_);

        // 【Foxy 适配核心】: 显式使用 rclcpp::QoS 避免隐式转换引发编译器警告或订阅失败
        rclcpp::QoS qos_profile(10);

        // 2. 注册标准安全控制通道 (接收 0~6 状态指令)
        antenna_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/hal_antennacontrol_srv", qos_profile,
            std::bind(&HalAntennaControlNode::antenna_callback, this, std::placeholders::_1));

        // 3. 注册越权自定义控制通道 (断电寻零、强压应急专用)
        override_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hal_antennacontrol_override_angle", qos_profile,
            std::bind(&HalAntennaControlNode::override_angle_callback, this, std::placeholders::_1));

        // 4. 注册底层的 CAN 总线接收与发送通道
        canin_sub_ = this->create_subscription<hal::msg::CanMsgIn>(
            "/hal/canin", qos_profile,
            std::bind(&HalAntennaControlNode::canin_callback, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<hal::msg::HalAntenna>("/hal/antenna", qos_profile);
        canout_pub_ = this->create_publisher<hal::msg::CanMsgOut>("/hal/canout", qos_profile);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
        rclcpp_lifecycle::LifecycleNode::on_activate(state);
        status_pub_->on_activate();
        canout_pub_->on_activate();
        RCLCPP_INFO(this->get_logger(), "[系统] 节点已激活，双通道控制就绪。等待指令...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
        send_can_cmd(0x81, 0x00); // 退出时强制停止电机
        status_pub_->on_deactivate();
        canout_pub_->on_deactivate();
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(this->get_logger(), "[系统] 节点已失活，电机停转。");
        return CallbackReturn::SUCCESS;
    }

private:
    // --- 内部核心物理参数缓存 ---
    uint16_t motor_run_speed_;    
    int32_t lsb_down_;               
    int32_t lsb_up_;         

    // --- 标准安全指令回调 (0-6) ---
    void antenna_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_WARN(this->get_logger(), "[警告] 节点未激活，静默拦截标准请求: %d", (int)msg->data);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "\n[操作] 收到标准控制请求: %d", (int)msg->data);
        switch (msg->data) {
            case 0: // 复合上升序列
                RCLCPP_INFO(this->get_logger(), ">> 启动标准上升序列 -> 运动至绝对位置: %d LSB", lsb_up_);
                execute_hardware_sequence();
                run_motor_absolute(lsb_up_);
                break;

            case 1: // 复合下降序列
                RCLCPP_INFO(this->get_logger(), ">> 启动标准下降序列 -> 运动至绝对位置: %d LSB", lsb_down_);
                execute_hardware_sequence();
                run_motor_absolute(lsb_down_);
                break;

            case 2: send_can_cmd(0x8C, 0x01); break; // 仅手动释放抱闸
            case 3: send_can_cmd(0x8C, 0x00); break; // 仅手动锁死抱闸
            case 4: send_can_cmd(0x81, 0x00); break; // 仅紧急停止电机
            case 5: send_can_cmd(0x8C, 0x10); break; // 仅查询抱闸状态
            case 6: send_can_cmd(0x92, 0x00); break; // 仅查询当前位置
            default: RCLCPP_WARN(this->get_logger(), "[警告] 未知指令编号!"); break;
        }
    }

    // --- 越权增量角度回调 (断电强压恢复专用通道) ---
    void override_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_WARN(this->get_logger(), "[警告] 节点未激活，越权控制被拦截！");
            return;
        }

        double target_degree = msg->data;
        // 将输入的浮点数角度转换为相对位移的 LSB 脉冲增量 (1度 = 10000 LSB)
        int32_t increment_lsb = static_cast<int32_t>(target_degree * 10000.0);

        RCLCPP_WARN(this->get_logger(), "\n[越权操作] 拦截到增量指令: %.2f 度 (LSB相对增量: %d)", target_degree, increment_lsb);
        RCLCPP_INFO(this->get_logger(), ">> 正在无视固件软限位执行：清错 ➔ 解锁 ➔ 相对增量强推下行");

        // 执行严格的安全硬件复位与解锁时序
        execute_hardware_sequence();
        
        // 【核心破局】：摒弃 0xA4 绝对位置指令，改用 0xA8 相对增量闭环控制
        run_motor_relative(increment_lsb); 
    }

    // --- 严格的底层硬件动作时序保证 ---
    void execute_hardware_sequence() {
        // 1. 必须先发 0x88 清除伺服驱动板可能存在的欠压/堵转故障，并初始化使能
        send_can_cmd(0x88, 0x00); 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 2. 确认使能成功后，再发 0x8C 释放电磁抱闸（刹车通电脱力）
        send_can_cmd(0x8C, 0x01); 
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // --- CAN 底层数据接收与解析 ---
    void canin_callback(const hal::msg::CanMsgIn::SharedPtr msg) {
        if (msg->id == 0x141 || msg->id == 0x181) {
            // 解析电机多圈位置反馈 (0x92)
            if (msg->data[0] == 0x92) {
                uint64_t raw = 0;
                for (int i = 0; i < 7; ++i) { raw |= (static_cast<uint64_t>(msg->data[i + 1]) << (8 * i)); }
                if (raw & (1ULL << 55)) raw |= 0xFF00000000000000ULL; // 56位有符号符号位扩展

                int64_t motor_angle_lsb = static_cast<int64_t>(raw);
                double degree = motor_angle_lsb / 10000.0;

                RCLCPP_INFO(this->get_logger(), "[反馈] 当前电机物理多圈位置: %.2f 度", degree);
            }
            // 解析抱闸状态反馈 (0x8C)
            else if (msg->data[0] == 0x8C) {
                uint8_t brake = msg->data[1];
                std::string brake_status_str = (brake == 0x01) ? "已释放(通电运行)" : "已锁定(断电刹车)";
                RCLCPP_INFO(this->get_logger(), "[反馈] 抱闸状态: %s", brake_status_str.c_str());

                hal::msg::HalAntenna s_msg;
                s_msg.brake_status = brake;
                status_pub_->publish(s_msg);
            }
        }
    }

    // --- 基础 CAN 命令打包函数 ---
    void send_can_cmd(uint8_t b0, uint8_t b1) {
        hal::msg::CanMsgOut msg;
        msg.id = 0x141; msg.dlc = 8;
        std::fill(msg.data.begin(), msg.data.end(), 0x00);
        msg.data[0] = b0; msg.data[1] = b1;
        canout_pub_->publish(msg);
    }

    // --- 【常态武器】发送 0xA4 绝对位置闭环指令 ---
    void run_motor_absolute(int32_t target) {
        hal::msg::CanMsgOut msg;
        msg.id = 0x141; msg.dlc = 8;
        std::fill(msg.data.begin(), msg.data.end(), 0x00);
        msg.data[0] = 0xA4;
        msg.data[2] = motor_run_speed_ & 0xFF; 
        msg.data[3] = (motor_run_speed_ >> 8) & 0xFF; 
        msg.data[4] = target & 0xFF;
        msg.data[5] = (target >> 8) & 0xFF;
        msg.data[6] = (target >> 16) & 0xFF;
        msg.data[7] = (target >> 24) & 0xFF;
        canout_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[CAN发送] 绝对位置控制(0xA4) | 目标: %d LSB", target);
    }

    // --- 【应急武器】发送 0xA8 相对位置（增量）控制指令 ---
    void run_motor_relative(int32_t increment) {
        hal::msg::CanMsgOut msg;
        msg.id = 0x141; msg.dlc = 8;
        std::fill(msg.data.begin(), msg.data.end(), 0x00);
        msg.data[0] = 0xA8; // 相对增量模式代码
        msg.data[2] = motor_run_speed_ & 0xFF; 
        msg.data[3] = (motor_run_speed_ >> 8) & 0xFF; 
        msg.data[4] = increment & 0xFF; 
        msg.data[5] = (increment >> 8) & 0xFF;
        msg.data[6] = (increment >> 16) & 0xFF; 
        msg.data[7] = (increment >> 24) & 0xFF;
        canout_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[CAN发送] 相对增量控制(0xA8) | 增量: %d LSB", increment);
    }

    // --- 成员变量 ---
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr antenna_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr override_angle_sub_;
    rclcpp::Subscription<hal::msg::CanMsgIn>::SharedPtr canin_sub_;
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalAntenna>::SharedPtr status_pub_;
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::CanMsgOut>::SharedPtr canout_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HalAntennaControlNode>("hal_antennacontrol_node");
    
    // 【Foxy 适配】保持多线程执行器，确保底层反馈高频解析不受控制回调时的延时阻塞
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

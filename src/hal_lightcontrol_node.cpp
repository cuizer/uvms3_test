#include <memory>
#include <chrono>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/u_int8.hpp"

// 引入统一底层通信的自定义 CAN 消息
#include "hal/msg/can_msg_out.hpp" 

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HalLightControlNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit HalLightControlNode(const std::string & n) : rclcpp_lifecycle::LifecycleNode(n) {
        curr_ = 0; 
        targ_ = 0; 
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), ">>> [系统] 正在配置灯光控制节点 (基于统一 CAN Bridge) <<<");

        // Foxy 规范的 QoS 声明
        rclcpp::QoS qos_profile(10); 

        // 1. 订阅上层亮度控制指令
        sub_ = create_subscription<std_msgs::msg::UInt8>(
            "/hal_lightcontrol_srv", qos_profile, 
            [this](std_msgs::msg::UInt8::SharedPtr msg){ targ_ = msg->data; });

        // 2. 创建发布到 CAN Bridge 的出局通道，取代原生 SocketCAN
        canout_pub_ = create_publisher<hal::msg::CanMsgOut>("/hal/canout", qos_profile);
          
        // 3. 平滑渐变定时器 (50ms)
        tmr_ = create_wall_timer(std::chrono::milliseconds(50), [this](){
            // 【规范修复】使用标准宏代替硬编码的 3
            if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE || curr_ == targ_) return;
            
            // 平滑计算逻辑
            if (targ_ > curr_) {
                curr_ += std::min(static_cast<uint8_t>(5), static_cast<uint8_t>(targ_ - curr_));
            } else {
                curr_ -= std::min(static_cast<uint8_t>(5), static_cast<uint8_t>(curr_ - targ_));
            }
            
            // 封装并发送至 CAN 桥接器
            hal::msg::CanMsgOut can_msg;
            can_msg.id = 0x123;  // TODO: 需替换为您实际灯光板的 CAN ID
            can_msg.dlc = 8;
            std::fill(can_msg.data.begin(), can_msg.data.end(), 0);
            can_msg.data[0] = curr_;
            
            canout_pub_->publish(can_msg);
            RCLCPP_INFO(get_logger(), "[灯光渐变] 投递至 Bridge | 亮度: %u", curr_);
        });
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & s) override {
        rclcpp_lifecycle::LifecycleNode::on_activate(s);
        canout_pub_->on_activate(); // 生命周期发布者必须显式激活
        RCLCPP_INFO(get_logger(), "灯光节点已激活，平滑渐变任务就绪。");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & s) override {
        canout_pub_->on_deactivate();
        rclcpp_lifecycle::LifecycleNode::on_deactivate(s);
        RCLCPP_INFO(get_logger(), "灯光节点已失活。");
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_;
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::CanMsgOut>::SharedPtr canout_pub_;
    rclcpp::TimerBase::SharedPtr tmr_;
    uint8_t curr_, targ_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HalLightControlNode>("hal_lightcontrol_node");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}

#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <gpiod.h> // Linux 原生底层 GPIO 库

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HalLightSoftwarePwmNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit HalLightSoftwarePwmNode(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("hal_light_sw_pwm_node", options),
      duty_cycle_(0.0f),
      running_(false),
      gpio_line_(nullptr)
    {
        // ������ 核心物理坐标：已根据探测脚本更新为真实的硬件映射
        this->declare_parameter<std::string>("gpio_chip", "gpiochip300");
        this->declare_parameter<int>("gpio_offset", 6);
        
        // 根据万用表实测，输出 1 为 3.3V 高电平导通，因此 active_low 设为 false
        this->declare_parameter<bool>("active_low", false); 
        this->declare_parameter<int>("pwm_freq_hz", 50);    // 默认 50Hz 频率

        RCLCPP_INFO(this->get_logger(), ">>> [HAL 紫外大灯控制大脑] 已构建，处于 UNCONFIGURED 状态 <<<");
    }

    ~HalLightSoftwarePwmNode()
    {
        stop_pwm_thread();
        release_gpio();
    }

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        // 从参数服务器获取最新配置
        this->get_parameter("gpio_chip", gpio_chip_name_);
        this->get_parameter("gpio_offset", gpio_line_offset_);
        this->get_parameter("active_low", active_low_);
        
        int freq_hz = 50;
        this->get_parameter("pwm_freq_hz", freq_hz);
        pwm_period_us_ = 1000000 / freq_hz; 

        // 确定物理电平的逻辑极性
        on_value_ = active_low_ ? 0 : 1;  // 导通(亮起)时的物理电平
        off_value_ = active_low_ ? 1 : 0; // 截止(熄灭)时的物理电平

        RCLCPP_INFO(this->get_logger(), "--- [配置中] 锁定物理引脚: %s line %d | 极性: %s | 频率: %dHz ---", 
            gpio_chip_name_.c_str(), gpio_line_offset_, active_low_ ? "Active-LOW" : "Active-HIGH", freq_hz);

        // 1. 打开 GPIO 芯片
        struct gpiod_chip *chip = gpiod_chip_open_by_name(gpio_chip_name_.c_str());
        if (!chip) {
            RCLCPP_FATAL(this->get_logger(), "❌ 无法打开 %s！请确认是否使用 sudo 运行！", gpio_chip_name_.c_str());
            return CallbackReturn::FAILURE;
        }

        // 2. 独占获取引脚，并配置为输出模式，初始状态强制熄灭 (输出 off_value_)
        gpio_line_ = gpiod_chip_get_line(chip, gpio_line_offset_);
        if (!gpio_line_ || gpiod_line_request_output(gpio_line_, "hal_light_pwm", off_value_) < 0) {
            RCLCPP_FATAL(this->get_logger(), "❌ 无法申请引脚权限！请先执行: echo %d > /sys/class/gpio/unexport 释放旧锁", 306);
            gpiod_chip_close(chip);
            return CallbackReturn::FAILURE;
        }

        // 3. 订阅调光控制话题
        light_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hal/light_pwm", 10,
            std::bind(&HalLightSoftwarePwmNode::pwm_cmd_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "������ [配置成功] 引脚锁定完成。初始状态已断电保护。");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        LifecycleNode::on_activate(state);
        
        duty_cycle_ = 0.0f; // 激活时默认全灭
        running_ = true;
        pwm_thread_ = std::thread(&HalLightSoftwarePwmNode::software_pwm_loop, this);
        
        RCLCPP_WARN(this->get_logger(), "������������������ [节点已激活] 软件 PWM 后台线程已拉起，随时准备调光！");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_WARN(this->get_logger(), "������ [节点已钝化] 正在掐死高频线程并执行物理关灯...");
        stop_pwm_thread();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        release_gpio();
        light_cmd_sub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        stop_pwm_thread();
        release_gpio();
        return CallbackReturn::SUCCESS;
    }

private:
    void pwm_cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (this->get_current_state().id() != 3) return; // 仅在 ACTIVE 处理

        duty_cycle_ = std::clamp(msg->data, 0.0f, 1.0f);
        RCLCPP_INFO(this->get_logger(), "【大脑指令】接收调光信号 ➔ 目标亮度: %.0f%%", duty_cycle_.load() * 100.0);
    }

    void software_pwm_loop()
    {
        while (running_) {
            float current_duty = duty_cycle_.load();

            if (current_duty <= 0.01f) {
                // 彻底断电熄灭
                gpiod_line_set_value(gpio_line_, off_value_); 
                std::this_thread::sleep_for(std::chrono::microseconds(pwm_period_us_));
            } 
            else if (current_duty >= 0.99f) {
                // 火力全开爆亮
                gpiod_line_set_value(gpio_line_, on_value_); 
                std::this_thread::sleep_for(std::chrono::microseconds(pwm_period_us_));
            } 
            else {
                // 调光切分阶段
                int on_time_us = static_cast<int>(pwm_period_us_ * current_duty);
                int off_time_us = pwm_period_us_ - on_time_us;

                gpiod_line_set_value(gpio_line_, on_value_);
                std::this_thread::sleep_for(std::chrono::microseconds(on_time_us));

                gpiod_line_set_value(gpio_line_, off_value_);
                std::this_thread::sleep_for(std::chrono::microseconds(off_time_us));
            }
        }
    }

    void stop_pwm_thread()
    {
        running_ = false;
        if (pwm_thread_.joinable()) {
            pwm_thread_.join();
        }
        if (gpio_line_) {
            gpiod_line_set_value(gpio_line_, off_value_); // 强行补发关灯指令
        }
    }

    void release_gpio()
    {
        if (gpio_line_) {
            gpiod_line_release(gpio_line_);
            gpio_line_ = nullptr;
        }
    }

    // --- 动态参数变量 ---
    std::string gpio_chip_name_;
    int gpio_line_offset_;
    bool active_low_;
    int pwm_period_us_;
    int on_value_;
    int off_value_;
    
    std::atomic<float> duty_cycle_; 
    std::atomic<bool> running_;
    std::thread pwm_thread_;
    struct gpiod_line *gpio_line_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr light_cmd_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<HalLightSoftwarePwmNode>(options);
    
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    
    rclcpp::shutdown();
    return 0;
}
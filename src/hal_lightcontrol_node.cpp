#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>
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
        RCLCPP_INFO(this->get_logger(), ">>> [HAL I2C 软件调光大脑] 已构建，处于 UNCONFIGURED 状态 <<<");
        
        // ������ 核心物理坐标：内核真实映射
        gpio_chip_name_ = "gpiochip2"; 
        gpio_line_offset_ = 0;         
        
        // ⚠️ I2C 降频保护机制：
        // 由于引脚挂载在 I2C 扩展芯片上，100Hz 太快会导致总线拥堵。
        // 这里设置为 50Hz (周期 20,000 微秒)，在调光与总线稳定性之间取最佳平衡。
        pwm_period_us_ = 20000; 
    }

    ~HalLightSoftwarePwmNode()
    {
        stop_pwm_thread();
        release_gpio();
    }

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "--- [配置中] 正在独占锁定物理引脚: %s line %d ---", gpio_chip_name_.c_str(), gpio_line_offset_);

        // 1. 打开 GPIO 芯片
        struct gpiod_chip *chip = gpiod_chip_open_by_name(gpio_chip_name_.c_str());
        if (!chip) {
            RCLCPP_FATAL(this->get_logger(), "❌ 无法打开 %s！请确认是否使用 sudo (root) 运行！", gpio_chip_name_.c_str());
            return CallbackReturn::FAILURE;
        }

        // 2. 独占获取引脚，并配置为输出模式，初始电平为 1 (假设 1 为关，0 为开，即内核显示的 ACTIVE LOW)
        gpio_line_ = gpiod_chip_get_line(chip, gpio_line_offset_);
        if (!gpio_line_ || gpiod_line_request_output(gpio_line_, "hal_i2c_pwm", 1) < 0) {
            RCLCPP_FATAL(this->get_logger(), "❌ 无法申请引脚输出权限，可能被内核或其他程序死锁占用！");
            gpiod_chip_close(chip);
            return CallbackReturn::FAILURE;
        }

        // 3. 订阅调光话题 (0.0 ~ 1.0)
        light_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hal/light_pwm", 10,
            std::bind(&HalLightSoftwarePwmNode::pwm_cmd_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "������ [配置成功] 引脚锁定完成，I2C 链路畅通。");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        LifecycleNode::on_activate(state);
        
        // 启动独立的高频软 PWM 后台线程
        duty_cycle_ = 0.0f;
        running_ = true;
        pwm_thread_ = std::thread(&HalLightSoftwarePwmNode::software_pwm_loop, this);
        
        RCLCPP_WARN(this->get_logger(), "������������������ [节点已激活] 50Hz 软件 PWM 线程已拉起，随时准备调光！");
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
    // =========================================================================
    // 话题回调：极速更新目标占空比，不堵塞线程
    // =========================================================================
    void pwm_cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (this->get_current_state().id() != 3) return; // 必须是 ACTIVE(3) 状态

        // 限定占空比在 0.0 (全关) 到 1.0 (全开) 之间
        duty_cycle_ = std::clamp(msg->data, 0.0f, 1.0f);
        RCLCPP_INFO(this->get_logger(), "【大脑指令】接收调光信号 ➔ 目标亮度: %.0f%%", duty_cycle_.load() * 100.0);
    }

    // =========================================================================
    // 核心科技：独立线程执行的高频波形切分
    // =========================================================================
    void software_pwm_loop()
    {
        while (running_) {
            float current_duty = duty_cycle_.load();

            if (current_duty <= 0.01f) {
                // < 1%：彻底断电熄灭
                gpiod_line_set_value(gpio_line_, 1); // 物理写入高电平截止
                std::this_thread::sleep_for(std::chrono::microseconds(pwm_period_us_));
            } 
            else if (current_duty >= 0.99f) {
                // > 99%：火力全开爆亮
                gpiod_line_set_value(gpio_line_, 0); // 物理写入低电平导通
                std::this_thread::sleep_for(std::chrono::microseconds(pwm_period_us_));
            } 
            else {
                // 1% ~ 99% 调光切分：
                int on_time_us = static_cast<int>(pwm_period_us_ * current_duty);
                int off_time_us = pwm_period_us_ - on_time_us;

                // 1. 瞬间导通 (亮起)
                gpiod_line_set_value(gpio_line_, 0);
                std::this_thread::sleep_for(std::chrono::microseconds(on_time_us));

                // 2. 瞬间截止 (熄灭)
                gpiod_line_set_value(gpio_line_, 1);
                std::this_thread::sleep_for(std::chrono::microseconds(off_time_us));
            }
        }
    }

    // =========================================================================
    // 安全屏障：强制断电
    // =========================================================================
    void stop_pwm_thread()
    {
        running_ = false;
        if (pwm_thread_.joinable()) {
            pwm_thread_.join();
        }
        if (gpio_line_) {
            gpiod_line_set_value(gpio_line_, 1); // 线程死后，强行补发一条关灯指令
        }
    }

    void release_gpio()
    {
        if (gpio_line_) {
            gpiod_line_release(gpio_line_);
            gpio_line_ = nullptr;
        }
    }

    std::string gpio_chip_name_;
    int gpio_line_offset_;
    int pwm_period_us_;
    
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
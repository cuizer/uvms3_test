#include <rclcpp/rclcpp.hpp>

#include "hal/msg/hal_armmotor.hpp"

#include <chrono>
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

class ArmmotorAggregatorNode : public rclcpp::Node
{
public:
    ArmmotorAggregatorNode()
    : Node("armmotor_aggregator_node")
    {
        // ============================================================
        // 1. 参数声明
        //
        // left_arm_topic:
        //   左臂 HAL 节点发布的电机状态话题
        //
        // right_arm_topic:
        //   右臂 HAL 节点发布的电机状态话题
        //
        // output_topic:
        //   汇总后发布给上位机的总话题
        //
        // publish_rate_hz:
        //   汇总状态发布频率
        //
        // stale_timeout_sec:
        //   如果某一侧长时间没有更新，可以认为该侧数据过期
        //
        // startup_grace_sec:
        //   armmotor 节点启动后的静默时间。
        //   在这段时间内，即使左右臂状态还没收到，也不打印 stale warning。
        // ============================================================
        this->declare_parameter<std::string>(
            "left_arm_topic",
            "/left_arm/hal/armmotor");

        this->declare_parameter<std::string>(
            "right_arm_topic",
            "/right_arm/hal/armmotor");

        this->declare_parameter<std::string>(
            "output_topic",
            "/hal/armmotor");

        this->declare_parameter<double>(
            "publish_rate_hz",
            50.0);

        this->declare_parameter<double>(
            "stale_timeout_sec",
            0.5);

        // 新增：启动后前 30 秒不打印 stale warning，
        // 给手动 configure / activate 留出时间。
        this->declare_parameter<double>(
            "startup_grace_sec",
            30.0);

        // 左臂电机 ID：1,3,5,7,9
        this->declare_parameter<std::vector<int64_t>>(
            "left_motor_ids",
            {1, 3, 5, 7, 9});

        // 右臂电机 ID：2,4,6,8,10
        this->declare_parameter<std::vector<int64_t>>(
            "right_motor_ids",
            {2, 4, 6, 8, 10});

        left_arm_topic_ =
            this->get_parameter("left_arm_topic").as_string();

        right_arm_topic_ =
            this->get_parameter("right_arm_topic").as_string();

        output_topic_ =
            this->get_parameter("output_topic").as_string();

        publish_rate_hz_ =
            this->get_parameter("publish_rate_hz").as_double();

        stale_timeout_sec_ =
            this->get_parameter("stale_timeout_sec").as_double();

        startup_grace_sec_ =
            this->get_parameter("startup_grace_sec").as_double();

        left_motor_ids_param_ =
            this->get_parameter("left_motor_ids").as_integer_array();

        right_motor_ids_param_ =
            this->get_parameter("right_motor_ids").as_integer_array();

        // ============================================================
        // 2. 参数检查
        //
        // 当前设计默认：
        // 左臂 5 个电机：1,3,5,7,9
        // 右臂 5 个电机：2,4,6,8,10
        //
        // 汇总后输出数组长度固定为 10：
        // index 0 -> motor 1
        // index 1 -> motor 2
        // index 2 -> motor 3
        // ...
        // index 9 -> motor 10
        // ============================================================
        if (publish_rate_hz_ <= 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "publish_rate_hz <= 0, reset to 50.0 Hz.");
            publish_rate_hz_ = 50.0;
        }

        if (stale_timeout_sec_ <= 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "stale_timeout_sec <= 0, reset to 0.5 sec.");
            stale_timeout_sec_ = 0.5;
        }

        if (startup_grace_sec_ < 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "startup_grace_sec < 0, reset to 30.0 sec.");
            startup_grace_sec_ = 30.0;
        }

        if (left_motor_ids_param_.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "left_motor_ids is empty. Default left motor mapping will be unavailable.");
        }

        if (right_motor_ids_param_.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "right_motor_ids is empty. Default right motor mapping will be unavailable.");
        }

        // ============================================================
        // 3. 初始化本地缓存
        // ============================================================
        last_left_rx_time_ = this->now();
        last_right_rx_time_ = this->now();

        // 新增：记录 armmotor 节点启动时间。
        // timer_callback() 中会根据这个时间判断是否处于启动静默期。
        start_time_ = this->now();

        left_received_ = false;
        right_received_ = false;

        // ============================================================
        // 4. 创建订阅器
        //
        // 左臂 HAL 节点应发布：
        // /left_arm/hal/armmotor
        //
        // 右臂 HAL 节点应发布：
        // /right_arm/hal/armmotor
        // ============================================================
        auto qos = rclcpp::QoS(10);

        left_sub_ = this->create_subscription<hal::msg::HalArmmotor>(
            left_arm_topic_,
            qos,
            std::bind(
                &ArmmotorAggregatorNode::left_arm_callback,
                this,
                std::placeholders::_1));

        right_sub_ = this->create_subscription<hal::msg::HalArmmotor>(
            right_arm_topic_,
            qos,
            std::bind(
                &ArmmotorAggregatorNode::right_arm_callback,
                this,
                std::placeholders::_1));

        // ============================================================
        // 5. 创建发布器
        //
        // 这个话题是给上位机订阅的统一状态话题。
        // 上位机只需要订阅：
        // /hal/armmotor
        // ============================================================
        merged_pub_ = this->create_publisher<hal::msg::HalArmmotor>(
            output_topic_,
            qos);

        auto period_ms = std::chrono::milliseconds(
            static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_)));

        timer_ = this->create_wall_timer(
            period_ms,
            std::bind(&ArmmotorAggregatorNode::timer_callback, this));

        RCLCPP_INFO(
            get_logger(),
            "ArmmotorAggregatorNode started.");

        RCLCPP_INFO(
            get_logger(),
            "Subscribe left : %s",
            left_arm_topic_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Subscribe right: %s",
            right_arm_topic_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Publish merged: %s",
            output_topic_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Startup grace time: %.2f sec. Stale warnings will be suppressed during this period.",
            startup_grace_sec_);
    }

private:
    // ============================================================
    // 左臂状态回调
    // ============================================================
    void left_arm_callback(const hal::msg::HalArmmotor::SharedPtr msg)
    {
        latest_left_msg_ = *msg;
        left_received_ = true;
        last_left_rx_time_ = this->now();
    }

    // ============================================================
    // 右臂状态回调
    // ============================================================
    void right_arm_callback(const hal::msg::HalArmmotor::SharedPtr msg)
    {
        latest_right_msg_ = *msg;
        right_received_ = true;
        last_right_rx_time_ = this->now();
    }

    // ============================================================
    // 将单臂 5 个电机状态写入双臂 10 电机总数组
    //
    // 输入：
    //   arm_msg:
    //     某一侧 HAL 发布的 HalArmmotor
    //
    //   motor_ids:
    //     该侧电机 ID 列表
    //
    // 例如左臂：
    //   arm_msg.motor_current[0] -> motor 1
    //   arm_msg.motor_current[1] -> motor 3
    //   arm_msg.motor_current[2] -> motor 5
    //   arm_msg.motor_current[3] -> motor 7
    //   arm_msg.motor_current[4] -> motor 9
    //
    // 汇总后：
    //   merged.motor_current[0] -> motor 1
    //   merged.motor_current[2] -> motor 3
    //   merged.motor_current[4] -> motor 5
    //   merged.motor_current[6] -> motor 7
    //   merged.motor_current[8] -> motor 9
    //
    // 右臂同理：
    //   motor 2 -> index 1
    //   motor 4 -> index 3
    //   motor 6 -> index 5
    //   motor 8 -> index 7
    //   motor 10 -> index 9
    // ============================================================
    void copy_arm_state_to_merged(
        const hal::msg::HalArmmotor& arm_msg,
        const std::vector<int64_t>& motor_ids,
        hal::msg::HalArmmotor& merged_msg)
    {
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            int64_t motor_id = motor_ids[i];

            if (motor_id < 1 || motor_id > 10) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Invalid motor_id=%ld in aggregator mapping. Valid range is [1, 10].",
                    static_cast<long>(motor_id));
                continue;
            }

            // 电机 ID 1 对应数组下标 0；
            // 电机 ID 10 对应数组下标 9。
            size_t out_idx = static_cast<size_t>(motor_id - 1);

            if (i < arm_msg.motor_current.size()) {
                merged_msg.motor_current[out_idx] = arm_msg.motor_current[i];
            }

            if (i < arm_msg.motor_speed.size()) {
                merged_msg.motor_speed[out_idx] = arm_msg.motor_speed[i];
            }

            if (i < arm_msg.motor_position.size()) {
                merged_msg.motor_position[out_idx] = arm_msg.motor_position[i];
            }

            if (i < arm_msg.motor_temp.size()) {
                merged_msg.motor_temp[out_idx] = arm_msg.motor_temp[i];
            }

            if (i < arm_msg.motor_error.size()) {
                merged_msg.motor_error[out_idx] = arm_msg.motor_error[i];
            }

            if (i < arm_msg.hal_comm_error.size()) {
                 merged_msg.hal_comm_error[out_idx] = arm_msg.hal_comm_error[i];
             }
        }
    }

    // ============================================================
    // 周期发布双臂 10 电机汇总状态
    // ============================================================
    void timer_callback()
    {
        auto now = this->now();

        bool left_fresh = false;
        bool right_fresh = false;

        if (left_received_) {
            double dt_left = (now - last_left_rx_time_).seconds();
            left_fresh = (dt_left <= stale_timeout_sec_);
        }

        if (right_received_) {
            double dt_right = (now - last_right_rx_time_).seconds();
            right_fresh = (dt_right <= stale_timeout_sec_);
        }

        // ========================================================
        // 启动静默期判断
        //
        // armmotor 节点启动后的前 startup_grace_sec_ 秒，
        // 不打印 stale warning。
        //
        // 这样可以给用户手动 configure / activate 左右臂 HAL 节点
        // 留出时间，避免启动后立刻刷屏。
        // ========================================================
        double since_start = (now - start_time_).seconds();
        bool allow_stale_warning = (since_start > startup_grace_sec_);

        // 如果某一侧暂时没有数据，不阻塞总状态发布。
        // 没收到的一侧保持 0。
        //
        // 这里 warning 被分成两层控制：
        // 1. startup_grace_sec_：启动后前 30 秒不报警
        // 2. RCLCPP_WARN_THROTTLE 10000：超过 30 秒后，每 10 秒最多提醒一次
        if (allow_stale_warning && !left_fresh) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                10000,
                "Left armmotor state is stale or not received.");
        }

        if (allow_stale_warning && !right_fresh) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                10000,
                "Right armmotor state is stale or not received.");
        }

        hal::msg::HalArmmotor merged_msg;

        merged_msg.timestamp = now.nanoseconds();

        // ========================================================
        // 总状态固定为 10 个电机：
        //
        // motor_current[0] -> 电机 1
        // motor_current[1] -> 电机 2
        // motor_current[2] -> 电机 3
        // motor_current[3] -> 电机 4
        // motor_current[4] -> 电机 5
        // motor_current[5] -> 电机 6
        // motor_current[6] -> 电机 7
        // motor_current[7] -> 电机 8
        // motor_current[8] -> 电机 9
        // motor_current[9] -> 电机 10
        // ========================================================
        merged_msg.motor_current.assign(10, 0);
        merged_msg.motor_speed.assign(10, 0);
        merged_msg.motor_position.assign(10, 0);
        merged_msg.motor_temp.assign(10, 0);
        merged_msg.motor_error.assign(10, 0);

        merged_msg.hal_comm_error.assign(10, 1);

        if (left_fresh) {
            copy_arm_state_to_merged(
                latest_left_msg_,
                left_motor_ids_param_,
                merged_msg);
        }

        if (right_fresh) {
            copy_arm_state_to_merged(
                latest_right_msg_,
                right_motor_ids_param_,
                merged_msg);
        }

        merged_pub_->publish(merged_msg);
    }

private:
    // ============================================================
    // 参数
    // ============================================================
    std::string left_arm_topic_;
    std::string right_arm_topic_;
    std::string output_topic_;

    double publish_rate_hz_{50.0};
    double stale_timeout_sec_{0.5};

    // 新增：启动静默期，默认 30 秒。
    // 在这段时间内不打印左右臂状态 stale warning。
    double startup_grace_sec_{30.0};

    std::vector<int64_t> left_motor_ids_param_;
    std::vector<int64_t> right_motor_ids_param_;

    // ============================================================
    // 订阅器 / 发布器 / 定时器
    // ============================================================
    rclcpp::Subscription<hal::msg::HalArmmotor>::SharedPtr left_sub_;
    rclcpp::Subscription<hal::msg::HalArmmotor>::SharedPtr right_sub_;

    rclcpp::Publisher<hal::msg::HalArmmotor>::SharedPtr merged_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ============================================================
    // 左右臂最新状态缓存
    // ============================================================
    hal::msg::HalArmmotor latest_left_msg_;
    hal::msg::HalArmmotor latest_right_msg_;

    bool left_received_{false};
    bool right_received_{false};

    rclcpp::Time last_left_rx_time_;
    rclcpp::Time last_right_rx_time_;

    // 新增：armmotor 节点启动时间。
    // 用于判断是否仍处于 startup_grace_sec_ 静默期。
    rclcpp::Time start_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArmmotorAggregatorNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
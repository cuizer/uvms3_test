#include <rclcpp/rclcpp.hpp>

#include "hal/msg/hal_armmotor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class ArmmotorAggregatorNode : public rclcpp::Node
{
public:
    ArmmotorAggregatorNode()
    : Node("armmotor_aggregator_node")
    {
        this->declare_parameter<std::string>("left_arm_topic", "/left_arm/hal/armmotor");
        this->declare_parameter<std::string>("right_arm_topic", "/right_arm/hal/armmotor");
        this->declare_parameter<std::string>("output_topic", "/hal/armmotor");
        this->declare_parameter<double>("publish_rate_hz", 50.0);
        this->declare_parameter<double>("stale_timeout_sec", 0.5);
        this->declare_parameter<double>("startup_grace_sec", 30.0);

        this->declare_parameter<bool>("enable_csv_logging", false);
        this->declare_parameter<std::string>("csv_log_directory", "armmotor_logs");
        this->declare_parameter<std::string>("csv_log_file_prefix", "armmotor");
        this->declare_parameter<int>("csv_flush_every_n", 50);

        this->declare_parameter<std::vector<int64_t>>("left_motor_ids", {1, 3, 5, 7, 9});
        this->declare_parameter<std::vector<int64_t>>("right_motor_ids", {2, 4, 6, 8, 10});

        left_arm_topic_ = this->get_parameter("left_arm_topic").as_string();
        right_arm_topic_ = this->get_parameter("right_arm_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
        stale_timeout_sec_ = this->get_parameter("stale_timeout_sec").as_double();
        startup_grace_sec_ = this->get_parameter("startup_grace_sec").as_double();

        enable_csv_logging_ = this->get_parameter("enable_csv_logging").as_bool();
        csv_log_directory_ = this->get_parameter("csv_log_directory").as_string();
        csv_log_file_prefix_ = this->get_parameter("csv_log_file_prefix").as_string();
        csv_flush_every_n_ = static_cast<int>(
            this->get_parameter("csv_flush_every_n").as_int());

        left_motor_ids_param_ = this->get_parameter("left_motor_ids").as_integer_array();
        right_motor_ids_param_ = this->get_parameter("right_motor_ids").as_integer_array();

        if (publish_rate_hz_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "publish_rate_hz <= 0, reset to 50.0 Hz.");
            publish_rate_hz_ = 50.0;
        }

        if (stale_timeout_sec_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "stale_timeout_sec <= 0, reset to 0.5 sec.");
            stale_timeout_sec_ = 0.5;
        }

        if (startup_grace_sec_ < 0.0) {
            RCLCPP_WARN(get_logger(), "startup_grace_sec < 0, reset to 30.0 sec.");
            startup_grace_sec_ = 30.0;
        }

        if (csv_flush_every_n_ <= 0) {
            RCLCPP_WARN(get_logger(), "csv_flush_every_n <= 0, reset to 50.");
            csv_flush_every_n_ = 50;
        }

        if (left_motor_ids_param_.empty()) {
            RCLCPP_WARN(get_logger(), "left_motor_ids is empty.");
        }

        if (right_motor_ids_param_.empty()) {
            RCLCPP_WARN(get_logger(), "right_motor_ids is empty.");
        }

        last_left_rx_time_ = this->now();
        last_right_rx_time_ = this->now();
        start_time_ = this->now();

        left_received_ = false;
        right_received_ = false;

        if (enable_csv_logging_) {
            init_csv_logger();
        }

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

        merged_pub_ = this->create_publisher<hal::msg::HalArmmotor>(
            output_topic_,
            qos);

        auto period_ms = std::chrono::milliseconds(
            static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_)));

        timer_ = this->create_wall_timer(
            period_ms,
            std::bind(&ArmmotorAggregatorNode::timer_callback, this));

        RCLCPP_INFO(get_logger(), "ArmmotorAggregatorNode started.");
        RCLCPP_INFO(get_logger(), "Subscribe left : %s", left_arm_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Subscribe right: %s", right_arm_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Publish merged: %s", output_topic_.c_str());
        RCLCPP_INFO(
            get_logger(),
            "Startup grace time: %.2f sec.",
            startup_grace_sec_);

        if (enable_csv_logging_) {
            RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", csv_log_path_.c_str());
        }
    }

    ~ArmmotorAggregatorNode() override
    {
        if (csv_file_.is_open()) {
            csv_file_.flush();
            csv_file_.close();
        }
    }

private:
    void left_arm_callback(const hal::msg::HalArmmotor::SharedPtr msg)
    {
        latest_left_msg_ = *msg;
        left_received_ = true;
        last_left_rx_time_ = this->now();
    }

    void right_arm_callback(const hal::msg::HalArmmotor::SharedPtr msg)
    {
        latest_right_msg_ = *msg;
        right_received_ = true;
        last_right_rx_time_ = this->now();
    }

    std::string make_timestamp_string() const
    {
        const auto now = std::chrono::system_clock::now();
        const auto time = std::chrono::system_clock::to_time_t(now);

        std::tm tm_snapshot{};
#if defined(_WIN32)
        localtime_s(&tm_snapshot, &time);
#else
        localtime_r(&time, &tm_snapshot);
#endif

        std::ostringstream oss;
        oss << std::put_time(&tm_snapshot, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    void init_csv_logger()
    {
        try {
            std::filesystem::create_directories(csv_log_directory_);

            std::filesystem::path file_path =
                std::filesystem::path(csv_log_directory_) /
                (csv_log_file_prefix_ + "_" + make_timestamp_string() + ".csv");

            csv_log_path_ = file_path.string();
            csv_file_.open(csv_log_path_, std::ios::out | std::ios::trunc);

            if (!csv_file_.is_open()) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to open CSV log file: %s",
                    csv_log_path_.c_str());
                enable_csv_logging_ = false;
                return;
            }

            csv_file_
                << "timestamp_ns,motor_id,current,speed,position,temp,error,hal_comm_error\n";
            csv_file_.flush();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize CSV logger: %s", e.what());
            enable_csv_logging_ = false;
        }
    }

    template<typename T>
    T get_or_default(const std::vector<T>& values, size_t index, T default_value) const
    {
        if (index >= values.size()) {
            return default_value;
        }

        return values[index];
    }

    void write_csv_log(const hal::msg::HalArmmotor& msg)
    {
        if (!enable_csv_logging_ || !csv_file_.is_open()) {
            return;
        }

        for (size_t i = 0; i < 10; ++i) {
            csv_file_
                << msg.timestamp << ','
                << (i + 1) << ','
                << get_or_default<int16_t>(msg.motor_current, i, 0) << ','
                << get_or_default<int16_t>(msg.motor_speed, i, 0) << ','
                << get_or_default<int16_t>(msg.motor_position, i, 0) << ','
                << get_or_default<uint16_t>(msg.motor_temp, i, 0) << ','
                << static_cast<int>(get_or_default<uint8_t>(msg.motor_error, i, 0)) << ','
                << static_cast<int>(get_or_default<uint8_t>(msg.hal_comm_error, i, 1))
                << '\n';
        }

        ++csv_write_count_;
        if (csv_write_count_ >= csv_flush_every_n_) {
            csv_file_.flush();
            csv_write_count_ = 0;
        }
    }

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

        double since_start = (now - start_time_).seconds();
        bool allow_stale_warning = (since_start > startup_grace_sec_);

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

        write_csv_log(merged_msg);
        merged_pub_->publish(merged_msg);
    }

private:
    std::string left_arm_topic_;
    std::string right_arm_topic_;
    std::string output_topic_;

    double publish_rate_hz_{50.0};
    double stale_timeout_sec_{0.5};
    double startup_grace_sec_{30.0};

    bool enable_csv_logging_{false};
    std::string csv_log_directory_{"armmotor_logs"};
    std::string csv_log_file_prefix_{"armmotor"};
    std::string csv_log_path_;
    int csv_flush_every_n_{50};
    int csv_write_count_{0};
    std::ofstream csv_file_;

    std::vector<int64_t> left_motor_ids_param_;
    std::vector<int64_t> right_motor_ids_param_;

    rclcpp::Subscription<hal::msg::HalArmmotor>::SharedPtr left_sub_;
    rclcpp::Subscription<hal::msg::HalArmmotor>::SharedPtr right_sub_;
    rclcpp::Publisher<hal::msg::HalArmmotor>::SharedPtr merged_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    hal::msg::HalArmmotor latest_left_msg_;
    hal::msg::HalArmmotor latest_right_msg_;

    bool left_received_{false};
    bool right_received_{false};

    rclcpp::Time last_left_rx_time_;
    rclcpp::Time last_right_rx_time_;
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
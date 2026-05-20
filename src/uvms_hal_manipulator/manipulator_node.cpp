#include "uvms_hal_manipulator/manipulator_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>

namespace uvms_hal_manipulator
{

ManipulatorLifecycleNode::ManipulatorLifecycleNode(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("manipulator_driver", options),
  arm_name_("arm"),
  can_interface_("can0"),
  base_frame_("base_link"),
  ee_frame_("ee_link"),
  arm_side_("left"),
  last_rx_time_(0, 0, RCL_ROS_TIME)
{
    RCLCPP_INFO(get_logger(), "ManipulatorLifecycleNode created.");
}

auto ManipulatorLifecycleNode::on_configure(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    RCLCPP_INFO(get_logger(), "[%s] on_configure()", get_name());

    declare_and_load_parameters();

    if (!init_safety_config()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize safety config.");
        return CallbackReturn::FAILURE;
    }

    if (!init_can_driver()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize CAN driver.");
        return CallbackReturn::FAILURE;
    }

    reset_runtime_state();

    joint_cmd_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "hal/manipulator/joint_cmd",
        rclcpp::QoS(10),
        std::bind(&ManipulatorLifecycleNode::joint_cmd_callback, this, std::placeholders::_1));

    emergency_stop_srv_ = create_service<std_srvs::srv::SetBool>(
        "hal/manipulator/emergency_stop",
        std::bind(
            &ManipulatorLifecycleNode::emergency_stop_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("hal/manipulator/joint_states", rclcpp::QoS(10));

    ee_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("hal/manipulator/end_effector_pose", rclcpp::QoS(10));

    status_pub_ =
        create_publisher<std_msgs::msg::String>("hal/manipulator/status", rclcpp::QoS(10));

    fault_pub_ =
        create_publisher<std_msgs::msg::Bool>("hal/manipulator/fault", rclcpp::QoS(10));

    const auto period_ms =
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_)));

    timer_ = create_wall_timer(
        period_ms,
        std::bind(&ManipulatorLifecycleNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "[%s] configured successfully.", get_name());
    return CallbackReturn::SUCCESS;
}

auto ManipulatorLifecycleNode::on_activate(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    RCLCPP_INFO(get_logger(), "[%s] on_activate()", get_name());

    if (require_initial_pose_before_activate_ && !initial_pose_complete_) {
        RCLCPP_WARN(
            get_logger(),
            "[%s] initial pose is not complete, activation refused.",
            get_name());
        return CallbackReturn::FAILURE;
    }

    joint_state_pub_->on_activate();
    ee_pose_pub_->on_activate();
    status_pub_->on_activate();
    fault_pub_->on_activate();

    control_enabled_ = true;

    publish_status("Manipulator HAL node activated.");
    return CallbackReturn::SUCCESS;
}

auto ManipulatorLifecycleNode::on_deactivate(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    RCLCPP_INFO(get_logger(), "[%s] on_deactivate()", get_name());

    if (joint_state_pub_) joint_state_pub_->on_deactivate();
    if (ee_pose_pub_) ee_pose_pub_->on_deactivate();
    if (status_pub_) status_pub_->on_deactivate();
    if (fault_pub_) fault_pub_->on_deactivate();

    control_enabled_ = false;
    return CallbackReturn::SUCCESS;
}

auto ManipulatorLifecycleNode::on_cleanup(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    RCLCPP_INFO(get_logger(), "[%s] on_cleanup()", get_name());

    timer_.reset();
    joint_cmd_sub_.reset();
    emergency_stop_srv_.reset();

    joint_state_pub_.reset();
    ee_pose_pub_.reset();
    status_pub_.reset();
    fault_pub_.reset();

    can_driver_.close();
    reset_runtime_state();

    return CallbackReturn::SUCCESS;
}

auto ManipulatorLifecycleNode::on_shutdown(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    RCLCPP_INFO(get_logger(), "[%s] on_shutdown()", get_name());
    can_driver_.close();
    return CallbackReturn::SUCCESS;
}

auto ManipulatorLifecycleNode::on_error(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    RCLCPP_ERROR(get_logger(), "[%s] on_error()", get_name());
    can_driver_.close();
    return CallbackReturn::SUCCESS;
}

void ManipulatorLifecycleNode::declare_and_load_parameters()
{
    this->declare_parameter<std::string>("arm_name", "arm");
    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("ee_frame", "ee_link");
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<bool>("debug_mode", false);

    this->declare_parameter<std::string>("arm_side", "left");
    this->declare_parameter<bool>("require_initial_pose_before_activate", true);
    this->declare_parameter<bool>("fault_stop_on_comm_loss", true);

    this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    this->declare_parameter<std::vector<double>>("joint.position_limit_min", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("joint.position_limit_max", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("joint.max_velocity", std::vector<double>{});

    this->declare_parameter<double>("safety.max_current", 500.0);
    this->declare_parameter<double>("safety.max_temperature", 100.0);
    this->declare_parameter<double>("safety.communication_timeout_sec", 0.2);

    arm_name_ = this->get_parameter("arm_name").as_string();
    can_interface_ = this->get_parameter("can_interface").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_frame_ = this->get_parameter("ee_frame").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();

    arm_side_ = this->get_parameter("arm_side").as_string();
    require_initial_pose_before_activate_ = this->get_parameter("require_initial_pose_before_activate").as_bool();
    fault_stop_on_comm_loss_ = this->get_parameter("fault_stop_on_comm_loss").as_bool();

    joint_names_ = this->get_parameter("joint_names").as_string_array();
    joint_pos_min_ = this->get_parameter("joint.position_limit_min").as_double_array();
    joint_pos_max_ = this->get_parameter("joint.position_limit_max").as_double_array();
    joint_vel_max_ = this->get_parameter("joint.max_velocity").as_double_array();

    max_current_ = this->get_parameter("safety.max_current").as_double();
    max_temperature_ = this->get_parameter("safety.max_temperature").as_double();
    comm_timeout_sec_ = this->get_parameter("safety.communication_timeout_sec").as_double();

    RCLCPP_INFO(
        get_logger(),
        "Loaded parameters: arm_name=%s, can_interface=%s, arm_side=%s, publish_rate=%.2f",
        arm_name_.c_str(), can_interface_.c_str(), arm_side_.c_str(), publish_rate_hz_);
}

bool ManipulatorLifecycleNode::init_safety_config()
{
    JointLimitConfig joint_cfg;
    joint_cfg.position_min = joint_pos_min_;
    joint_cfg.position_max = joint_pos_max_;
    joint_cfg.max_velocity = joint_vel_max_;

    MotorSafetyConfig motor_cfg;
    motor_cfg.max_current = max_current_;
    motor_cfg.max_temperature = max_temperature_;
    motor_cfg.communication_timeout_sec = comm_timeout_sec_;

    safety_manager_.set_joint_limit_config(joint_cfg);
    safety_manager_.set_motor_safety_config(motor_cfg);
    return true;
}

bool ManipulatorLifecycleNode::init_can_driver()
{
    if (!can_driver_.open(can_interface_)) {
        return false;
    }
    can_driver_.flush();

    // 开机发送电机启动指令 + 获取初始位置指令
    RCLCPP_INFO(get_logger(), "Sending motor start upload command to all motors...");

    // 双臂所有电机 ID
    std::vector<uint32_t> motor_ids = {1,2,3,4,5,6,7,8,9,10};

    // ==============================
    // 第一步：发送 0x01 启动自动上传
    // ==============================
    for (uint32_t id : motor_ids) {
        CanFrame frame;
        frame.can_id = id;
        frame.dlc = 5;
        frame.data[0] = 0x01;  // 启动自动上传
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        can_driver_.write_frame(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // ==============================
    // 第二步：发送指令 8 → 获取电机当前位置（你要的初始姿态校准！）
    // ==============================
    RCLCPP_INFO(get_logger(), "Sending GET POSITION (cmd=8) to all motors...");

    for (uint32_t id : motor_ids) {
        CanFrame frame;
        frame.can_id = id;
        frame.dlc = 5;
        frame.data[0] = 0x08;  // <--- 指令 8：获取当前位置
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        can_driver_.write_frame(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    RCLCPP_INFO(get_logger(), "All motor init commands sent.");
    return true;
}

void ManipulatorLifecycleNode::reset_runtime_state()
{
    communication_ok_ = false;
    last_rx_time_ = this->now();
    latest_joint_position_.assign(joint_names_.size(), 0.0);
    latest_joint_velocity_.assign(joint_names_.size(), 0.0);
    latest_joint_effort_.assign(joint_names_.size(), 0.0);

    initial_pose_complete_ = false;
    control_enabled_ = false;

    communication_lost_latched_ = false;
    fault_stop_requested_ = false;

    build_expected_motor_id_list();
}

// 关键修正：双臂共用一个CAN口，只匹配本臂配置的电机ID，不按奇偶粗暴过滤
bool ManipulatorLifecycleNode::is_my_motor_id(uint32_t can_id) const
{
    for (uint32_t id : expected_motor_ids_) {
        if (id == can_id) {
            return true;
        }
    }
    return false;
}

void ManipulatorLifecycleNode::build_expected_motor_id_list()
{
    expected_motor_ids_.clear();
    motor_ready_map_.clear();

    if (arm_side_ == "left") {
        expected_motor_ids_ = {0x01, 0x03, 0x05, 0x07, 0x09, 0x13};
    } else if (arm_side_ == "right") {
        expected_motor_ids_ = {0x02, 0x04, 0x06, 0x08, 0x0A, 0x14};
    }

    for (uint32_t id : expected_motor_ids_) {
        motor_ready_map_[id] = false;
    }
}

void ManipulatorLifecycleNode::update_initial_pose_completion()
{
    bool all_ready = !expected_motor_ids_.empty();
    for (const auto& kv : motor_ready_map_) {
        if (!kv.second) {
            all_ready = false;
            break;
        }
    }
    initial_pose_complete_ = all_ready;
}

void ManipulatorLifecycleNode::handle_communication_loss()
{
    if (communication_lost_latched_) return;

    communication_lost_latched_ = true;
    fault_stop_requested_ = true;
    control_enabled_ = false;
    communication_ok_ = false;

    RCLCPP_ERROR(get_logger(), "[%s] Communication timeout. Fault stop.", get_name());
    publish_status("Communication timeout. Fault stop.");

    if (fault_pub_ && fault_pub_->is_activated()) {
        std_msgs::msg::Bool msg;
        msg.data = true;
        fault_pub_->publish(msg);
    }
}

void ManipulatorLifecycleNode::perform_fault_stop()
{
    if (!fault_stop_requested_) return;

    RCLCPP_ERROR(get_logger(), "[%s] Perform fault stop.", get_name());

    control_enabled_ = false;
    if (joint_state_pub_) joint_state_pub_->on_deactivate();
    if (ee_pose_pub_) ee_pose_pub_->on_deactivate();
    if (status_pub_) status_pub_->on_deactivate();
    if (fault_pub_) fault_pub_->on_deactivate();

    if (timer_) timer_->cancel();
    can_driver_.close();
    fault_stop_requested_ = false;
}

void ManipulatorLifecycleNode::joint_cmd_callback(
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    if (communication_lost_latched_ || fault_stop_requested_) return;
    if (!control_enabled_) {
        publish_status("Control not enabled.");
        return;
    }

    std::vector<double> positions(msg->positions.begin(), msg->positions.end());
    std::vector<double> velocities(msg->velocities.begin(), msg->velocities.end());

    auto check = safety_manager_.validate_joint_command(positions, velocities);
    if (!check.ok) {
        RCLCPP_WARN(get_logger(), "Reject: %s", check.reason.c_str());
        publish_status("Reject: " + check.reason);
        return;
    }

    latest_joint_position_ = positions;
    latest_joint_velocity_ = safety_manager_.clamp_velocity(velocities);
    publish_status("Joint command accepted.");
}

void ManipulatorLifecycleNode::emergency_stop_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    safety_manager_.set_estop(req->data);
    res->success = true;
    res->message = req->data ? "EStop ON" : "EStop OFF";
    publish_status(res->message);
}

void ManipulatorLifecycleNode::timer_callback()
{
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    if (fault_stop_requested_) {
        perform_fault_stop();
        return;
    }

    CanFrame frame;
    bool received = false;
    for (int i = 0; i < 20; ++i) {
        if (!can_driver_.read_frame(frame)) break;
        received = true;
        process_rx_frame(frame);
    }

    if (received) {
        last_rx_time_ = this->now();
        communication_ok_ = true;
    }

    if (!debug_mode_) {
        double elapsed = (this->now() - last_rx_time_).seconds();
        auto comm_check = safety_manager_.check_communication_timeout(elapsed);
        if (!comm_check.ok) {
            communication_ok_ = false;
            if (fault_stop_on_comm_loss_) {
                handle_communication_loss();
                return;
            }
            publish_status(comm_check.reason);
            return;
        }
    } else {
        communication_ok_ = true;
    }

    auto currents = int16_array_to_double_vector_10(latest_arm_motor_state_.current);
    auto temps = uint16_array_to_double_vector_10(latest_arm_motor_state_.temperature);
    auto overload = safety_manager_.check_motor_overload(currents, temps);
    if (!overload.ok) publish_status(overload.reason);

    publish_joint_states();
    publish_end_effector_pose();
}

bool ManipulatorLifecycleNode::process_rx_frame(const CanFrame& frame)
{
    // 共用CAN口：只处理本臂电机ID
    if (!is_my_motor_id(frame.can_id)) return false;

    auto msg = protocol_parser_.process_can_frame(frame);
    if (!msg.has_value()) return false;

    if (msg->type == CompleteMessageType::ARMCABIN_MOTOR) {
        protocol_parser_.get_armcabin_motor_state(latest_armcabin_motor_state_);
    } else if (msg->type == CompleteMessageType::ARM_MOTOR) {
        protocol_parser_.get_arm_motor_state(latest_arm_motor_state_);

        size_t n = std::min({joint_names_.size(), latest_joint_position_.size(), latest_arm_motor_state_.position.size()});
        for (size_t i = 0; i < n; ++i) {
            latest_joint_position_[i] = static_cast<double>(latest_arm_motor_state_.position[i]);
            latest_joint_velocity_[i] = static_cast<double>(latest_arm_motor_state_.speed[i]);
            latest_joint_effort_[i] = static_cast<double>(latest_arm_motor_state_.current[i]);
        }

        auto it = motor_ready_map_.find(frame.can_id);
        if (it != motor_ready_map_.end()) it->second = true;
        update_initial_pose_completion();
    } else if (msg->type == CompleteMessageType::ARM_CONTROLLER) {
        protocol_parser_.get_arm_controller_state(latest_arm_controller_state_);
    }

    return true;
}

void ManipulatorLifecycleNode::publish_joint_states()
{
    if (!joint_state_pub_ || !joint_state_pub_->is_activated()) return;
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position = latest_joint_position_;
    msg.velocity = latest_joint_velocity_;
    msg.effort = latest_joint_effort_;
    joint_state_pub_->publish(msg);
}

void ManipulatorLifecycleNode::publish_end_effector_pose()
{
    if (!ee_pose_pub_ || !ee_pose_pub_->is_activated()) return;
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = base_frame_;
    ee_pose_pub_->publish(msg);
}

void ManipulatorLifecycleNode::publish_status(const std::string& text)
{
    if (!status_pub_ || !status_pub_->is_activated()) return;
    std_msgs::msg::String msg;
    msg.data = "[" + arm_name_ + "] " + text;
    status_pub_->publish(msg);
}

std::vector<double> ManipulatorLifecycleNode::int16_array_to_double_vector_2(const std::array<int16_t, 2>& arr) const {
    return {static_cast<double>(arr[0]), static_cast<double>(arr[1])};
}
std::vector<double> ManipulatorLifecycleNode::int16_array_to_double_vector_10(const std::array<int16_t, 10>& arr) const {
    std::vector<double> v(10);
    for (int i=0;i<10;++i) v[i] = arr[i];
    return v;
}
std::vector<double> ManipulatorLifecycleNode::uint16_array_to_double_vector_10(const std::array<uint16_t, 10>& arr) const {
    std::vector<double> v(10);
    for (int i=0;i<10;++i) v[i] = arr[i];
    return v;
}

}  // namespace uvms_hal_manipulator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uvms_hal_manipulator::ManipulatorLifecycleNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
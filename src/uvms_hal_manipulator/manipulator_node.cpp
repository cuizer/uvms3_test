#include "uvms_hal_manipulator/manipulator_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>
#include <thread>
#include <map>
#include <iostream>

namespace uvms_hal_manipulator
{

ManipulatorLifecycleNode::ManipulatorLifecycleNode(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("manipulator_driver", options),
  arm_name_("arm"),
  can_interface_("can0"),
  base_frame_("base_link"),
  ee_frame_("ee_link"),
  arm_side_("left"),
  last_rx_time_(this->now()),
  communication_ok_(false),
  initial_pose_complete_(false),
  control_enabled_(false),
  communication_lost_latched_(false),
  fault_stop_requested_(false),
  data_upload_enabled_(true)
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
        std::bind(&ManipulatorLifecycleNode::emergency_stop_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "hal/manipulator/joint_states", rclcpp::QoS(10));

    ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "hal/manipulator/end_effector_pose", rclcpp::QoS(10));

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "hal/manipulator/status", rclcpp::QoS(10));

    fault_pub_ = create_publisher<std_msgs::msg::Bool>(
        "hal/manipulator/fault", rclcpp::QoS(10));

    armmotor_state_pub_ = create_publisher<hal::msg::HalArmmotor>("/hal/armmotor", rclcpp::QoS(10));

    armmotor_cmd_srv_ = create_service<hal::srv::HalArmmotorSrv>(
        "/hal/armmotor_cmd",
        std::bind(&ManipulatorLifecycleNode::armmotor_cmd_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_)));
    timer_ = create_wall_timer(period_ms, std::bind(&ManipulatorLifecycleNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "[%s] configured successfully.", get_name());
    return CallbackReturn::SUCCESS;
}

auto ManipulatorLifecycleNode::on_activate(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    if (!initial_pose_complete_) {
        RCLCPP_ERROR(get_logger(), "Cannot activate: initial pose not ready!");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "[%s] on_activate()", get_name());

    joint_state_pub_->on_activate();
    ee_pose_pub_->on_activate();
    status_pub_->on_activate();
    fault_pub_->on_activate();

    if (armmotor_state_pub_) {
        armmotor_state_pub_->on_activate();
    }

    control_enabled_ = true;
    fault_stop_requested_ = false;
    publish_status("Manipulator activated.");
    return CallbackReturn::SUCCESS;
}

auto ManipulatorLifecycleNode::on_deactivate(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    RCLCPP_INFO(get_logger(), "[%s] on_deactivate()", get_name());

    if (joint_state_pub_) joint_state_pub_->on_deactivate();
    if (ee_pose_pub_) ee_pose_pub_->on_deactivate();
    if (status_pub_) status_pub_->on_deactivate();
    if (fault_pub_) fault_pub_->on_deactivate();

    if (armmotor_state_pub_) {
        armmotor_state_pub_->on_deactivate();
    }

    control_enabled_ = false;
    return CallbackReturn::SUCCESS;
}

auto ManipulatorLifecycleNode::on_cleanup(const rclcpp_lifecycle::State&) -> CallbackReturn
{
    RCLCPP_INFO(get_logger(), "[%s] on_cleanup()", get_name());

    timer_.reset();
    joint_cmd_sub_.reset();
    emergency_stop_srv_.reset();
    armmotor_cmd_srv_.reset();

    joint_state_pub_.reset();
    ee_pose_pub_.reset();
    status_pub_.reset();
    fault_pub_.reset();
    armmotor_state_pub_.reset();

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
    this->declare_parameter<bool>("debug_mode", true);

    this->declare_parameter<std::string>("arm_side", "left");
    this->declare_parameter<bool>("require_initial_pose_before_activate", true);
    this->declare_parameter<bool>("fault_stop_on_comm_loss", true);

    this->declare_parameter<std::vector<std::string>>("joint_names", {"joint1"});
    this->declare_parameter<std::vector<double>>("joint.position_limit_min", {-3.14159});
    this->declare_parameter<std::vector<double>>("joint.position_limit_max", {3.14159});
    this->declare_parameter<std::vector<double>>("joint.max_velocity", {1.0});

    this->declare_parameter<double>("safety.max_current", 500.0);
    this->declare_parameter<double>("safety.max_temperature", 80.0);
    this->declare_parameter<double>("safety.communication_timeout_sec", 0.5);

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
}

bool ManipulatorLifecycleNode::init_safety_config()
{
    return true;
}

bool ManipulatorLifecycleNode::init_can_driver()
{
    if (!can_driver_.open(can_interface_)) {
        return false;
    }
    can_driver_.flush();
    RCLCPP_INFO(get_logger(), "CAN interface opened: %s", can_interface_.c_str());
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

void ManipulatorLifecycleNode::build_expected_motor_id_list()
{
    expected_motor_ids_.clear();
    motor_ready_map_.clear();

    if (arm_side_ == "left") {
        expected_motor_ids_ = {1};
    } else {
        expected_motor_ids_ = {2};
    }

    for (uint32_t id : expected_motor_ids_) {
        motor_ready_map_[id] = false;
    }
}

bool ManipulatorLifecycleNode::is_my_motor_id(uint32_t can_id) const
{
    for (uint32_t id : expected_motor_ids_) {
        if (id == can_id) return true;
    }
    return false;
}

void ManipulatorLifecycleNode::update_initial_pose_completion()
{
    bool all_ready = true;
    for (auto& p : motor_ready_map_) {
        if (!p.second) all_ready = false;
    }
    initial_pose_complete_ = all_ready;

    if (initial_pose_complete_) {
        RCLCPP_INFO(get_logger(), "✅ All motor initial positions received. Ready to activate.");
    }
}

void ManipulatorLifecycleNode::joint_cmd_callback(
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    if (!control_enabled_ || fault_stop_requested_) return;

    if (msg->positions.empty()) return;

    double target = msg->positions[0];
    if (target < joint_pos_min_[0] || target > joint_pos_max_[0]) {
        RCLCPP_WARN(get_logger(), "❌ Command REJECTED: position out of limit!");
        return;
    }

    latest_joint_position_[0] = target;
}

void ManipulatorLifecycleNode::emergency_stop_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    if (req->data) {
        RCLCPP_ERROR(get_logger(), "EMERGENCY STOP TRIGGERED");
        fault_stop_requested_ = true;
        control_enabled_ = false;
    }
    res->success = true;
}

void ManipulatorLifecycleNode::armmotor_cmd_callback(
    const std::shared_ptr<hal::srv::HalArmmotorSrv::Request> req,
    std::shared_ptr<hal::srv::HalArmmotorSrv::Response> res)
{
    res->success = true;
    res->message = "ok";
}

void ManipulatorLifecycleNode::timer_callback()
{
    auto now = this->now();
    bool is_active = (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // 1. 发送查询位置指令给所有电机
    for (uint32_t id : expected_motor_ids_) {
        CanFrame req;
        req.can_id = id;
        req.dlc = 5;
        req.data[0] = 0x08;
        req.data[1] = 0x00;
        req.data[2] = 0x00;
        req.data[3] = 0x00;
        req.data[4] = 0x00;
        can_driver_.write_frame(req);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    // 2. 接收所有帧
    bool received_any = false;
    CanFrame frame;
    for (int i = 0; i < 50; i++) {
        if (can_driver_.read_frame(frame)) {
            received_any = true;
            process_rx_frame(frame);
        }
    }

    if (received_any) {
        last_rx_time_ = now;
        communication_ok_ = true;
        communication_lost_latched_ = false;
    } else {
        communication_ok_ = false;
    }

    // 3. 安全监控：通信超时
    if (is_active && fault_stop_on_comm_loss_) {
        double dt = (now - last_rx_time_).seconds();
        if (dt > comm_timeout_sec_) {
            RCLCPP_ERROR(get_logger(), "❌ Communication lost -> fault stop");
            fault_stop_requested_ = true;
            control_enabled_ = false;
        }
    }

    // 4. 故障 -> 自动退出 activate
    if (fault_stop_requested_ && is_active) {
        this->deactivate();
        auto msg_fault = std_msgs::msg::Bool();
        msg_fault.data = true;
        fault_pub_->publish(msg_fault);
        return;
    }

    // 5. 激活状态：下发位置指令
    if (is_active && control_enabled_ && !fault_stop_requested_)
    {
        double target_rad = latest_joint_position_[0];
        double target_deg = target_rad * 180.0 / M_PI;
        const double ratio = 101.0;

        int32_t val = (target_deg / 360.0) * ratio * 65536.0;
        uint32_t u_val = static_cast<uint32_t>(val);

        CanFrame tx;
        tx.can_id = 1;
        tx.dlc = 5;
        tx.data[0] = 0x1E;
        tx.data[1] = (u_val >> 0) & 0xFF;
        tx.data[2] = (u_val >> 8) & 0xFF;
        tx.data[3] = (u_val >> 16) & 0xFF;
        tx.data[4] = (u_val >> 24) & 0xFF;

        can_driver_.write_frame(tx);

        RCLCPP_INFO(get_logger(), "✅ Send: %.2f deg -> 1E %02X %02X %02X %02X",
            target_deg, tx.data[1], tx.data[2], tx.data[3], tx.data[4]);
    }

    // 6. 状态上传
    publish_joint_states();
}

bool ManipulatorLifecycleNode::process_rx_frame(const CanFrame& frame)
{
    if (!is_my_motor_id(frame.can_id)) return false;

    if (frame.dlc == 5 && frame.data[0] == 0x08)
    {
        uint32_t raw = 0;
        raw |= (uint8_t)frame.data[1] << 0;
        raw |= (uint8_t)frame.data[2] << 8;
        raw |= (uint8_t)frame.data[3] << 16;
        raw |= (uint8_t)frame.data[4] << 24;

        const double ratio = 101.0;
        double deg = (raw / 65536.0 / ratio) * 360.0;
        double rad = deg * M_PI / 180.0;

        latest_joint_position_[0] = rad;
        motor_ready_map_[frame.can_id] = true;
        update_initial_pose_completion();
        return true;
    }
    return false;
}

void ManipulatorLifecycleNode::publish_joint_states()
{
    if (!joint_state_pub_) return;
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position = latest_joint_position_;
    msg.velocity = latest_joint_velocity_;
    joint_state_pub_->publish(msg);
}

void ManipulatorLifecycleNode::publish_end_effector_pose() {}
void ManipulatorLifecycleNode::publish_status(const std::string& text) {}

}

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
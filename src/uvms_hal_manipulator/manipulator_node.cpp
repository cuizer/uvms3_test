#include "uvms_hal_manipulator/manipulator_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>
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
  last_rx_time_(this->now()),
  communication_ok_(false),
  initial_pose_complete_(false),
  control_enabled_(false),
  communication_lost_latched_(false),
  fault_stop_requested_(false)
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

    // 上位机状态发布
    armmotor_state_pub_ = create_publisher<hal::msg::HalArmmotor>("/hal/armmotor", rclcpp::QoS(10));

    // 上位机指令服务
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
    RCLCPP_INFO(get_logger(), "[%s] on_activate()", get_name());

    joint_state_pub_->on_activate();
    ee_pose_pub_->on_activate();
    status_pub_->on_activate();
    fault_pub_->on_activate();

    if (armmotor_state_pub_) {
        armmotor_state_pub_->on_activate();
    }

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
    this->declare_parameter<bool>("require_initial_pose_before_activate", false);
    this->declare_parameter<bool>("fault_stop_on_comm_loss", false);

    this->declare_parameter<std::vector<std::string>>("joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    this->declare_parameter<std::vector<double>>("joint.position_limit_min", {-3.14, -3.14, -3.14, -3.14, -3.14, -3.14});
    this->declare_parameter<std::vector<double>>("joint.position_limit_max", { 3.14,  3.14,  3.14,  3.14,  3.14,  3.14});
    this->declare_parameter<std::vector<double>>("joint.max_velocity", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    this->declare_parameter<double>("safety.max_current", 500.0);
    this->declare_parameter<double>("safety.max_temperature", 100.0);
    this->declare_parameter<double>("safety.communication_timeout_sec", 1.0);

    arm_name_ = this->get_parameter("arm_name").as_string();
    can_interface_ = this->get_parameter("can_interface").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_frame_ = this->get_parameter("ee_frame").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();

    arm_side_ = this->get_parameter("arm_side").as_string();
    require_initial_pose_before_activate_ = false;
    fault_stop_on_comm_loss_ = false;

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

    RCLCPP_INFO(get_logger(), "Sending motor start upload command to all motors...");
    std::vector<uint32_t> motor_ids = {1,2,3,4,5,6,7,8,9,10};

    for (uint32_t id : motor_ids) {
        CanFrame frame;
        frame.can_id = id;
        frame.dlc = 5;
        frame.data[0] = 0x01;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        can_driver_.write_frame(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    RCLCPP_INFO(get_logger(), "Sending GET POSITION (cmd=8) to all motors...");

    for (uint32_t id : motor_ids) {
        CanFrame frame;
        frame.can_id = id;
        frame.dlc = 5;
        frame.data[0] = 0x08;
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

    data_upload_enabled_ = true;

    build_expected_motor_id_list();
}

bool ManipulatorLifecycleNode::is_my_motor_id(uint32_t can_id) const
{
    for (uint32_t id : expected_motor_ids_) {
        if (id == can_id) return true;
    }
    return false;
}

void ManipulatorLifecycleNode::build_expected_motor_id_list()
{
    expected_motor_ids_.clear();
    motor_ready_map_.clear();

    if (arm_side_ == "left") {
        expected_motor_ids_ = {0x01, 0x03, 0x05, 0x07, 0x09, 0x13};
    } else {
        expected_motor_ids_ = {0x02, 0x04, 0x06, 0x08, 0x0A, 0x14};
    }

    for (uint32_t id : expected_motor_ids_) {
        motor_ready_map_[id] = false;
    }
}

void ManipulatorLifecycleNode::update_initial_pose_completion()
{
    bool all_ready = true;
    for (auto& p : motor_ready_map_) {
        if (!p.second) all_ready = false;
    }
    initial_pose_complete_ = all_ready;
}

void ManipulatorLifecycleNode::handle_communication_loss() {}
void ManipulatorLifecycleNode::perform_fault_stop() {}

void ManipulatorLifecycleNode::joint_cmd_callback(
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    if (!control_enabled_) return;

    std::vector<double> positions(msg->positions.begin(), msg->positions.end());
    std::vector<double> velocities(msg->velocities.begin(), msg->velocities.end());
    latest_joint_position_ = positions;
    latest_joint_velocity_ = velocities;
}

void ManipulatorLifecycleNode::emergency_stop_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    res->success = true;
    res->message = "OK";
}

// 上位机指令回调
void ManipulatorLifecycleNode::armmotor_cmd_callback(
    const std::shared_ptr<hal::srv::HalArmmotorSrv::Request> req,
    std::shared_ptr<hal::srv::HalArmmotorSrv::Response> res)
{
    uint8_t cmd = req->cmd;
    res->success = true;

    switch (cmd) {
        case 0x01:
            RCLCPP_INFO(get_logger(), "[上位机] 机械臂舱开启");
            res->message = "cabin open";
            break;
        case 0x02:
            RCLCPP_INFO(get_logger(), "[上位机] 机械臂舱关闭");
            res->message = "cabin close";
            break;
        case 0x03:
            RCLCPP_INFO(get_logger(), "[上位机] 机械臂伸出");
            res->message = "arm extend";
            break;
        case 0x04:
            RCLCPP_INFO(get_logger(), "[上位机] 机械臂回收");
            res->message = "arm retract";
            break;
        case 0x05:
            data_upload_enabled_ = true;
            RCLCPP_INFO(get_logger(), "[上位机] 数据上传开启");
            res->message = "data upload on";
            break;
        case 0x06:
            data_upload_enabled_ = false;
            RCLCPP_INFO(get_logger(), "[上位机] 数据上传关闭");
            res->message = "data upload off";
            break;
        default:
            res->success = false;
            res->message = "unknown cmd";
            break;
    }
}

void ManipulatorLifecycleNode::timer_callback()
{
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;

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

    CanFrame frame;
    bool received = false;
    for (int i = 0; i < 30; i++) {
        if (can_driver_.read_frame(frame)) {
            received = true;
            process_rx_frame(frame);
        }
    }

    if (received) {
        last_rx_time_ = this->now();
        communication_ok_ = true;
        initial_pose_complete_ = true;
    }

    if (control_enabled_ && communication_ok_ && !latest_joint_position_.empty())
    {
        double target_rad = latest_joint_position_[0];
        double target_deg = target_rad * 180.0 / M_PI;
        const double reduction_ratio = 101.0;
        int32_t send_val = (target_deg / 360.0) * reduction_ratio * 65536.0;

        CanFrame tx_frame;
        tx_frame.can_id = 1;
        tx_frame.dlc = 5;
        tx_frame.data[0] = 0x1E;
        tx_frame.data[1] = send_val & 0xFF;
        tx_frame.data[2] = (send_val >> 8) & 0xFF;
        tx_frame.data[3] = (send_val >> 16) & 0xFF;
        tx_frame.data[4] = (send_val >> 24) & 0xFF;

        can_driver_.write_frame(tx_frame);
    }

    // 上位机状态上传
    if (data_upload_enabled_ && armmotor_state_pub_) {
        auto msg = hal::msg::HalArmmotor();
        msg.timestamp = this->now().nanoseconds() / 1000000;

        for (int i = 0; i < 10; i++) {
            msg.motor_current.push_back(0);
            msg.motor_speed.push_back(0);
            msg.motor_position.push_back(0);
            msg.motor_temp.push_back(0);
            msg.motor_error.push_back(0);
        }

        armmotor_state_pub_->publish(msg);
    }

    publish_joint_states();
    publish_end_effector_pose();
}

bool ManipulatorLifecycleNode::process_rx_frame(const CanFrame& frame)
{
    if (!is_my_motor_id(frame.can_id)) return false;

    if (frame.dlc == 5 && frame.data[0] == 0x08)
    {
        int32_t raw = 0;
        raw |= (uint8_t)frame.data[1];
        raw |= (uint8_t)frame.data[2] << 8;
        raw |= (uint8_t)frame.data[3] << 16;
        raw |= (uint8_t)frame.data[4] << 24;

        const double reduction = 101.0;
        double angle_deg = (raw / 65536.0 / reduction) * 360.0;

        int idx = -1;
        for (int i = 0; i < (int)expected_motor_ids_.size(); ++i) {
            if (expected_motor_ids_[i] == frame.can_id) {
                idx = i;
                break;
            }
        }

        if (idx >= 0 && idx < (int)latest_joint_position_.size()) {
            latest_joint_position_[idx] = angle_deg;
        }

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
    msg.effort = latest_joint_effort_;
    joint_state_pub_->publish(msg);
}

void ManipulatorLifecycleNode::publish_end_effector_pose()
{
    if (!ee_pose_pub_) return;
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = base_frame_;
    ee_pose_pub_->publish(msg);
}

void ManipulatorLifecycleNode::publish_status(const std::string& text)
{
    if (!status_pub_) return;
    std_msgs::msg::String msg;
    msg.data = "[" + arm_name_ + "] " + text;
    status_pub_->publish(msg);
}

std::vector<double> ManipulatorLifecycleNode::int16_array_to_double_vector_2(const std::array<int16_t, 2>& arr) const {
    return {static_cast<double>(arr[0]), static_cast<double>(arr[1])};
}

std::vector<double> ManipulatorLifecycleNode::int16_array_to_double_vector_10(const std::array<int16_t, 10>& arr) const {
    std::vector<double> v(10);
    for (int i = 0; i < 10; ++i) v[i] = arr[i];
    return v;
}

std::vector<double> ManipulatorLifecycleNode::uint16_array_to_double_vector_10(const std::array<uint16_t, 10>& arr) const {
    std::vector<double> v(10);
    for (int i = 0; i < 10; ++i) v[i] = arr[i];
    return v;
}

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
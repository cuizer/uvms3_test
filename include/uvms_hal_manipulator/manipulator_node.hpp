#ifndef UVMS_HAL_MANIPULATOR_MANIPULATOR_NODE_HPP
#define UVMS_HAL_MANIPULATOR_MANIPULATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "can_driver.hpp"
#include "protocol_parser.hpp"
#include "safety_manager.hpp"

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

// 上位机消息与服务
#include "hal/msg/hal_armmotor.hpp"
#include "hal/srv/hal_armmotor_srv.hpp"

#include <array>
#include <map>
#include <string>
#include <vector>
#include <cstdint>

namespace uvms_hal_manipulator
{

class ManipulatorLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit ManipulatorLifecycleNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ManipulatorLifecycleNode() override = default;

protected:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(
        const rclcpp_lifecycle::State& state) override;

    CallbackReturn on_activate(
        const rclcpp_lifecycle::State& state) override;

    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& state) override;

    CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& state) override;

    CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& state) override;

    CallbackReturn on_error(
        const rclcpp_lifecycle::State& state) override;

private:
    // ===================== 回调函数 =====================

    // 关节目标指令回调
    void joint_cmd_callback(
        const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);

    // 急停服务回调
    void emergency_stop_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // 上位机指令服务回调
    void armmotor_cmd_callback(
        const std::shared_ptr<hal::srv::HalArmmotorSrv::Request> request,
        std::shared_ptr<hal::srv::HalArmmotorSrv::Response> response);

    // 定时器主循环
    void timer_callback();

    // ===================== 初始化与配置 =====================

    void declare_and_load_parameters();
    bool init_safety_config();
    bool init_can_driver();
    void reset_runtime_state();

    // ===================== 电机与通信处理 =====================

    bool is_my_motor_id(uint32_t can_id) const;
    void build_expected_motor_id_list();
    void update_initial_pose_completion();

    void handle_communication_loss();
    void perform_fault_stop();

    bool process_rx_frame(const CanFrame& frame);

    // ===================== 状态发布 =====================

    void publish_joint_states();

    // 发布给上位机通信数据管理节点：/hal/armmotor
    void publish_armmotor_state();

    void publish_end_effector_pose();
    void publish_status(const std::string& text);

    // ===================== 工具函数 =====================

    std::vector<double> int16_array_to_double_vector_2(
        const std::array<int16_t, 2>& arr) const;

    std::vector<double> int16_array_to_double_vector_10(
        const std::array<int16_t, 10>& arr) const;

    std::vector<double> uint16_array_to_double_vector_10(
        const std::array<uint16_t, 10>& arr) const;

private:
    // ============================================================
    // 上位机通信相关成员
    // ============================================================

    rclcpp::Service<hal::srv::HalArmmotorSrv>::SharedPtr armmotor_cmd_srv_;

    rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalArmmotor>::SharedPtr
        armmotor_state_pub_;

    bool data_upload_enabled_{true};

    // ============================================================
    // 基础参数
    // ============================================================

    std::string arm_name_;
    std::string can_interface_;
    std::string base_frame_;
    std::string ee_frame_;

    double publish_rate_hz_{50.0};
    bool debug_mode_{false};

    // ============================================================
    // 关节参数
    // ============================================================

    std::vector<std::string> joint_names_;
    std::vector<double> joint_pos_min_;
    std::vector<double> joint_pos_max_;
    std::vector<double> joint_vel_max_;

    // ============================================================
    // 安全参数
    // ============================================================

    double max_current_{500.0};
    double max_temperature_{100.0};
    double comm_timeout_sec_{0.2};

    // ============================================================
    // ROS 通信接口
    // ============================================================

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
        joint_cmd_sub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_srv_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_pub_;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        ee_pose_pub_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
        status_pub_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
        fault_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ============================================================
    // 驱动、协议、安全管理
    // ============================================================

    ProtocolParser protocol_parser_;
    CanDriver can_driver_;
    SafetyManager safety_manager_;

    // ============================================================
    // 通信状态
    // ============================================================

    rclcpp::Time last_rx_time_;
    bool communication_ok_{false};

    bool communication_lost_latched_{false};
    bool fault_stop_requested_{false};
    bool fault_stop_on_comm_loss_{true};

    // ============================================================
    // 关节状态
    //
    // latest_joint_position_：电机反馈当前位置
    // target_joint_position_：上位机/ROS 下发的目标位置
    //
    // 注意：
    // 二者必须分开，否则 0x08 / 0x44 的反馈位置会覆盖目标位置，
    // 导致电机最终下发当前位置，看起来不运动。
    // ============================================================

    std::vector<double> latest_joint_position_;
    std::vector<double> latest_joint_velocity_;
    std::vector<double> latest_joint_effort_;

    // 新增：目标关节位置，单位 rad
    std::vector<double> target_joint_position_;

    // ============================================================
    // 原有机械臂状态结构体
    // ============================================================

    ArmCabinMotorState latest_armcabin_motor_state_{};
    ArmMotorState latest_arm_motor_state_{};
    ArmControllerState latest_arm_controller_state_{};

    // ============================================================
    // 机械臂侧别与生命周期控制状态
    // ============================================================

    std::string arm_side_;

    bool require_initial_pose_before_activate_{true};
    bool initial_pose_complete_{false};
    bool control_enabled_{false};

    // ============================================================
    // 电机 ID 与初始化状态
    // ============================================================

    std::vector<uint32_t> expected_motor_ids_;

    // 每个电机是否已经收到过初始位置
    std::map<uint32_t, bool> motor_ready_map_;

    // 每个电机初始位置日志是否已经打印过，避免刷屏
    std::map<uint32_t, bool> initial_pose_printed_map_;

    // ============================================================
    // 上位机上传数据缓存
    //
    // HalArmmotor.msg:
    // int64 timestamp
    // int16[] motor_current
    // int16[] motor_speed
    // int16[] motor_position
    // uint16[] motor_temp
    // byte[] motor_error
    // ============================================================

    std::map<uint32_t, int16_t> latest_motor_current_;
    std::map<uint32_t, int16_t> latest_motor_speed_;
    std::map<uint32_t, int16_t> latest_motor_position_;
    std::map<uint32_t, uint16_t> latest_motor_temp_;

    // 电机协议返回的错误状态是 int32_t / uint32_t，
    // 后续发布到 byte[] motor_error 时，在 cpp 中压缩成 8 位。
    std::map<uint32_t, uint32_t> latest_motor_error_;

    // ============================================================
    // 日志防刷屏标志
    // ============================================================

    // 发送 0x44 位置指令日志只打印一次
    bool send_position_log_printed_{false};

    // 所有电机初始位置接收完成日志只打印一次
    bool initial_pose_complete_log_printed_{false};
};

}  // namespace uvms_hal_manipulator

#endif  // UVMS_HAL_MANIPULATOR_MANIPULATOR_NODE_HPP
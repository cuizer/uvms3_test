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

// 上位机消息与服务（正确路径）
#include "hal/msg/hal_armmotor.hpp"
#include "hal/srv/hal_armmotor_srv.hpp"

namespace uvms_hal_manipulator
{

class ManipulatorLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit ManipulatorLifecycleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ManipulatorLifecycleNode() override = default;

protected:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

private:
    // 原有关节指令回调
    void joint_cmd_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);

    // 原有关机停止服务
    void emergency_stop_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // ===================== 上位机指令回调（已添加）=====================
    void armmotor_cmd_callback(
        const std::shared_ptr<hal::srv::HalArmmotorSrv::Request> request,
        std::shared_ptr<hal::srv::HalArmmotorSrv::Response> response);

    // 定时器
    void timer_callback();

    void declare_and_load_parameters();
    bool init_safety_config();
    bool init_can_driver();
    void reset_runtime_state();

    bool is_my_motor_id(uint32_t can_id) const;
    void build_expected_motor_id_list();
    void update_initial_pose_completion();
    void handle_communication_loss();
    void perform_fault_stop();

    bool process_rx_frame(const CanFrame& frame);
    void publish_joint_states();
    void publish_end_effector_pose();
    void publish_status(const std::string& text);

    std::vector<double> int16_array_to_double_vector_2(const std::array<int16_t, 2>& arr) const;
    std::vector<double> int16_array_to_double_vector_10(const std::array<int16_t, 10>& arr) const;
    std::vector<double> uint16_array_to_double_vector_10(const std::array<uint16_t, 10>& arr) const;

private:
    // ===================== 上位机相关成员（已添加）=====================
    rclcpp::Service<hal::srv::HalArmmotorSrv>::SharedPtr armmotor_cmd_srv_;
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalArmmotor>::SharedPtr armmotor_state_pub_;
    bool data_upload_enabled_{true};

    // 原有成员
    std::string arm_name_;
    std::string can_interface_;
    std::string base_frame_;
    std::string ee_frame_;
    double publish_rate_hz_{50.0};
    bool debug_mode_{false};

    std::vector<std::string> joint_names_;
    std::vector<double> joint_pos_min_;
    std::vector<double> joint_pos_max_;
    std::vector<double> joint_vel_max_;

    double max_current_{500.0};
    double max_temperature_{100.0};
    double comm_timeout_sec_{0.2};

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr joint_cmd_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_srv_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr fault_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    ProtocolParser protocol_parser_;
    CanDriver can_driver_;
    SafetyManager safety_manager_;

    rclcpp::Time last_rx_time_;
    bool communication_ok_{false};

    std::vector<double> latest_joint_position_;
    std::vector<double> latest_joint_velocity_;
    std::vector<double> latest_joint_effort_;

    ArmCabinMotorState latest_armcabin_motor_state_{};
    ArmMotorState latest_arm_motor_state_{};
    ArmControllerState latest_arm_controller_state_{};

    std::string arm_side_;
    bool require_initial_pose_before_activate_{true};
    bool initial_pose_complete_{false};
    bool control_enabled_{false};

    std::vector<uint32_t> expected_motor_ids_;
    std::map<uint32_t, bool> motor_ready_map_;

    bool communication_lost_latched_{false};
    bool fault_stop_requested_{false};
    bool fault_stop_on_comm_loss_{true};
};

}  // namespace uvms_hal_manipulator

#endif
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace hal
{

struct MitLimits
{
  double p_min{-12.5};
  double p_max{12.5};
  double v_min{-45.0};
  double v_max{45.0};
  double kp_min{0.0};
  double kp_max{500.0};
  double kd_min{0.0};
  double kd_max{5.0};
  double t_min{-18.0};
  double t_max{18.0};
};

struct MechanicalLimits
{
  double position_min{-1.0};
  double position_max{1.0};
  double velocity_max{1.5};
  double torque_max{5.0};
};

struct MitCommand
{
  double position{0.0};
  double velocity{0.0};
  double kp{2.0};
  double kd{1.0};
  double torque_ff{0.0};
};

struct MitFeedback
{
  uint8_t motor_id{0};
  uint8_t status_code{0};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double mos_temperature{0.0};
  double rotor_temperature{0.0};
  bool valid{false};
};

struct CanFrame
{
  uint32_t can_id{0};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};
};

struct CabinMotorState
{
  std::string name;
  uint16_t can_id{0};
  uint16_t master_id{0};

  MitLimits mit_limits;
  MechanicalLimits mechanical_limits;

  MitCommand desired_command;
  MitCommand active_command;
  MitFeedback feedback;

  bool feedback_received{false};
  bool command_received{false};
  bool command_timed_out{false};
  bool enable_command_sent{false};
  bool enabled_feedback{false};
  bool faulted{false};

  double hold_position{0.0};
  rclcpp::Time last_feedback_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time enable_command_time{0, 0, RCL_ROS_TIME};
};

class MitCanDriver
{
public:
  using ReceiveCallback = std::function<void(const CanFrame &)>;

  MitCanDriver();
  ~MitCanDriver();

  MitCanDriver(const MitCanDriver &) = delete;
  MitCanDriver & operator=(const MitCanDriver &) = delete;

  bool open(const std::string & interface_name, int retry_count, int retry_interval_ms);
  void close();
  bool isOpen() const;

  bool startReceive(const ReceiveCallback & callback);
  void stopReceive();

  bool sendMitCommand(
    uint16_t can_id,
    const MitCommand & command,
    const MitLimits & limits);

  bool sendEnable(uint16_t can_id);
  bool sendDisable(uint16_t can_id);
  bool sendClearFault(uint16_t can_id);
  bool sendSetZero(uint16_t can_id);

  MitFeedback parseFeedback(
    const CanFrame & frame,
    const MitLimits & limits) const;

private:
  bool openOnce(const std::string & interface_name);
  bool sendFrame(
    uint16_t can_id,
    const std::array<uint8_t, 8> & data,
    uint8_t dlc = 8);

  std::array<uint8_t, 8> packMitCommand(
    const MitCommand & command,
    const MitLimits & limits) const;

  static uint32_t floatToUint(
    double value,
    double min_value,
    double max_value,
    uint8_t bits);

  static double uintToFloat(
    uint32_t value,
    double min_value,
    double max_value,
    uint8_t bits);

  void receiveLoop();

  int socket_fd_{-1};
  std::atomic<bool> receiving_{false};
  std::thread receive_thread_;
  std::mutex send_mutex_;
  ReceiveCallback receive_callback_;
};

class HalCabinMotorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit HalCabinMotorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  void declareParameters();
  bool loadParameters();
  bool loadMotorParameters(
    const std::string & prefix,
    CabinMotorState & motor);
  bool validateMotorParameters(const CabinMotorState & motor) const;

  void commandCallback(
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
  void emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void clearFaultCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void setZeroCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleCanFrame(const CanFrame & frame);
  void controlCycle();
  void publishStateAndStatus();

  CabinMotorState * findMotorByFeedback(
    uint32_t arbitration_id,
    uint8_t feedback_id);

  bool allFeedbackReady() const;
  bool enableAllMotors();
  void disableAllMotors();
  void synchronizeHoldPositions();

  MitCommand makeHoldCommand(const CabinMotorState & motor) const;
  MitCommand applyMechanicalLimits(
    const CabinMotorState & motor,
    const MitCommand & command,
    bool & was_clamped) const;

  bool isFeedbackTimedOut(
    const CabinMotorState & motor,
    const rclcpp::Time & current_time) const;
  bool hasMotorFault(const CabinMotorState & motor) const;
  bool isTemperatureUnsafe(const CabinMotorState & motor) const;

  void triggerSystemFault(const std::string & reason);
  void resetRuntimeFlags();

  static std::string statusCodeToString(uint8_t status_code);
  static double clampValue(double value, double min_value, double max_value);

  std::array<CabinMotorState, 2> motors_{};
  std::unique_ptr<MitCanDriver> can_driver_;
  mutable std::mutex motor_mutex_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr command_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_zero_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string can_interface_{"can0"};
  int expected_bitrate_{1000000};
  int can_retry_count_{3};
  int can_retry_interval_ms_{1000};
  int control_period_ms_{10};
  int state_publish_period_ms_{20};

  double command_timeout_sec_{0.2};
  double feedback_timeout_sec_{0.2};
  double enable_confirmation_timeout_sec_{0.5};
  double default_kp_{2.0};
  double default_kd_{1.0};
  double hold_kp_{2.0};
  double hold_kd_{1.0};
  double max_mos_temperature_{55.0};
  double max_rotor_temperature_{55.0};

  bool require_feedback_before_activate_{true};
  bool stop_all_on_single_fault_{true};
  std::atomic<bool> node_active_{false};
  std::atomic<bool> emergency_stop_active_{false};
  std::atomic<bool> fault_latched_{false};
  std::string fault_reason_;
  rclcpp::Time last_state_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace hal
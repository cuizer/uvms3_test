#include "uvms_hal_manipulator/hal_cabinmotor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace hal
{
namespace
{
constexpr std::array<uint8_t, 8> kEnableFrame{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
constexpr std::array<uint8_t, 8> kDisableFrame{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
constexpr std::array<uint8_t, 8> kSetZeroFrame{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
constexpr std::array<uint8_t, 8> kClearFaultFrame{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

bool isFiniteCommand(const MitCommand & command)
{
  return std::isfinite(command.position) &&
         std::isfinite(command.velocity) &&
         std::isfinite(command.kp) &&
         std::isfinite(command.kd) &&
         std::isfinite(command.torque_ff);
}
}  // namespace

// ========================= MitCanDriver =========================

MitCanDriver::MitCanDriver() = default;

MitCanDriver::~MitCanDriver()
{
  stopReceive();
  close();
}

bool MitCanDriver::open(
  const std::string & interface_name,
  int retry_count,
  int retry_interval_ms)
{
  close();

  const int attempts = std::max(1, retry_count);
  for (int attempt = 1; attempt <= attempts; ++attempt) {
    if (openOnce(interface_name)) {
      return true;
    }

    if (attempt < attempts) {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(std::max(0, retry_interval_ms)));
    }
  }
  return false;
}

bool MitCanDriver::openOnce(const std::string & interface_name)
{
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd);
    return false;
  }

  int receive_own_messages = 0;
  (void)::setsockopt(
    fd,
    SOL_CAN_RAW,
    CAN_RAW_RECV_OWN_MSGS,
    &receive_own_messages,
    sizeof(receive_own_messages));

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;

  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    ::close(fd);
    return false;
  }

  socket_fd_ = fd;
  return true;
}

void MitCanDriver::close()
{
  stopReceive();

  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool MitCanDriver::isOpen() const
{
  return socket_fd_ >= 0;
}

bool MitCanDriver::startReceive(const ReceiveCallback & callback)
{
  if (!isOpen() || receiving_.load()) {
    return false;
  }

  receive_callback_ = callback;
  receiving_.store(true);
  receive_thread_ = std::thread(&MitCanDriver::receiveLoop, this);
  return true;
}

void MitCanDriver::stopReceive()
{
  receiving_.store(false);
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

void MitCanDriver::receiveLoop()
{
  while (receiving_.load()) {
    struct pollfd poll_descriptor {};
    poll_descriptor.fd = socket_fd_;
    poll_descriptor.events = POLLIN;

    const int poll_result = ::poll(&poll_descriptor, 1, 100);
    if (!receiving_.load()) {
      break;
    }
    if (poll_result <= 0 || (poll_descriptor.revents & POLLIN) == 0) {
      continue;
    }

    struct can_frame linux_frame {};
    const ssize_t bytes_read = ::read(socket_fd_, &linux_frame, sizeof(linux_frame));
    if (bytes_read != static_cast<ssize_t>(sizeof(linux_frame))) {
      continue;
    }
    if ((linux_frame.can_id & CAN_EFF_FLAG) != 0 ||
      (linux_frame.can_id & CAN_RTR_FLAG) != 0 ||
      (linux_frame.can_id & CAN_ERR_FLAG) != 0)
    {
      continue;
    }

    CanFrame frame;
    frame.can_id = linux_frame.can_id & CAN_SFF_MASK;
    frame.dlc = std::min<uint8_t>(linux_frame.can_dlc, 8);
    std::copy_n(linux_frame.data, frame.dlc, frame.data.begin());

    if (receive_callback_) {
      receive_callback_(frame);
    }
  }
}

bool MitCanDriver::sendFrame(
  uint16_t can_id,
  const std::array<uint8_t, 8> & data,
  uint8_t dlc)
{
  if (!isOpen() || dlc > 8 || can_id > CAN_SFF_MASK) {
    return false;
  }

  struct can_frame frame {};
  frame.can_id = static_cast<canid_t>(can_id & CAN_SFF_MASK);
  frame.can_dlc = dlc;
  std::copy_n(data.begin(), dlc, frame.data);

  std::lock_guard<std::mutex> lock(send_mutex_);
  const ssize_t bytes_written = ::write(socket_fd_, &frame, sizeof(frame));
  return bytes_written == static_cast<ssize_t>(sizeof(frame));
}

bool MitCanDriver::sendMitCommand(
  uint16_t can_id,
  const MitCommand & command,
  const MitLimits & limits)
{
  return sendFrame(can_id, packMitCommand(command, limits));
}

bool MitCanDriver::sendEnable(uint16_t can_id)
{
  return sendFrame(can_id, kEnableFrame);
}

bool MitCanDriver::sendDisable(uint16_t can_id)
{
  return sendFrame(can_id, kDisableFrame);
}

bool MitCanDriver::sendClearFault(uint16_t can_id)
{
  return sendFrame(can_id, kClearFaultFrame);
}

bool MitCanDriver::sendSetZero(uint16_t can_id)
{
  return sendFrame(can_id, kSetZeroFrame);
}

std::array<uint8_t, 8> MitCanDriver::packMitCommand(
  const MitCommand & command,
  const MitLimits & limits) const
{
  const uint32_t position = floatToUint(
    command.position, limits.p_min, limits.p_max, 16);
  const uint32_t velocity = floatToUint(
    command.velocity, limits.v_min, limits.v_max, 12);
  const uint32_t kp = floatToUint(
    command.kp, limits.kp_min, limits.kp_max, 12);
  const uint32_t kd = floatToUint(
    command.kd, limits.kd_min, limits.kd_max, 12);
  const uint32_t torque = floatToUint(
    command.torque_ff, limits.t_min, limits.t_max, 12);

  return {
    static_cast<uint8_t>((position >> 8) & 0xFF),
    static_cast<uint8_t>(position & 0xFF),
    static_cast<uint8_t>((velocity >> 4) & 0xFF),
    static_cast<uint8_t>(((velocity & 0x0F) << 4) | ((kp >> 8) & 0x0F)),
    static_cast<uint8_t>(kp & 0xFF),
    static_cast<uint8_t>((kd >> 4) & 0xFF),
    static_cast<uint8_t>(((kd & 0x0F) << 4) | ((torque >> 8) & 0x0F)),
    static_cast<uint8_t>(torque & 0xFF)};
}

uint32_t MitCanDriver::floatToUint(
  double value,
  double min_value,
  double max_value,
  uint8_t bits)
{
  if (!std::isfinite(value) || max_value <= min_value || bits == 0 || bits > 31) {
    return 0;
  }

  const double clamped = std::max(min_value, std::min(max_value, value));
  const double span = max_value - min_value;
  const uint32_t max_integer = (static_cast<uint32_t>(1U) << bits) - 1U;
  const double scaled = (clamped - min_value) * static_cast<double>(max_integer) / span;
  return static_cast<uint32_t>(scaled);
}

double MitCanDriver::uintToFloat(
  uint32_t value,
  double min_value,
  double max_value,
  uint8_t bits)
{
  if (max_value <= min_value || bits == 0 || bits > 31) {
    return min_value;
  }

  const uint32_t max_integer = (static_cast<uint32_t>(1U) << bits) - 1U;
  return static_cast<double>(value) * (max_value - min_value) /
         static_cast<double>(max_integer) + min_value;
}

MitFeedback MitCanDriver::parseFeedback(
  const CanFrame & frame,
  const MitLimits & limits) const
{
  MitFeedback feedback;
  if (frame.dlc < 8) {
    return feedback;
  }

  feedback.status_code = static_cast<uint8_t>((frame.data[0] >> 4) & 0x0F);
  feedback.motor_id = static_cast<uint8_t>(frame.data[0] & 0x0F);

  const uint32_t position =
    (static_cast<uint32_t>(frame.data[1]) << 8) |
    static_cast<uint32_t>(frame.data[2]);
  const uint32_t velocity =
    (static_cast<uint32_t>(frame.data[3]) << 4) |
    (static_cast<uint32_t>(frame.data[4]) >> 4);
  const uint32_t torque =
    ((static_cast<uint32_t>(frame.data[4]) & 0x0F) << 8) |
    static_cast<uint32_t>(frame.data[5]);

  feedback.position = uintToFloat(position, limits.p_min, limits.p_max, 16);
  feedback.velocity = uintToFloat(velocity, limits.v_min, limits.v_max, 12);
  feedback.torque = uintToFloat(torque, limits.t_min, limits.t_max, 12);
  feedback.mos_temperature = static_cast<double>(frame.data[6]);
  feedback.rotor_temperature = static_cast<double>(frame.data[7]);
  feedback.valid = true;
  return feedback;
}

// ========================= HalCabinMotorNode =========================

HalCabinMotorNode::HalCabinMotorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("hal_cabinmotor_node", options)
{
  declareParameters();
}

void HalCabinMotorNode::declareParameters()
{
  declare_parameter<std::string>("can_interface", "can0");
  declare_parameter<int>("expected_bitrate", 1000000);
  declare_parameter<int>("can_retry_count", 3);
  declare_parameter<int>("can_retry_interval_ms", 1000);
  declare_parameter<int>("control_period_ms", 10);
  declare_parameter<int>("state_publish_period_ms", 20);

  declare_parameter<double>("command_timeout_sec", 0.2);
  declare_parameter<double>("feedback_timeout_sec", 0.2);
  declare_parameter<double>("enable_confirmation_timeout_sec", 0.5);
  declare_parameter<double>("default_kp", 2.0);
  declare_parameter<double>("default_kd", 1.0);
  declare_parameter<double>("hold_kp", 2.0);
  declare_parameter<double>("hold_kd", 1.0);
  declare_parameter<double>("max_mos_temperature", 55.0);
  declare_parameter<double>("max_rotor_temperature", 55.0);

  declare_parameter<bool>("require_feedback_before_activate", true);
  declare_parameter<bool>("stop_all_on_single_fault", true);

  const std::array<std::string, 2> prefixes{"motor_13", "motor_15"};
  const std::array<int, 2> default_ids{19, 21};
  for (std::size_t i = 0; i < prefixes.size(); ++i) {
    const auto & prefix = prefixes[i];
    declare_parameter<std::string>(prefix + ".name", "cabin_" + prefix);
    declare_parameter<int>(prefix + ".can_id", default_ids[i]);
    declare_parameter<int>(prefix + ".master_id", 0);

    declare_parameter<double>(prefix + ".mit.p_min", -12.5);
    declare_parameter<double>(prefix + ".mit.p_max", 12.5);
    declare_parameter<double>(prefix + ".mit.v_min", -45.0);
    declare_parameter<double>(prefix + ".mit.v_max", 45.0);
    declare_parameter<double>(prefix + ".mit.kp_min", 0.0);
    declare_parameter<double>(prefix + ".mit.kp_max", 500.0);
    declare_parameter<double>(prefix + ".mit.kd_min", 0.0);
    declare_parameter<double>(prefix + ".mit.kd_max", 5.0);
    declare_parameter<double>(prefix + ".mit.t_min", -18.0);
    declare_parameter<double>(prefix + ".mit.t_max", 18.0);

    declare_parameter<double>(prefix + ".mechanical.position_min", -1.0);
    declare_parameter<double>(prefix + ".mechanical.position_max", 1.0);
    declare_parameter<double>(prefix + ".mechanical.velocity_max", 1.5);
    declare_parameter<double>(prefix + ".mechanical.torque_max", 5.0);
  }
}

bool HalCabinMotorNode::loadMotorParameters(
  const std::string & prefix,
  CabinMotorState & motor)
{
  const int can_id = get_parameter(prefix + ".can_id").as_int();
  const int master_id = get_parameter(prefix + ".master_id").as_int();
  if (can_id < 0 || can_id > CAN_SFF_MASK || master_id < 0 || master_id > CAN_SFF_MASK) {
    RCLCPP_ERROR(get_logger(), "%s CAN ID参数超出标准帧范围", prefix.c_str());
    return false;
  }

  motor.name = get_parameter(prefix + ".name").as_string();
  motor.can_id = static_cast<uint16_t>(can_id);
  motor.master_id = static_cast<uint16_t>(master_id);

  motor.mit_limits.p_min = get_parameter(prefix + ".mit.p_min").as_double();
  motor.mit_limits.p_max = get_parameter(prefix + ".mit.p_max").as_double();
  motor.mit_limits.v_min = get_parameter(prefix + ".mit.v_min").as_double();
  motor.mit_limits.v_max = get_parameter(prefix + ".mit.v_max").as_double();
  motor.mit_limits.kp_min = get_parameter(prefix + ".mit.kp_min").as_double();
  motor.mit_limits.kp_max = get_parameter(prefix + ".mit.kp_max").as_double();
  motor.mit_limits.kd_min = get_parameter(prefix + ".mit.kd_min").as_double();
  motor.mit_limits.kd_max = get_parameter(prefix + ".mit.kd_max").as_double();
  motor.mit_limits.t_min = get_parameter(prefix + ".mit.t_min").as_double();
  motor.mit_limits.t_max = get_parameter(prefix + ".mit.t_max").as_double();

  motor.mechanical_limits.position_min =
    get_parameter(prefix + ".mechanical.position_min").as_double();
  motor.mechanical_limits.position_max =
    get_parameter(prefix + ".mechanical.position_max").as_double();
  motor.mechanical_limits.velocity_max =
    get_parameter(prefix + ".mechanical.velocity_max").as_double();
  motor.mechanical_limits.torque_max =
    get_parameter(prefix + ".mechanical.torque_max").as_double();

  return validateMotorParameters(motor);
}

bool HalCabinMotorNode::validateMotorParameters(const CabinMotorState & motor) const
{
  const bool mit_valid =
    motor.mit_limits.p_min < motor.mit_limits.p_max &&
    motor.mit_limits.v_min < motor.mit_limits.v_max &&
    motor.mit_limits.kp_min < motor.mit_limits.kp_max &&
    motor.mit_limits.kd_min < motor.mit_limits.kd_max &&
    motor.mit_limits.t_min < motor.mit_limits.t_max;

  const bool mechanical_valid =
    motor.mechanical_limits.position_min < motor.mechanical_limits.position_max &&
    motor.mechanical_limits.velocity_max > 0.0 &&
    motor.mechanical_limits.torque_max > 0.0;

  if (!mit_valid || !mechanical_valid) {
    RCLCPP_ERROR(get_logger(), "%s 参数范围无效", motor.name.c_str());
    return false;
  }
  if (motor.mechanical_limits.position_min < motor.mit_limits.p_min ||
    motor.mechanical_limits.position_max > motor.mit_limits.p_max ||
    motor.mechanical_limits.velocity_max >
    std::max(std::fabs(motor.mit_limits.v_min), std::fabs(motor.mit_limits.v_max)) ||
    motor.mechanical_limits.torque_max >
    std::max(std::fabs(motor.mit_limits.t_min), std::fabs(motor.mit_limits.t_max)))
  {
    RCLCPP_ERROR(get_logger(), "%s 机械限位超出MIT映射范围", motor.name.c_str());
    return false;
  }
  return true;
}

bool HalCabinMotorNode::loadParameters()
{
  can_interface_ = get_parameter("can_interface").as_string();
  expected_bitrate_ = get_parameter("expected_bitrate").as_int();
  can_retry_count_ = get_parameter("can_retry_count").as_int();
  can_retry_interval_ms_ = get_parameter("can_retry_interval_ms").as_int();
  control_period_ms_ = get_parameter("control_period_ms").as_int();
  state_publish_period_ms_ = get_parameter("state_publish_period_ms").as_int();

  command_timeout_sec_ = get_parameter("command_timeout_sec").as_double();
  feedback_timeout_sec_ = get_parameter("feedback_timeout_sec").as_double();
  enable_confirmation_timeout_sec_ =
    get_parameter("enable_confirmation_timeout_sec").as_double();
  default_kp_ = get_parameter("default_kp").as_double();
  default_kd_ = get_parameter("default_kd").as_double();
  hold_kp_ = get_parameter("hold_kp").as_double();
  hold_kd_ = get_parameter("hold_kd").as_double();
  max_mos_temperature_ = get_parameter("max_mos_temperature").as_double();
  max_rotor_temperature_ = get_parameter("max_rotor_temperature").as_double();

  require_feedback_before_activate_ =
    get_parameter("require_feedback_before_activate").as_bool();
  stop_all_on_single_fault_ = get_parameter("stop_all_on_single_fault").as_bool();

  if (can_interface_.empty() || expected_bitrate_ != 1000000 ||
    can_retry_count_ < 1 || can_retry_interval_ms_ < 0 ||
    control_period_ms_ < 1 || state_publish_period_ms_ < 1 ||
    command_timeout_sec_ <= 0.0 || feedback_timeout_sec_ <= 0.0 ||
    enable_confirmation_timeout_sec_ <= 0.0 ||
    max_mos_temperature_ <= 0.0 || max_rotor_temperature_ <= 0.0)
  {
    RCLCPP_ERROR(get_logger(), "舱段节点公共参数无效");
    return false;
  }

  if (!loadMotorParameters("motor_13", motors_[0]) ||
    !loadMotorParameters("motor_15", motors_[1]))
  {
    return false;
  }

  if (motors_[0].can_id == motors_[1].can_id) {
    RCLCPP_ERROR(get_logger(), "两台舱段电机CAN ID不能相同");
    return false;
  }
  return true;
}

HalCabinMotorNode::CallbackReturn HalCabinMotorNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!loadParameters()) {
    return CallbackReturn::FAILURE;
  }

  resetRuntimeFlags();
  can_driver_ = std::make_unique<MitCanDriver>();
  if (!can_driver_->open(can_interface_, can_retry_count_, can_retry_interval_ms_)) {
    RCLCPP_ERROR(
      get_logger(),
      "打开SocketCAN接口%s失败，已重试%d次",
      can_interface_.c_str(),
      can_retry_count_);
    can_driver_.reset();
    return CallbackReturn::FAILURE;
  }

  if (!can_driver_->startReceive(
      [this](const CanFrame & frame) {handleCanFrame(frame);} ))
  {
    RCLCPP_ERROR(get_logger(), "启动CAN接收线程失败");
    can_driver_->close();
    can_driver_.reset();
    return CallbackReturn::FAILURE;
  }

  command_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    "~/joint_cmd",
    rclcpp::QoS(10),
    std::bind(&HalCabinMotorNode::commandCallback, this, std::placeholders::_1));

  state_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
  fault_pub_ = create_publisher<std_msgs::msg::String>("~/fault", 10);

  emergency_stop_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/emergency_stop",
    std::bind(
      &HalCabinMotorNode::emergencyStopCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  clear_fault_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/clear_fault",
    std::bind(
      &HalCabinMotorNode::clearFaultCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  set_zero_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/set_zero",
    std::bind(
      &HalCabinMotorNode::setZeroCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "舱段双MIT电机节点配置完成：CAN=%s，期望波特率=%d，ID=0x%02X/0x%02X",
    can_interface_.c_str(),
    expected_bitrate_,
    motors_[0].can_id,
    motors_[1].can_id);
  RCLCPP_WARN(
    get_logger(),
    "SocketCAN波特率需在系统层预先配置为1Mbps，本节点只负责打开和使用接口");
  return CallbackReturn::SUCCESS;
}

HalCabinMotorNode::CallbackReturn HalCabinMotorNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!can_driver_ || !can_driver_->isOpen()) {
    RCLCPP_ERROR(get_logger(), "CAN未打开，无法激活舱段节点");
    return CallbackReturn::FAILURE;
  }

  if (require_feedback_before_activate_ && !allFeedbackReady()) {
    RCLCPP_ERROR(get_logger(), "0x13或0x15尚未收到有效反馈，拒绝激活");
    return CallbackReturn::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    for (const auto & motor : motors_) {
      if (hasMotorFault(motor) || isTemperatureUnsafe(motor)) {
        RCLCPP_ERROR(get_logger(), "%s存在故障或过温，拒绝激活", motor.name.c_str());
        return CallbackReturn::FAILURE;
      }
    }
  }

  emergency_stop_active_ = false;
  fault_latched_ = false;
  fault_reason_.clear();
  synchronizeHoldPositions();

  state_pub_->on_activate();
  status_pub_->on_activate();
  fault_pub_->on_activate();

  if (!enableAllMotors()) {
    disableAllMotors();
    state_pub_->on_deactivate();
    status_pub_->on_deactivate();
    fault_pub_->on_deactivate();
    RCLCPP_ERROR(get_logger(), "舱段电机使能命令发送失败");
    return CallbackReturn::FAILURE;
  }

  node_active_ = true;
  last_state_publish_time_ = now();
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(control_period_ms_),
    std::bind(&HalCabinMotorNode::controlCycle, this));

  RCLCPP_INFO(get_logger(), "舱段节点已激活，两台电机进入当前位姿保持");
  return CallbackReturn::SUCCESS;
}

HalCabinMotorNode::CallbackReturn HalCabinMotorNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  node_active_ = false;
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }

  disableAllMotors();
  if (state_pub_) {
    state_pub_->on_deactivate();
  }
  if (status_pub_) {
    status_pub_->on_deactivate();
  }
  if (fault_pub_) {
    fault_pub_->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "舱段节点已失活，两台电机已发送失能命令");
  return CallbackReturn::SUCCESS;
}

HalCabinMotorNode::CallbackReturn HalCabinMotorNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  node_active_ = false;
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  disableAllMotors();

  if (can_driver_) {
    can_driver_->stopReceive();
    can_driver_->close();
    can_driver_.reset();
  }

  command_sub_.reset();
  state_pub_.reset();
  status_pub_.reset();
  fault_pub_.reset();
  emergency_stop_srv_.reset();
  clear_fault_srv_.reset();
  set_zero_srv_.reset();
  resetRuntimeFlags();

  RCLCPP_INFO(get_logger(), "舱段节点资源已清理");
  return CallbackReturn::SUCCESS;
}

HalCabinMotorNode::CallbackReturn HalCabinMotorNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  node_active_ = false;
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  disableAllMotors();
  if (can_driver_) {
    can_driver_->stopReceive();
    can_driver_->close();
  }
  return CallbackReturn::SUCCESS;
}

HalCabinMotorNode::CallbackReturn HalCabinMotorNode::on_error(
  const rclcpp_lifecycle::State &)
{
  node_active_ = false;
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  disableAllMotors();
  return CallbackReturn::SUCCESS;
}

void HalCabinMotorNode::resetRuntimeFlags()
{
  std::lock_guard<std::mutex> lock(motor_mutex_);
  for (auto & motor : motors_) {
    motor.desired_command = MitCommand{};
    motor.active_command = MitCommand{};
    motor.feedback = MitFeedback{};
    motor.feedback_received = false;
    motor.command_received = false;
    motor.command_timed_out = false;
    motor.enable_command_sent = false;
    motor.enabled_feedback = false;
    motor.faulted = false;
    motor.hold_position = 0.0;
    motor.last_feedback_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    motor.last_command_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    motor.enable_command_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
  emergency_stop_active_ = false;
  fault_latched_ = false;
  fault_reason_.clear();
}

void HalCabinMotorNode::commandCallback(
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
  if (!node_active_ || emergency_stop_active_ || fault_latched_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "节点非ACTIVE、急停或故障锁存状态，忽略舱段控制指令");
    return;
  }
  if (msg->positions.size() != motors_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "joint_cmd.positions必须包含2个元素，顺序为0x13、0x15");
    return;
  }
  if (!msg->velocities.empty() && msg->velocities.size() != motors_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "joint_cmd.velocities必须为空或包含2个元素");
    return;
  }
  if (!msg->effort.empty() && msg->effort.size() != motors_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "joint_cmd.effort必须为空或包含2个元素");
    return;
  }

  std::array<MitCommand, 2> requested_commands{};
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    MitCommand command;
    command.position = msg->positions[i];
    command.velocity = msg->velocities.empty() ? 0.0 : msg->velocities[i];
    command.kp = default_kp_;
    command.kd = default_kd_;
    command.torque_ff = msg->effort.empty() ? 0.0 : msg->effort[i];

    if (!isFiniteCommand(command)) {
      RCLCPP_ERROR(get_logger(), "收到非有限数值，整条舱段指令被拒绝");
      return;
    }
    requested_commands[i] = command;
  }

  const rclcpp::Time current_time = now();
  std::lock_guard<std::mutex> lock(motor_mutex_);
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    motors_[i].desired_command = requested_commands[i];
    motors_[i].last_command_time = current_time;
    motors_[i].command_received = true;
    motors_[i].command_timed_out = false;
  }
}

void HalCabinMotorNode::emergencyStopCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    emergency_stop_active_ = true;
    disableAllMotors();
    response->success = true;
    response->message = "舱段双电机急停已触发并失能";
    return;
  }

  if (fault_latched_) {
    response->success = false;
    response->message = "存在故障锁存，请先deactivate并调用clear_fault";
    return;
  }
  if (!node_active_) {
    emergency_stop_active_ = false;
    response->success = true;
    response->message = "急停标志已清除，节点当前未激活";
    return;
  }
  if (!allFeedbackReady()) {
    response->success = false;
    response->message = "反馈未就绪，不能解除急停并重新使能";
    return;
  }

  synchronizeHoldPositions();
  emergency_stop_active_ = false;
  if (!enableAllMotors()) {
    emergency_stop_active_ = true;
    response->success = false;
    response->message = "重新使能失败，急停保持";
    return;
  }

  response->success = true;
  response->message = "急停已解除，舱段电机恢复当前位置保持";
}

void HalCabinMotorNode::clearFaultCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (node_active_) {
    response->success = false;
    response->message = "请先将生命周期节点切换到inactive再清除故障";
    return;
  }
  if (!can_driver_ || !can_driver_->isOpen()) {
    response->success = false;
    response->message = "CAN接口未打开";
    return;
  }

  bool success = true;
  for (const auto & motor : motors_) {
    success = can_driver_->sendClearFault(motor.can_id) && success;
  }
  if (success) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    fault_latched_ = false;
    fault_reason_.clear();
    emergency_stop_active_ = false;
    for (auto & motor : motors_) {
      motor.faulted = false;
    }
  }

  response->success = success;
  response->message = success ? "已向0x13和0x15发送清故障命令" : "清故障命令发送失败";
}

void HalCabinMotorNode::setZeroCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (node_active_) {
    response->success = false;
    response->message = "设置零点只允许在inactive状态执行";
    return;
  }
  if (!can_driver_ || !can_driver_->isOpen()) {
    response->success = false;
    response->message = "CAN接口未打开";
    return;
  }

  bool success = true;
  for (const auto & motor : motors_) {
    success = can_driver_->sendDisable(motor.can_id) && success;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  for (const auto & motor : motors_) {
    success = can_driver_->sendSetZero(motor.can_id) && success;
  }

  response->success = success;
  response->message = success ? "已向0x13和0x15发送保存当前位置为零点命令" :
    "设置零点命令发送失败";
}

void HalCabinMotorNode::handleCanFrame(const CanFrame & frame)
{
  if (frame.dlc < 8 || !can_driver_) {
    return;
  }

  const uint8_t feedback_id = static_cast<uint8_t>(frame.data[0] & 0x0F);
  std::lock_guard<std::mutex> lock(motor_mutex_);
  CabinMotorState * motor = findMotorByFeedback(frame.can_id, feedback_id);
  if (motor == nullptr) {
    return;
  }

  const MitFeedback feedback = can_driver_->parseFeedback(frame, motor->mit_limits);
  if (!feedback.valid) {
    return;
  }

  motor->feedback = feedback;
  motor->feedback_received = true;
  motor->last_feedback_time = now();
  motor->enabled_feedback = feedback.status_code == 1;
  motor->faulted = feedback.status_code >= 8;
}

CabinMotorState * HalCabinMotorNode::findMotorByFeedback(
  uint32_t arbitration_id,
  uint8_t feedback_id)
{
  for (auto & motor : motors_) {
    const uint8_t expected_feedback_id = static_cast<uint8_t>(motor.can_id & 0x0F);
    if (motor.master_id == arbitration_id && expected_feedback_id == feedback_id) {
      return &motor;
    }
  }
  return nullptr;
}

bool HalCabinMotorNode::allFeedbackReady() const
{
  const rclcpp::Time current_time = now();
  std::lock_guard<std::mutex> lock(motor_mutex_);
  for (const auto & motor : motors_) {
    if (!motor.feedback_received || isFeedbackTimedOut(motor, current_time)) {
      return false;
    }
  }
  return true;
}

bool HalCabinMotorNode::enableAllMotors()
{
  if (!can_driver_ || !can_driver_->isOpen()) {
    return false;
  }

  const rclcpp::Time current_time = now();
  bool success = true;
  for (const auto & motor : motors_) {
    success = can_driver_->sendEnable(motor.can_id) && success;
  }

  std::lock_guard<std::mutex> lock(motor_mutex_);
  for (auto & motor : motors_) {
    motor.enable_command_sent = success;
    motor.enable_command_time = current_time;
    if (!success) {
      motor.enabled_feedback = false;
    }
  }
  return success;
}

void HalCabinMotorNode::disableAllMotors()
{
  if (can_driver_ && can_driver_->isOpen()) {
    for (const auto & motor : motors_) {
      (void)can_driver_->sendDisable(motor.can_id);
    }
  }

  std::lock_guard<std::mutex> lock(motor_mutex_);
  for (auto & motor : motors_) {
    motor.enable_command_sent = false;
    motor.enabled_feedback = false;
  }
}

void HalCabinMotorNode::synchronizeHoldPositions()
{
  std::lock_guard<std::mutex> lock(motor_mutex_);
  for (auto & motor : motors_) {
    motor.hold_position = motor.feedback_received ? motor.feedback.position : 0.0;
    motor.desired_command = makeHoldCommand(motor);
    motor.active_command = motor.desired_command;
    motor.command_received = false;
    motor.command_timed_out = false;
  }
}

MitCommand HalCabinMotorNode::makeHoldCommand(const CabinMotorState & motor) const
{
  MitCommand command;
  command.position = motor.hold_position;
  command.velocity = 0.0;
  command.kp = hold_kp_;
  command.kd = hold_kd_;
  command.torque_ff = 0.0;
  return command;
}

double HalCabinMotorNode::clampValue(
  double value,
  double min_value,
  double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

MitCommand HalCabinMotorNode::applyMechanicalLimits(
  const CabinMotorState & motor,
  const MitCommand & command,
  bool & was_clamped) const
{
  MitCommand safe = command;
  const double original_position = safe.position;
  const double original_velocity = safe.velocity;
  const double original_torque = safe.torque_ff;

  safe.position = clampValue(
    safe.position,
    motor.mechanical_limits.position_min,
    motor.mechanical_limits.position_max);
  safe.velocity = clampValue(
    safe.velocity,
    -motor.mechanical_limits.velocity_max,
    motor.mechanical_limits.velocity_max);
  safe.torque_ff = clampValue(
    safe.torque_ff,
    -motor.mechanical_limits.torque_max,
    motor.mechanical_limits.torque_max);
  safe.kp = clampValue(safe.kp, motor.mit_limits.kp_min, motor.mit_limits.kp_max);
  safe.kd = clampValue(safe.kd, motor.mit_limits.kd_min, motor.mit_limits.kd_max);

  was_clamped =
    safe.position != original_position ||
    safe.velocity != original_velocity ||
    safe.torque_ff != original_torque;
  return safe;
}

bool HalCabinMotorNode::isFeedbackTimedOut(
  const CabinMotorState & motor,
  const rclcpp::Time & current_time) const
{
  if (!motor.feedback_received) {
    return true;
  }
  return (current_time - motor.last_feedback_time).seconds() > feedback_timeout_sec_;
}

bool HalCabinMotorNode::hasMotorFault(const CabinMotorState & motor) const
{
  return motor.feedback.valid && motor.feedback.status_code >= 8;
}

bool HalCabinMotorNode::isTemperatureUnsafe(const CabinMotorState & motor) const
{
  return motor.feedback.valid &&
         (motor.feedback.mos_temperature >= max_mos_temperature_ ||
         motor.feedback.rotor_temperature >= max_rotor_temperature_);
}

void HalCabinMotorNode::controlCycle()
{
  if (!node_active_ || emergency_stop_active_ || fault_latched_ || !can_driver_) {
    return;
  }

  const rclcpp::Time current_time = now();
  std::array<MitCommand, 2> commands{};
  std::array<uint16_t, 2> can_ids{};
  std::array<MitLimits, 2> limits{};
  std::string fault_reason;

  {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      auto & motor = motors_[i];

      if (isFeedbackTimedOut(motor, current_time)) {
        fault_reason = motor.name + "反馈超时";
        break;
      }
      if (hasMotorFault(motor)) {
        fault_reason = motor.name + "反馈故障：" + statusCodeToString(motor.feedback.status_code);
        break;
      }
      if (isTemperatureUnsafe(motor)) {
        fault_reason = motor.name + "温度超过安全阈值";
        break;
      }
      if (motor.enable_command_sent && !motor.enabled_feedback &&
        (current_time - motor.enable_command_time).seconds() > enable_confirmation_timeout_sec_)
      {
        fault_reason = motor.name + "使能反馈确认超时";
        break;
      }

      MitCommand command = motor.desired_command;
      const bool command_timeout =
        !motor.command_received ||
        (current_time - motor.last_command_time).seconds() > command_timeout_sec_;

      if (command_timeout) {
        if (!motor.command_timed_out) {
          motor.hold_position = motor.feedback.position;
          motor.command_timed_out = true;
        }
        command = makeHoldCommand(motor);
      } else {
        motor.command_timed_out = false;
      }

      bool was_clamped = false;
      command = applyMechanicalLimits(motor, command, was_clamped);
      if (was_clamped) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "%s控制指令触发机械层限幅",
          motor.name.c_str());
      }

      motor.active_command = command;
      commands[i] = command;
      can_ids[i] = motor.can_id;
      limits[i] = motor.mit_limits;
    }
  }

  if (!fault_reason.empty()) {
    triggerSystemFault(fault_reason);
    return;
  }

  for (std::size_t i = 0; i < commands.size(); ++i) {
    if (!can_driver_->sendMitCommand(can_ids[i], commands[i], limits[i])) {
      triggerSystemFault("向CAN ID 0x" + [&]() {
        std::ostringstream stream;
        stream << std::hex << std::uppercase << can_ids[i];
        return stream.str();
      }() + "发送MIT指令失败");
      return;
    }
  }

  if ((current_time - last_state_publish_time_).seconds() >=
    static_cast<double>(state_publish_period_ms_) / 1000.0)
  {
    publishStateAndStatus();
    last_state_publish_time_ = current_time;
  }
}

void HalCabinMotorNode::publishStateAndStatus()
{
  if (!state_pub_ || !status_pub_ || !state_pub_->is_activated() ||
    !status_pub_->is_activated())
  {
    return;
  }

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now();
  std_msgs::msg::String status;
  std::ostringstream status_stream;
  status_stream << std::fixed << std::setprecision(3);

  {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      const auto & motor = motors_[i];
      joint_state.name.push_back(motor.name);
      joint_state.position.push_back(motor.feedback.position);
      joint_state.velocity.push_back(motor.feedback.velocity);
      joint_state.effort.push_back(motor.feedback.torque);

      if (i > 0) {
        status_stream << " | ";
      }
      status_stream << motor.name
                    << " id=0x" << std::hex << std::uppercase << motor.can_id
                    << std::dec
                    << " state=" << statusCodeToString(motor.feedback.status_code)
                    << " p=" << motor.feedback.position
                    << " v=" << motor.feedback.velocity
                    << " t=" << motor.feedback.torque
                    << " mos=" << motor.feedback.mos_temperature
                    << " rotor=" << motor.feedback.rotor_temperature;
    }
  }

  status.data = status_stream.str();
  state_pub_->publish(joint_state);
  status_pub_->publish(status);
}

void HalCabinMotorNode::triggerSystemFault(const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (fault_latched_) {
      return;
    }
    fault_latched_ = true;
    fault_reason_ = reason;
    for (auto & motor : motors_) {
      motor.faulted = true;
    }
  }

  if (stop_all_on_single_fault_) {
    disableAllMotors();
  }

  RCLCPP_ERROR(get_logger(), "舱段系统故障锁存：%s", reason.c_str());
  if (fault_pub_ && fault_pub_->is_activated()) {
    std_msgs::msg::String message;
    message.data = reason;
    fault_pub_->publish(message);
  }
}

std::string HalCabinMotorNode::statusCodeToString(uint8_t status_code)
{
  switch (status_code) {
    case 0x0: return "disabled";
    case 0x1: return "enabled";
    case 0x8: return "over_voltage";
    case 0x9: return "under_voltage";
    case 0xA: return "over_current";
    case 0xB: return "mos_over_temperature";
    case 0xC: return "rotor_over_temperature";
    case 0xD: return "communication_lost";
    case 0xE: return "overload";
    default: return "unknown";
  }
}

}  // namespace hal

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hal::HalCabinMotorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
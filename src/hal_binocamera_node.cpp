#include "hal_binocamera_node/hal_binocamera_node.hpp"
//commit
#include <chrono>
#include <cstring>
#include <depthai/depthai.hpp>
#include <exception>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>

namespace hal_binocamera
{

struct HalBinocameraNode::Impl
{
  std::unique_ptr<dai::Device> device_;
  std::shared_ptr<dai::DataOutputQueue> color_queue_;
  std::shared_ptr<dai::DataOutputQueue> depth_queue_;
};

namespace
{

constexpr char kLeftImageTopic[] = "/uvms/perception/image_raw";
constexpr char kDepthImageTopic[] = "/uvms/perception/depth";
constexpr char kCameraStatusTopic[] = "/uvms/perception/camera_status";
constexpr char kColorStreamName[] = "color";
constexpr char kDepthStreamName[] = "depth";
constexpr std::uint8_t kCameraStatusOk = 0x00;
constexpr std::uint8_t kCameraStatusFault = 0x01;
constexpr int kMaxGrabFailCount = 5;

}  // namespace

HalBinocameraNode::HalBinocameraNode()
: rclcpp_lifecycle::LifecycleNode("hal_binocamera_node"),
  pimpl_(std::make_unique<Impl>()),
  grab_fail_count_(0),
  is_camera_open_(false),
  camera_enabled_(true)
{
  declareParameters();
}

HalBinocameraNode::~HalBinocameraNode()
{
  closeCamera();
}

void HalBinocameraNode::declareParameters()
{
  this->declare_parameter("camera_fps", 30.0);
  this->declare_parameter("color_resolution", std::string("THE_1080_P"));
  this->declare_parameter("color_width", 1280);
  this->declare_parameter("color_height", 720);
  this->declare_parameter("mono_resolution", std::string("THE_720_P"));
  this->declare_parameter("stereo_confidence_threshold", 200);
  this->declare_parameter("stereo_left_right_check", true);
  this->declare_parameter("stereo_extended_disparity", false);
  this->declare_parameter("stereo_subpixel", true);
  this->declare_parameter("device_mx_id", std::string(""));
  this->declare_parameter("grab_period_ms", 33);
  this->declare_parameter("frame_id_left", std::string("oak_rgb_camera_optical_frame"));
  this->declare_parameter("frame_id_depth", std::string("oak_rgb_camera_optical_frame"));
}

CallbackReturn HalBinocameraNode::on_configure(const rclcpp_lifecycle::State &)
{
  left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    kLeftImageTopic, rclcpp::SensorDataQoS());
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    kDepthImageTopic, rclcpp::SensorDataQoS());
  status_pub_ = this->create_publisher<hal::msg::HalbinocameraMsg>(
    kCameraStatusTopic, rclcpp::SystemDefaultsQoS());

  toggle_camera_service_ =
    this->create_service<hal::srv::HalbinocameraSrv>(
    "~/toggle_camera",
    std::bind(
      &HalBinocameraNode::handleToggleCamera,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  if (!openCamera()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open OAK-D-SR camera during configure.");
    return CallbackReturn::FAILURE;
  }

  const auto period_ms = this->get_parameter("grab_period_ms").as_int();
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&HalBinocameraNode::captureAndPublish, this));
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "hal_binocamera_node configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HalBinocameraNode::on_activate(const rclcpp_lifecycle::State &)
{
  left_pub_->on_activate();
  depth_pub_->on_activate();
  status_pub_->on_activate();

  if (timer_) {
    timer_->reset();
  }

  RCLCPP_INFO(this->get_logger(), "hal_binocamera_node activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HalBinocameraNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (timer_) {
    timer_->cancel();
  }

  left_pub_->on_deactivate();
  depth_pub_->on_deactivate();
  status_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "hal_binocamera_node deactivated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HalBinocameraNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  left_pub_.reset();
  depth_pub_.reset();
  status_pub_.reset();
  toggle_camera_service_.reset();
  closeCamera();

  RCLCPP_INFO(this->get_logger(), "hal_binocamera_node cleaned up.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HalBinocameraNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (timer_) {
    timer_->cancel();
  }

  closeCamera();
  return CallbackReturn::SUCCESS;
}

CallbackReturn HalBinocameraNode::on_error(const rclcpp_lifecycle::State &)
{
  if (timer_) {
    timer_->cancel();
  }

  closeCamera();
  return CallbackReturn::SUCCESS;
}

bool HalBinocameraNode::openCamera()
{
  closeCamera();

  try {
    dai::Pipeline pipeline;

    auto color_camera = pipeline.create<dai::node::ColorCamera>();
    auto mono_left = pipeline.create<dai::node::MonoCamera>();
    auto mono_right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto color_xout = pipeline.create<dai::node::XLinkOut>();
    auto depth_xout = pipeline.create<dai::node::XLinkOut>();

    color_xout->setStreamName(kColorStreamName);
    depth_xout->setStreamName(kDepthStreamName);

    const auto color_width = this->get_parameter("color_width").as_int();
    const auto color_height = this->get_parameter("color_height").as_int();
    const auto camera_fps = static_cast<float>(this->get_parameter("camera_fps").as_double());

    color_camera->setBoardSocket(dai::CameraBoardSocket::CAM_A);

    const auto color_resolution_str = this->get_parameter("color_resolution").as_string();
    dai::ColorCameraProperties::SensorResolution color_resolution;
    if (color_resolution_str == "THE_4_K") {
      color_resolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
    } else if (color_resolution_str == "THE_12_MP") {
      color_resolution = dai::ColorCameraProperties::SensorResolution::THE_12_MP;
    } else if (color_resolution_str == "THE_13_MP") {
      color_resolution = dai::ColorCameraProperties::SensorResolution::THE_13_MP;
    } else {
      if (color_resolution_str != "THE_1080_P") {
        RCLCPP_WARN(
          this->get_logger(),
          "Unsupported color_resolution '%s', defaulting to THE_1080_P.",
          color_resolution_str.c_str());
      }
      color_resolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    }
    color_camera->setResolution(color_resolution);

    color_camera->setPreviewSize(color_width, color_height);
    color_camera->setFps(camera_fps);
    color_camera->setInterleaved(false);
    color_camera->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    mono_left->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    mono_right->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    const auto mono_resolution_str = this->get_parameter("mono_resolution").as_string();
    dai::MonoCameraProperties::SensorResolution mono_resolution;
    if (mono_resolution_str == "THE_400_P") {
      mono_resolution = dai::MonoCameraProperties::SensorResolution::THE_400_P;
    } else if (mono_resolution_str == "THE_480_P") {
      mono_resolution = dai::MonoCameraProperties::SensorResolution::THE_480_P;
    } else if (mono_resolution_str == "THE_800_P") {
      mono_resolution = dai::MonoCameraProperties::SensorResolution::THE_800_P;
    } else {
      if (mono_resolution_str != "THE_720_P") {
        RCLCPP_WARN(
          this->get_logger(),
          "Unsupported mono_resolution '%s', defaulting to THE_720_P.",
          mono_resolution_str.c_str());
      }
      mono_resolution = dai::MonoCameraProperties::SensorResolution::THE_720_P;
    }
    mono_left->setResolution(mono_resolution);
    mono_right->setResolution(mono_resolution);

    mono_left->setFps(camera_fps);
    mono_right->setFps(camera_fps);

    stereo->initialConfig.setConfidenceThreshold(
      this->get_parameter("stereo_confidence_threshold").as_int());
    stereo->setLeftRightCheck(this->get_parameter("stereo_left_right_check").as_bool());
    stereo->setExtendedDisparity(
      this->get_parameter("stereo_extended_disparity").as_bool());
    stereo->setSubpixel(this->get_parameter("stereo_subpixel").as_bool());
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    stereo->setOutputSize(color_width, color_height);

    color_camera->preview.link(color_xout->input);
    mono_left->out.link(stereo->left);
    mono_right->out.link(stereo->right);
    stereo->depth.link(depth_xout->input);

    const auto mx_id = this->get_parameter("device_mx_id").as_string();

    if (mx_id.empty()) {
      pimpl_->device_ = std::make_unique<dai::Device>(pipeline);
    } else {
      pimpl_->device_ = std::make_unique<dai::Device>(pipeline, dai::DeviceInfo(mx_id));
    }

    pimpl_->color_queue_ = pimpl_->device_->getOutputQueue(kColorStreamName, 4, false);
    pimpl_->depth_queue_ = pimpl_->device_->getOutputQueue(kDepthStreamName, 4, false);

    grab_fail_count_ = 0;
    is_camera_open_ = true;
    RCLCPP_INFO(this->get_logger(), "OAK-D-SR camera opened successfully.");
    return true;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to open OAK-D-SR camera: %s",
      exception.what());
    closeCamera();
    return false;
  }
}

void HalBinocameraNode::closeCamera()
{
  if (!pimpl_) {
    return;
  }

  pimpl_->depth_queue_.reset();
  pimpl_->color_queue_.reset();
  pimpl_->device_.reset();

  is_camera_open_ = false;
  grab_fail_count_ = 0;
}

void HalBinocameraNode::captureAndPublish()
{
  if (!left_pub_ || !depth_pub_ || !status_pub_) {
    return;
  }

  if (!left_pub_->is_activated() || !depth_pub_->is_activated() || !status_pub_->is_activated()) {
    return;
  }

  if (!camera_enabled_) {
    return;
  }

  if (!is_camera_open_) {
    publishStatus(kCameraStatusFault);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Camera is not open, attempting reconnection.");
    openCamera();
    return;
  }

  try {
    auto color_frame =
      pimpl_->color_queue_ ? pimpl_->color_queue_->tryGet<dai::ImgFrame>() : nullptr;
    auto depth_frame =
      pimpl_->depth_queue_ ? pimpl_->depth_queue_->tryGet<dai::ImgFrame>() : nullptr;

    if (!color_frame && !depth_frame) {
      publishStatus(kCameraStatusFault);
      ++grab_fail_count_;

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "No frames received from DepthAI output queues.");

      if (grab_fail_count_ >= kMaxGrabFailCount) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Frame retrieval failed %d times in a row, closing camera for reconnect.",
          grab_fail_count_);
        closeCamera();
      }
      return;
    }

    grab_fail_count_ = 0;
    publishStatus(kCameraStatusOk);

    if (color_frame) {
      const auto width = static_cast<int>(color_frame->getWidth());
      const auto height = static_cast<int>(color_frame->getHeight());
      const auto expected_step = width * 3;
      const auto expected_bytes = static_cast<std::size_t>(expected_step) * height;
      const auto & raw = color_frame->getData();

      if (width > 0 && height > 0 && raw.size() >= expected_bytes) {
        std::vector<std::uint8_t> image_data(raw.begin(), raw.end());
        publishColorImage(image_data, width, height, expected_step);
      }
    }

    if (depth_frame) {
      const auto width = static_cast<int>(depth_frame->getWidth());
      const auto height = static_cast<int>(depth_frame->getHeight());
      const auto expected_step = width * static_cast<int>(sizeof(std::uint16_t));
      const auto expected_bytes = static_cast<std::size_t>(expected_step) * height;
      const auto & raw = depth_frame->getData();

      if (width > 0 && height > 0 && raw.size() >= expected_bytes) {
        std::vector<std::uint16_t> depth_data;
        depth_data.reserve(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
        for (std::size_t i = 0; i + sizeof(std::uint16_t) <= raw.size(); i += sizeof(std::uint16_t)) {
          std::uint16_t pixel_value;
          std::memcpy(&pixel_value, &raw[i], sizeof(std::uint16_t));
          depth_data.push_back(pixel_value);
        }
        publishDepthImage(depth_data, width, height, expected_step);
      }
    }
  } catch (const std::exception & exception) {
    publishStatus(kCameraStatusFault);
    RCLCPP_ERROR(
      this->get_logger(),
      "DepthAI capture error: %s",
      exception.what());
    closeCamera();
  }
}

void HalBinocameraNode::publishStatus(std::uint8_t status_code)
{
  if (!status_pub_ || !status_pub_->is_activated()) {
    return;
  }

  hal::msg::HalbinocameraMsg message;
  message.header.stamp = this->now();
  message.header.frame_id = this->get_parameter("frame_id_left").as_string();
  message.status_code = status_code;
  status_pub_->publish(message);
}

void HalBinocameraNode::handleToggleCamera(
  const std::shared_ptr<hal::srv::HalbinocameraSrv::Request> request,
  std::shared_ptr<hal::srv::HalbinocameraSrv::Response> response)
{
  camera_enabled_ = request->enable;

  if (camera_enabled_) {
    response->success = true;
    response->message = "Camera enabled";
    RCLCPP_INFO(this->get_logger(), "Camera capture and publishing enabled.");
    return;
  }

  response->success = true;
  response->message = "Camera disabled";
  RCLCPP_INFO(this->get_logger(), "Camera capture and publishing disabled.");
}

void HalBinocameraNode::publishColorImage(
  const std::vector<std::uint8_t> & image_data,
  int width,
  int height,
  int stride)
{
  if (image_data.empty() || width <= 0 || height <= 0) {
    return;
  }

  sensor_msgs::msg::Image message;
  message.header.stamp = this->now();
  message.header.frame_id = this->get_parameter("frame_id_left").as_string();
  message.height = height;
  message.width = width;
  message.encoding = "bgr8";
  message.is_bigendian = false;
  message.step = static_cast<sensor_msgs::msg::Image::_step_type>(stride);
  message.data.resize(image_data.size());
  std::memcpy(message.data.data(), image_data.data(), image_data.size());

  left_pub_->publish(message);
}

void HalBinocameraNode::publishDepthImage(
  const std::vector<std::uint16_t> & depth_data,
  int width,
  int height,
  int stride)
{
  if (depth_data.empty() || width <= 0 || height <= 0) {
    return;
  }

  sensor_msgs::msg::Image message;
  message.header.stamp = this->now();
  message.header.frame_id = this->get_parameter("frame_id_depth").as_string();
  message.height = height;
  message.width = width;
  message.encoding = "16UC1";
  message.is_bigendian = false;
  message.step = static_cast<sensor_msgs::msg::Image::_step_type>(stride);
  message.data.resize(depth_data.size() * sizeof(std::uint16_t));
  std::memcpy(message.data.data(), depth_data.data(), depth_data.size() * sizeof(std::uint16_t));

  depth_pub_->publish(message);
}

}  // namespace hal_binocamera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<hal_binocamera::HalBinocameraNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
cmake_minimum_required(VERSION 3.8)
project(uvms_hal_manipulator)

# 强制要求 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 头文件目录
include_directories(include)

# 2. 接口（保留你原来的）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HalTailservoMsg.msg"
  "msg/HalWingservoMsg.msg"
  "msg/HalMainthrusterMsg.msg"
  "msg/HalAuxithrusterMsg.msg"
  "srv/HalServocontrolSrv.srv"
  "srv/HalThrustercontrolSrv.srv"
  "msg/HalBattery.msg"
  "srv/HalBatteryControlSrv.srv"
  "msg/HalInertialnavi.msg"
  "msg/HalDvl.msg"
  "msg/HalDepthsensor.msg"
  "msg/HalAntenna.msg"
  "msg/CanMsgIn.msg"
  "msg/CanMsgOut.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

set(GEN_INCLUDE_DIR "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>")

# ==============================================================================
# ====================== 机械臂核心库（我们新增的）==============================
# ==============================================================================
add_library(manipulator_core
  src/uvms_hal_manipulator/can_driver.cpp
  src/uvms_hal_manipulator/protocol_parser.cpp
  src/uvms_hal_manipulator/safety_manager.cpp
)
ament_target_dependencies(manipulator_core
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  lifecycle_msgs
)
include_directories(include ${GEN_INCLUDE_DIR})

# ==============================================================================
# ====================== 机械臂驱动节点 =========================================
# ==============================================================================
add_executable(manipulator_driver
  src/uvms_hal_manipulator/manipulator_node.cpp
)
target_link_libraries(manipulator_driver manipulator_core)
ament_target_dependencies(manipulator_driver
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  lifecycle_msgs
)

# ==============================================================================
# ====================== 双臂生命周期管理节点 ===================================
# ==============================================================================
add_executable(dual_arm_lifecycle_manager
  src/uvms_hal_manipulator/dual_arm_manager.cpp
)
ament_target_dependencies(dual_arm_lifecycle_manager
  rclcpp
  std_msgs
  lifecycle_msgs
)

# ==============================================================================
# ====================== 你原来的其他节点（保持不变）============================
# ==============================================================================

# 电池
add_executable(hal_battery_node src/hal_battery_node.cpp)
ament_target_dependencies(hal_battery_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# 天线
add_executable(hal_antennacontrol_node src/hal_antennacontrol_node.cpp)
ament_target_dependencies(hal_antennacontrol_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# 灯光
add_executable(hal_lightcontrol_node src/hal_lightcontrol_node.cpp)
ament_target_dependencies(hal_lightcontrol_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# 舵机
add_executable(hal_servo_node src/hal_servo_node.cpp)
ament_target_dependencies(hal_servo_node rclcpp rclcpp_lifecycle std_msgs)

# 推进器
add_executable(hal_thruster_node src/hal_thruster_node.cpp)
ament_target_dependencies(hal_thruster_node rclcpp rclcpp_lifecycle std_msgs std_srvs)

# 水深
add_executable(hal_depthsensor_node src/hal_depthsensor_node.cpp)
ament_target_dependencies(hal_depthsensor_node rclcpp rclcpp_lifecycle std_msgs)

# DVL
add_executable(hal_dvl_node src/hal_dvl_node.cpp)
ament_target_dependencies(hal_dvl_node rclcpp rclcpp_lifecycle std_msgs)

# 惯导
add_executable(hal_inertialnavi_node src/hal_inertialnavi_node.cpp)
ament_target_dependencies(hal_inertialnavi_node rclcpp rclcpp_lifecycle std_msgs)

# 通信
add_executable(bsp_comm_node src/bsp_comm_node.cpp)
ament_target_dependencies(bsp_comm_node rclcpp rclcpp_lifecycle std_msgs)

# 链接接口
rosidl_target_interfaces(hal_battery_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_antennacontrol_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_lightcontrol_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_servo_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_thruster_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_depthsensor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_dvl_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_inertialnavi_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(bsp_comm_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ==============================================================================
# ====================== 安装所有可执行文件 ====================================
# ==============================================================================
install(TARGETS
  hal_battery_node
  hal_antennacontrol_node
  hal_lightcontrol_node
  hal_servo_node
  hal_thruster_node
  hal_depthsensor_node
  hal_dvl_node
  hal_inertialnavi_node
  bsp_comm_node
  manipulator_driver
  dual_arm_lifecycle_manager
  DESTINATION lib/${PROJECT_NAME}
)

# 安装文件夹
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# 导出
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  trajectory_msgs
)

ament_package()
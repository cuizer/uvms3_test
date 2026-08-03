cmake_minimum_required(VERSION 3.12)
project(hal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# 基础与硬件依赖项寻包（合并并全面启用）
# ==============================================================================
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

# PkgConfig 与原生 GPIO 库 (Jetson 硬件依赖)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# depthai 相机库 (Jetson 硬件依赖)
find_package(depthai CONFIG REQUIRED
   PATHS /home/nvidia/depthai-core/build/install/lib/cmake/depthai
   NO_DEFAULT_PATH
)
 find_package(cv_bridge REQUIRED)
 find_package(OpenCV REQUIRED)

include_directories(include)

# ==============================================================================
# ROS 2 接口生成（合并所有自定义 msg/srv）
# ==============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HalTailservo.msg"
  "msg/HalWingservo.msg"
  "msg/HalMainthruster.msg"
  "msg/HalAuxithruster.msg"
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
  "msg/HalbinocameraMsg.msg"
  "srv/HalbinocameraSrv.srv"
  "msg/HalArmmotor.msg"
  "srv/HalArmmotorSrv.srv"
  "msg/CanFrameManipulator.msg" # 左边版本独有

  DEPENDENCIES std_msgs builtin_interfaces
)

# ==============================================================================
# 核心业务节点配置（包含所有旧版、新版及测试节点）
# ==============================================================================

# 1. 机械臂驱动节点（合并源文件列表，加入 protocol_parser.cpp）
add_executable(manipulator_driver
  src/uvms_hal_manipulator/manipulator_node.cpp
  src/uvms_hal_manipulator/can_driver.cpp
  #src/uvms_hal_manipulator/protocol_parser.cpp
  src/uvms_hal_manipulator/safety_manager.cpp
)
ament_target_dependencies(manipulator_driver
  rclcpp rclcpp_lifecycle std_msgs sensor_msgs geometry_msgs trajectory_msgs lifecycle_msgs std_srvs
)

# 2. 双臂生命周期管理器
add_executable(dual_arm_lifecycle_manager src/uvms_hal_manipulator/dual_arm_manager.cpp)
ament_target_dependencies(dual_arm_lifecycle_manager rclcpp std_msgs lifecycle_msgs)

# 3. 关节电机节点
add_executable(armmotor src/uvms_hal_manipulator/armmotor.cpp)
ament_target_dependencies(armmotor rclcpp)

# 4. 机械臂轨迹规划节点
add_executable(bsp_arm_trajectory_node src/uvms_hal_manipulator/bsp_arm_trajectory_node.cpp)
ament_target_dependencies(bsp_arm_trajectory_node rclcpp sensor_msgs trajectory_msgs)

# 5. CAN 总线管理器 (左边版本独有)
add_executable(can_manager
  src/uvms_hal_manipulator/can_manager_node.cpp
  src/uvms_hal_manipulator/can_driver.cpp
)
ament_target_dependencies(can_manager rclcpp std_msgs builtin_interfaces)

# 添加 hal_cabinmotor_node 节点 (本地独有)
add_executable(hal_cabinmotor_node
  src/hal_cabinmotor_node.cpp
)

target_include_directories(hal_cabinmotor_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(hal_cabinmotor_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  std_srvs
  trajectory_msgs
)

target_link_libraries(hal_cabinmotor_node
  Threads::Threads
)

# ==============================================================================

# 6. 基础硬件抽象层节点群
add_executable(hal_battery_node src/hal_battery_node.cpp)
ament_target_dependencies(hal_battery_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

add_executable(hal_antennacontrol_node src/hal_antennacontrol_node.cpp)
ament_target_dependencies(hal_antennacontrol_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# 灯光控制节点 (全面启用并链接 gpiod)
add_executable(hal_lightcontrol_node src/hal_lightcontrol_node.cpp)
ament_target_dependencies(hal_lightcontrol_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)
target_link_libraries(hal_lightcontrol_node gpiod)

add_executable(hal_servo_node src/hal_servo_node.cpp)
ament_target_dependencies(hal_servo_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(hal_thruster_node src/hal_thruster_node.cpp)
ament_target_dependencies(hal_thruster_node rclcpp rclcpp_lifecycle std_msgs std_srvs)

add_executable(hal_depthsensor_node src/hal_depthsensor_node.cpp)
ament_target_dependencies(hal_depthsensor_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(hal_dvl_node src/hal_dvl_node.cpp)
ament_target_dependencies(hal_dvl_node rclcpp rclcpp_lifecycle std_msgs std_srvs)

add_executable(hal_inertialnavi_node src/hal_inertialnavi_node.cpp)
ament_target_dependencies(hal_inertialnavi_node rclcpp rclcpp_lifecycle std_msgs)

# 7. 基础通信与运动控制节点 (全面启用并链接 OpenCV)
add_executable(bsp_comm_node src/bsp_comm_node.cpp)
ament_target_dependencies(bsp_comm_node rclcpp rclcpp_lifecycle std_msgs cv_bridge)
target_link_libraries(bsp_comm_node ${OpenCV_LIBS})

add_executable(bsp_motioncontrol_node src/bsp_motioncontrol_node.cpp)
ament_target_dependencies(bsp_motioncontrol_node rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)

add_executable(app_keyboard_control_node src/app_keyboard_control_node.cpp)
ament_target_dependencies(app_keyboard_control_node rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)

add_executable(app_motion_target_node src/app_motion_target_node.cpp)
ament_target_dependencies(app_motion_target_node rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)

add_executable(app_motion_mode_manager_node src/app_motion_mode_manager_node.cpp)
ament_target_dependencies(app_motion_mode_manager_node rclcpp lifecycle_msgs)
target_link_libraries(app_motion_mode_manager_node app_motion_mode_core)

# 9. 传感器模拟节点 (测试用)
add_executable(sensor_mock_node src/sensor_mock_node.cpp)
ament_target_dependencies(sensor_mock_node rclcpp rclcpp_lifecycle std_msgs)

# 10. 远程运动模式管理静态库与节点 (右边版本新增)
add_library(app_motion_mode_core STATIC src/app_motion_mode_coordinator.cpp)
target_include_directories(app_motion_mode_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)


# ==============================================================================
# 为所有需要自定义接口的节点绑定 rosidl 依赖
# ==============================================================================
rosidl_target_interfaces(manipulator_driver ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(armmotor ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(can_manager ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(sensor_mock_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_battery_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_antennacontrol_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_lightcontrol_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_servo_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_thruster_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_depthsensor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_dvl_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_inertialnavi_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(bsp_comm_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(bsp_motioncontrol_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(app_keyboard_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(app_motion_target_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


# ==============================================================================
# 安装与导出配置
# ==============================================================================
install(TARGETS
  manipulator_driver
  dual_arm_lifecycle_manager
  armmotor
  bsp_arm_trajectory_node
  can_manager
  hal_cabinmotor_node
  hal_battery_node
  hal_antennacontrol_node
  hal_lightcontrol_node
  hal_servo_node
  hal_thruster_node
  hal_depthsensor_node
  hal_dvl_node
  hal_inertialnavi_node
  bsp_comm_node
  bsp_motioncontrol_node
  app_motion_mode_manager_node
  app_keyboard_control_node
  app_motion_target_node
  sensor_mock_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include/)

# ==============================================================================
# 自动化测试配置 (右边版本新增)
# ==============================================================================
if(BUILD_TESTING)
  enable_testing()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_app_motion_mode_core test/test_app_motion_mode_core.cpp)
  target_link_libraries(test_app_motion_mode_core app_motion_mode_core)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_app_motion_naming test/test_app_motion_naming.py)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # add_executable(test_app_motion_mode_core test/test_app_motion_mode_core.cpp)
  # target_link_libraries(test_app_motion_mode_core app_motion_mode_core)
  # add_test(NAME app_motion_mode_core COMMAND test_app_motion_mode_core)
  add_test(
    NAME app_motion_naming
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_app_motion_naming.py
  )
endif()

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
cmake_minimum_required(VERSION 3.8)
project(hal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

include_directories(include)

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
  "msg/HalbinocameraMsg.msg" 
  "srv/HalbinocameraSrv.srv" 
  DEPENDENCIES std_msgs builtin_interfaces
)

# ==============================================================================
# �� 直接把所有源码合成一个节点！彻底消灭库错误！
# ==============================================================================
add_executable(manipulator_driver
  src/uvms_hal_manipulator/manipulator_node.cpp
  src/uvms_hal_manipulator/can_driver.cpp
  src/uvms_hal_manipulator/protocol_parser.cpp
  src/uvms_hal_manipulator/safety_manager.cpp
)
ament_target_dependencies(manipulator_driver
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  lifecycle_msgs
  std_srvs
)

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

add_executable(hal_battery_node src/hal_battery_node.cpp)
ament_target_dependencies(hal_battery_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

add_executable(hal_antennacontrol_node src/hal_antennacontrol_node.cpp)
ament_target_dependencies(hal_antennacontrol_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

add_executable(hal_lightcontrol_node src/hal_lightcontrol_node.cpp)
ament_target_dependencies(hal_lightcontrol_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

add_executable(hal_servo_node src/hal_servo_node.cpp)
ament_target_dependencies(hal_servo_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(hal_thruster_node src/hal_thruster_node.cpp)
ament_target_dependencies(hal_thruster_node rclcpp rclcpp_lifecycle std_msgs std_srvs)

add_executable(hal_depthsensor_node src/hal_depthsensor_node.cpp)
ament_target_dependencies(hal_depthsensor_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(hal_dvl_node src/hal_dvl_node.cpp)
ament_target_dependencies(hal_dvl_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(hal_inertialnavi_node src/hal_inertialnavi_node.cpp)
ament_target_dependencies(hal_inertialnavi_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(hal_binocamera_node src/hal_binocamera_node.cpp) 
ament_target_dependencies(hal_binocamera_node rclcpp rclcpp_lifecycle std_msgs sensor_msgs geometry_msgs std_srvs)

add_executable(bsp_comm_node src/bsp_comm_node.cpp)
ament_target_dependencies(bsp_comm_node rclcpp rclcpp_lifecycle std_msgs)

rosidl_target_interfaces(hal_battery_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_antennacontrol_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_lightcontrol_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_servo_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_thruster_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_depthsensor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_dvl_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_inertialnavi_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(hal_binocamera_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(bsp_comm_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  hal_battery_node
  hal_antennacontrol_node
  hal_lightcontrol_node
  hal_servo_node
  hal_thruster_node
  hal_depthsensor_node
  hal_dvl_node
  hal_inertialnavi_node
  hal_binocamera_node
  bsp_comm_node
  manipulator_driver
  dual_arm_lifecycle_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include/)

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

# 查找DepthAI库
find_program(PYTHON3_EXECUTABLE NAMES python3)
if(PYTHON3_EXECUTABLE)
  execute_process(
    COMMAND ${PYTHON3_EXECUTABLE} -c "import depthai; import os; print(os.path.dirname(depthai.__file__), end='')"
    OUTPUT_VARIABLE DEPTHAI_PYTHON_PATH
    RESULT_VARIABLE PYTHON_IMPORT_RESULT
    ERROR_QUIET
  )
  
  if(NOT ${PYTHON_IMPORT_RESULT})
    # 如果找到了Python的depthai路径，添加包含目录
    find_path(DEPTHAI_INCLUDE_DIR
      NAMES depthai.hpp
      PATHS ${DEPTHAI_PYTHON_PATH}/include
            ${DEPTHAI_PYTHON_PATH}/../include
            /usr/include
      PATH_SUFFIXES depthai
    )
  endif()
endif()

# 如果上面没找到，尝试使用pkg-config
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(DEPTHAI_PC QUIET depthai-core)
endif()

# 备选方案：直接寻找库文件
find_library(DEPTHAI_LIBRARY
  NAMES depthai
  PATHS /usr/lib /usr/local/lib
)

# 为hal_binocamera_node添加DepthAI支持
if(DEPTHAI_INCLUDE_DIR AND DEPTHAI_LIBRARY)
  target_include_directories(hal_binocamera_node PRIVATE ${DEPTHAI_INCLUDE_DIR})
  target_link_libraries(hal_binocamera_node ${DEPTHAI_LIBRARY})
elseif(EXISTS "${CMAKE_BINARY_DIR}/depthai-core/include")
  target_include_directories(hal_binocamera_node PRIVATE ${CMAKE_BINARY_DIR}/depthai-core/include)
  target_link_libraries(hal_binocamera_node depthai)
else()
  message(WARNING "Could not find DepthAI library, please install it or build from source")
endif()

ament_package()
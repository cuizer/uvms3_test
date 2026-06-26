from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 你的包名 = hal
    pkg_share = get_package_share_directory('hal')

    left_arm_params = os.path.join(pkg_share, 'config', 'left_arm.yaml')
    right_arm_params = os.path.join(pkg_share, 'config', 'right_arm.yaml')

    # ---------------- 左臂驱动节点 ----------------
    left_arm_node = LifecycleNode(
        package='hal',                       # 修正包名
        executable='manipulator_driver',     # 修正可执行文件
        name='manipulator_driver',
        namespace='left_arm',
        output='screen',
        parameters=[left_arm_params]
    )

    # ---------------- 右臂驱动节点 ----------------
    right_arm_node = LifecycleNode(
        package='hal',                       # 修正包名
        executable='manipulator_driver',     # 修正可执行文件
        name='manipulator_driver',
        namespace='right_arm',
        output='screen',
        parameters=[right_arm_params]
    )

    # ---------------- 双臂故障管理节点 ----------------
    dual_arm_manager_node = Node(
        package='hal',                       # 修正包名
        executable='dual_arm_lifecycle_manager',   # 你的可执行文件
        name='dual_arm_lifecycle_manager',
        output='screen'
    )

    # ---------------- 双臂电机状态汇总节点 ----------------
    # 功能：
    # 订阅 /left_arm/hal/armmotor
    # 订阅 /right_arm/hal/armmotor
    # 发布 /hal/armmotor 给上位机
    armmotor_node = Node(
        package='hal',
        executable='armmotor',
        name='armmotor',
        output='screen'
    )

    # ---------------- 左臂 BSP 轨迹规划节点 ----------------
    # 功能：
    # 订阅 /left_arm/hal/manipulator/joint_states
    # 订阅 /left_arm/bsp/manipulator/target_joint
    # 使用五次多项式插值生成中间轨迹点
    # 发布 /left_arm/hal/manipulator/joint_cmd 给左臂 HAL
    left_arm_bsp_trajectory_node = Node(
        package='hal',
        executable='bsp_arm_trajectory_node',
        name='bsp_arm_trajectory_node',
        namespace='left_arm',
        output='screen',
        parameters=[{
            'joint_state_topic': '/left_arm/hal/manipulator/joint_states',
            'target_joint_topic': '/left_arm/bsp/manipulator/target_joint',
            'joint_cmd_topic': '/left_arm/hal/manipulator/joint_cmd',
            'publish_rate_hz': 50.0,
            'default_duration_sec': 3.0,
        }]
    )

    # ---------------- 右臂 BSP 轨迹规划节点 ----------------
    # 功能：
    # 订阅 /right_arm/hal/manipulator/joint_states
    # 订阅 /right_arm/bsp/manipulator/target_joint
    # 使用五次多项式插值生成中间轨迹点
    # 发布 /right_arm/hal/manipulator/joint_cmd 给右臂 HAL
    right_arm_bsp_trajectory_node = Node(
        package='hal',
        executable='bsp_arm_trajectory_node',
        name='bsp_arm_trajectory_node',
        namespace='right_arm',
        output='screen',
        parameters=[{
            'joint_state_topic': '/right_arm/hal/manipulator/joint_states',
            'target_joint_topic': '/right_arm/bsp/manipulator/target_joint',
            'joint_cmd_topic': '/right_arm/hal/manipulator/joint_cmd',
            'publish_rate_hz': 50.0,
            'default_duration_sec': 3.0,
        }]
    )

    return LaunchDescription([
        left_arm_node,
        right_arm_node,
        dual_arm_manager_node,
        armmotor_node,
        left_arm_bsp_trajectory_node,
        right_arm_bsp_trajectory_node,
    ])
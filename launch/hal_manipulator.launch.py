from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 你的包名 = uvms_hal_manipulator
    pkg_share = get_package_share_directory('uvms_hal_manipulator')

    left_arm_params = os.path.join(pkg_share, 'config', 'left_arm.yaml')
    right_arm_params = os.path.join(pkg_share, 'config', 'right_arm.yaml')

    # ---------------- 左臂驱动节点 ----------------
    left_arm_node = LifecycleNode(
        package='uvms_hal_manipulator',      # 修正包名
        executable='manipulator_driver',    # 修正可执行文件
        name='manipulator_driver',
        namespace='left_arm',
        output='screen',
        parameters=[left_arm_params]
    )

    # ---------------- 右臂驱动节点 ----------------
    right_arm_node = LifecycleNode(
        package='uvms_hal_manipulator',      # 修正包名
        executable='manipulator_driver',    # 修正可执行文件
        name='manipulator_driver',
        namespace='right_arm',
        output='screen',
        parameters=[right_arm_params]
    )

    # ---------------- 双臂故障管理节点 ----------------
    dual_arm_manager_node = Node(
        package='uvms_hal_manipulator',            # 修正包名
        executable='dual_arm_lifecycle_manager',   # 你的可执行文件
        name='dual_arm_lifecycle_manager',
        output='screen'
    )

    return LaunchDescription([
        left_arm_node,
        right_arm_node,
        dual_arm_manager_node,
    ])
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 你的包名
    pkg_name = 'uvms_hal_manipulator'
    pkg_share = get_package_share_directory(pkg_name)

    # 单电机测试配置
    test_config = os.path.join(pkg_share, 'config', 'test_single_motor.yaml')

    # ----------------------- 单臂驱动节点（左臂）-----------------------
    single_arm_node = LifecycleNode(
        package='uvms_hal_manipulator',
        executable='manipulator_driver',
        name='manipulator_driver',
        namespace='left_arm',
        output='screen',
        parameters=[test_config]
    )

    # ----------------------- 双臂管理节点（也能监控单臂）-----------------------
    # 即使只有一个臂，也能正常测试故障停机
    dual_arm_manager_node = Node(
        package='uvms_hal_manipulator',
        executable='dual_arm_lifecycle_manager',
        name='dual_arm_lifecycle_manager',
        output='screen'
    )

    return LaunchDescription([
        single_arm_node,
        dual_arm_manager_node,
    ])
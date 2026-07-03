from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('hal')

    left_arm_params = os.path.join(pkg_share, 'config', 'left_arm.yaml')
    right_arm_params = os.path.join(pkg_share, 'config', 'right_arm.yaml')

    can_manager_node = Node(
        package='hal',
        executable='can_manager',
        name='can_manager',
        output='screen',
        parameters=[{
            'can_interface': 'can4',
            'send_interval_us': 300,
            'max_queue_size': 500,
        }]
    )

    left_arm_node = LifecycleNode(
        package='hal',
        executable='manipulator_driver',
        name='manipulator_driver',
        namespace='left_arm',
        output='screen',
        parameters=[left_arm_params]
    )

    right_arm_node = LifecycleNode(
        package='hal',
        executable='manipulator_driver',
        name='manipulator_driver',
        namespace='right_arm',
        output='screen',
        parameters=[right_arm_params]
    )

    dual_arm_manager_node = Node(
        package='hal',
        executable='dual_arm_lifecycle_manager',
        name='dual_arm_lifecycle_manager',
        output='screen'
    )

    armmotor_node = Node(
        package='hal',
        executable='armmotor',
        name='armmotor',
        output='screen',
        parameters=[{
            'enable_csv_logging': True,
            'csv_log_directory': 'armmotor_logs',
            'csv_log_file_prefix': 'armmotor',
            'csv_flush_every_n': 50,
        }]
    )

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
        can_manager_node,
        left_arm_node,
        right_arm_node,
        dual_arm_manager_node,
        armmotor_node,
        left_arm_bsp_trajectory_node,
        right_arm_bsp_trajectory_node,
    ])
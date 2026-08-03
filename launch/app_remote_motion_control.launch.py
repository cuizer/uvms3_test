"""Bring up all APP-managed motion nodes without auto-activation."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    keyboard = LifecycleNode(
        package="hal",
        executable="app_keyboard_control_node",
        name="app_keyboard_control_node",
        namespace="",
        output="screen",
        parameters=[{
            "udp_port": 5002,
            "control_rate_hz": 20.0,
            "cmd_timeout_s": 0.5,
            "surge_scale": 300.0,
            "sway_scale": 160.0,
            "heave_scale": 80.0,
            "yaw_scale": 5.0,
            "smoothing.enable": True,
            "smoothing.slew_Fx": 600.0,
            "smoothing.slew_Fy": 320.0,
            "smoothing.slew_Fz": 160.0,
            "smoothing.slew_Mz": 10.0,
            "smoothing.thruster_tau": 0.2,
        }],
    )

    target = LifecycleNode(
        package="hal",
        executable="app_motion_target_node",
        name="app_motion_target_node",
        namespace="",
        output="screen",
        parameters=[{
            "udp_port": 5003,
            "publish_rate_hz": 20.0,
            "max_velocity_x": 1.0,
            "max_velocity_y": 0.5,
            "cmd_timeout_s": 1.0,
        }],
    )

    controller = LifecycleNode(
        package="hal",
        executable="bsp_motioncontrol_node",
        name="bsp_motioncontrol_node",
        namespace="",
        output="screen",
    )

    manager = Node(
        package="hal",
        executable="app_motion_mode_manager_node",
        name="app_motion_mode_manager_node",
        namespace="",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            "manager_port": 5004,
            "service_timeout_ms": 1000,
            "request_cache_size": 256,
            "request_cache_ttl_ms": 30000,
        }],
    )

    return LaunchDescription([keyboard, target, controller, manager])



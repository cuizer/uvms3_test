"""Bring up BSP remote-control and PID motion-control nodes."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    remote = LifecycleNode(
        package="hal",
        executable="bsp_remotecontrol_node",
        name="bsp_remotecontrol_node",
        namespace="",
        output="screen",
        parameters=[{
            "mode_topic": "/hal/modecontrol",
            "remote_topic": "/hal/remotecontrol",
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

    controller = LifecycleNode(
        package="hal",
        executable="bsp_motioncontrol_node",
        name="bsp_motioncontrol_node",
        namespace="",
        output="screen",
        parameters=[{
            "mode_topic": "/hal/modecontrol",
            "remote_topic": "/hal/remotecontrol",
        }],
    )

    return LaunchDescription([remote, controller])

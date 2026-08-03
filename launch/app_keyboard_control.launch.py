from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
import launch.events
import lifecycle_msgs.msg


def generate_launch_description():
    """启动 app_keyboard_control_node (开环键盘直驱)"""

    node = LifecycleNode(
        package='hal',
        executable='app_keyboard_control_node',
        name='app_keyboard_control_node',
        namespace='',
        output='screen',
        parameters=[{
            'udp_port': 5002,
            'control_rate_hz': 20.0,
            'surge_scale': 300.0,
            'sway_scale': 160.0,
            'heave_scale': 80.0,
            'yaw_scale': 5.0,
            # 平滑控制 (v2)
            'smoothing.enable': True,
            'smoothing.slew_Fx': 600.0,     # N/s  surge 力变化率
            'smoothing.slew_Fy': 320.0,     # N/s  sway  力变化率
            'smoothing.slew_Fz': 160.0,     # N/s  heave 力变化率
            'smoothing.slew_Mz': 10.0,      # Nm/s yaw  力矩变化率
            'smoothing.thruster_tau': 0.2,  # s    推进器一阶时滞常数
        }],
    )

    register_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    register_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    return LaunchDescription([node, register_configure, register_activate])



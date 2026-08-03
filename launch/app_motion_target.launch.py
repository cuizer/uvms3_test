from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
import launch.events
import lifecycle_msgs.msg


def generate_launch_description():
    """启动 app_motion_target_node (UDP → /app/motioncontrol)"""

    node = LifecycleNode(
        package='hal',
        executable='app_motion_target_node',
        name='app_motion_target_node',
        namespace='',
        output='screen',
        parameters=[{
            'udp_port': 5003,
            'publish_rate_hz': 20.0,
            'max_velocity_x': 1.0,
            'max_velocity_y': 0.5,
            'cmd_timeout_s': 1.0,
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



from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
import launch.events  # <==== [修改点] 引入 Foxy 支持的基础事件匹配器
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hal')
    antenna_params = os.path.join(pkg_share, 'config', 'antenna.yaml')

    # 定义节点对象
    antenna_node = LifecycleNode(
        package='hal',
        executable='hal_antennacontrol_node',
        name='hal_antennacontrol_node',
        namespace='',
        output='screen',
        parameters=[antenna_params] 
    )

    # 自动化魔法 1：节点启动后，立刻自动发送 Configure 指令 (Foxy 写法)
    emit_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(antenna_node), # <==== [修改点] 直接绑定节点对象
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # 自动化魔法 2：监听到节点进入 inactive (已配置) 状态后，立刻自动发送 Activate 指令 (Foxy 写法)
    register_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=antenna_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(antenna_node), # <==== [修改点] 
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        antenna_node,
        emit_configure,
        register_activate
    ])

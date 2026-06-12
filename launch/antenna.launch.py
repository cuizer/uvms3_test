from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart  # <==== [新增] 引入进程启动事件监听
import launch.events  
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hal')
    antenna_params = os.path.join(pkg_share, 'config', 'antenna.yaml')

    # 1. 定义节点对象
    antenna_node = LifecycleNode(
        package='hal',
        executable='hal_antennacontrol_node',
        name='hal_antennacontrol_node',
        namespace='',
        output='screen',
        parameters=[antenna_params] 
    )

    # 2. 自动化魔法 1：监听节点"进程已启动"事件，随后发送 Configure 指令
    # 【修复竞态条件】：用 RegisterEventHandler 和 OnProcessStart 包装 EmitEvent
    register_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=antenna_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(antenna_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    # 3. 自动化魔法 2：监听到节点进入 'inactive' (已配置) 状态后，自动发送 Activate 指令
    register_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=antenna_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(antenna_node), 
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    # 返回编排好的 LaunchDescription
    return LaunchDescription([
        antenna_node,
        register_configure,  # <==== 使用包装好的 register_configure 替代原先直接裸奔的 EmitEvent
        register_activate
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number

from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
import lifecycle_msgs.msg

# https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/

# https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/launch/zed.launch.py
# Also need to add config to the CMakeLists.txt install(DIRECTORY
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    map_file = os.path.join(
        get_package_share_directory('nav_sim'),
        'config', 'blank_map.yaml'
    )
    
    load_map_node = Node(
        package="nav_sim",
        executable="load_map_client",
        name="blank_map_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"map_yaml_file": map_file}],
        remappings = [('/map_server/load_map', '/blank_map_server/load_map')]
    )
    
    # https://answers.ros.org/question/326070/ros2-nav2_map_server-can-not-load-map/
    # https://index.ros.org/p/nav2_map_server/
    map_node = LifecycleNode(
        package = 'nav2_map_server',
        namespace = '',
        executable='map_server',
        name='blank_map_server',
        parameters=[
            {"yaml_filename": map_file}
        ],
        remappings = [('/map', '/amap')]
    )
    # ros2 nav2_util lifecycle_bringup map_server
    # OR
    # ros2 lifecycle set map_server configure
    # ros2 lifecycle set map_server activate
    map_start_node = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        arguments=['blank_map_server'],
        remappings = [('/map', '/amap')]
    )
    cmd_string = 'ros2 lifecycle set blank_map_server configure & ros2 lifecycle set blank_map_server activate'
    map_start_cmd = ExecuteProcess(
            cmd=cmd_string.split(' '),
            output='screen'
    )
    configure_map_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] map_server node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(map_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    
    ld.add_action(load_map_node)
    ld.add_action(map_node)
    ld.add_action(configure_map_event)
    ld.add_action(activate_map_event)
    
    return ld

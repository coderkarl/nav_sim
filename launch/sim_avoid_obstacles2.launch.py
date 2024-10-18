from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number
from math import pi

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    nav_states_node = Node(
        package="nav_sim",
        executable="nav_states",
        output='screen',
        emulate_tty='True',
        parameters=[{'plan_rate_hz': 10.0},
                    {'use_PotFields': False},
                    {'close_cone_to_bot_dist': 0.5},
                    {'valid_cone_to_wp_dist': 0.5},
                    {'near_path_dist': 0.15},
                    {'valid_end_of_path_dist': 0.2},
                    {'desired_speed': 0.5},
                    {'slow_speed': 0.2},
                    {'max_omega': 0.75},
                    {'max_fwd_heading_error_deg': 40.0},
                    {'search_time': 2.0},
                    {'search_omega': 0.8},
                    {'reverse_time': 1.3},
                    {'cmd_control_ver': 0},
                    {'scan_collision_db_limit': 4},
                    {'scan_collision_range': 0.01},
                    {'cone_detect_db_limit': 1},
                    {'cmd_speed_filter_factor': 0.1},
                    {'report_bumped_obstacles': False},
                    {'max_camera_search_time': 30.0},
                    {'slow_approach_distance': 0.3},
                    {'reverse_speed': 0.3},
                    {'bump_db_limit': 2},
                    {'path_step_size': 3},
                    {'waypoints_are_in_map_frame': True},
                    {'x_coords0':          [2.2, 0.0]}, # SELECT, east side
                    {'y_coords0':          [0.0, 0.0]}, # 22 to -10
                    
                    {'waypoint_types':     [1,   0]},
                    {'hill_waypoint_list': [0,   0]},
                    {'is_mow_boundary': False},
                    {'mow_ccw': True},
                    {'mow_width': 0.2}, #0.2
                    {'mow_wp_spacing': 2.0},
        ],
        remappings=[('cam_cone_pose', 'raw_cone_pose')]
        #remappings=[('cmd_vel', 'ignore_cmd_vel')]
    )
    
    avoid_obs_node = Node(
        package="nav_sim",
        executable="avoid_obs",
        output='screen',
        emulate_tty='True',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{'plan_rate_hz': 10.0},
                    {'map_res_m': 0.1},
                    {'map_size': 301},
                    {'min_range': 0.05},
                    {'max_range': 10.0},
                    {'min_hill_range': 1.0},
                    {'plan_range': 15.0},
                    {'clear_decrement': -2},
                    {'adjacent_cost_offset': 0.5},
                    {'adjacent_cost_slope': 2.0},
                    {'inflation_factor': 3},
                    {'reinflate_cost_thresh': 30},
                    {'reinflate_radius': 0.2},
                    {'use_PotFields': False},
                    {'cone_search_radius': 1.0},
                    {'cone_obs_thresh': 0},
                    {'max_num_known_obstacles': 30},
                    {'known_obstacle_time_limit': 30.0},
                    {'useLinear': True},
                    {'useTan': True},
                    {'R1': 1.5},
                    {'R2': 2.0},
                    {'Kt': 10.0},
                    {'offset_gamma': pi/2},
                    {'max_heading_error': pi/3},
                    {'Kw': 1.3},
                    {'des_speed': 0.4},
                    {'min_omega': 0.5},
                    {'d_retreat': 1.7}
        ],
        remappings=[('known_obstacle', 'ignore_known_obs'), ('cmd_vel', 'pf_cmd_vel')]
    )
    
    astar_node = Node(
        package="nav_sim",
        executable="astar",
        output="screen",
        emulate_tty=True,
        parameters=[{'plan_rate_hz': 5.0},
                    {'obs_thresh': 30},
                    {'obs_weight': 0.1},
                    {'max_plan_time_sec': 1.0}
        ]
    )
    
    static_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        arguments=['0.0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    static_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        arguments=['0.0', '0', '0', '0', '0', '0', 'base_laser', 'laser']
    )
    
    ld.add_action(avoid_obs_node)
    ld.add_action(nav_states_node)
    ld.add_action(astar_node)
    ld.add_action(static_map_tf_node)
    ld.add_action(static_laser_tf_node)
    
    return ld

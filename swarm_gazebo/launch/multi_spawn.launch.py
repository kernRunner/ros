from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('swarm_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_name = 'tree_exploration_test'
    world_file = os.path.join(gazebo_pkg, 'worlds', f'{world_name}.sdf')
    model_file = os.path.join(gazebo_pkg, 'models', 'swarm_bot.sdf')

    robots = [
        {'name': 'robot1', 'x': '1.2', 'y':  '0.6', 'z': '0.0'},
        {'name': 'robot2', 'x': '1.2', 'y': '-0.6', 'z': '0.0'},
        {'name': 'robot3', 'x': '0.0', 'y':  '0.6', 'z': '0.0'},
        {'name': 'robot4', 'x': '0.0', 'y': '-0.6', 'z': '0.0'},
    ]

    active_robots = ['robot1', 'robot2', 'robot3', 'robot4']

    actions = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
        )
    )

    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '--ros-args',
                '-p', 'qos_overrides./clock.publisher.reliability:=best_effort',
                '-p', 'qos_overrides./clock.publisher.depth:=1',
            ],
        )
    )

    actions.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_robot1_map',
            arguments=[
                '--x', '0.0',
                '--y', '-9.0',
                '--z', '0.0',
                '--yaw', '-0.045',
                '--frame-id', 'world',
                '--child-frame-id', 'robot1/map',
            ],
        )
    )

    for robot in robots:
        ns = robot['name']
        is_active = ns in active_robots
        spawn_x = float(robot['x'])
        spawn_y = float(robot['y'])

        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', ns,
                    '-allow_renaming', 'false',
                    '-file', model_file,
                    '-x', robot['x'],
                    '-y', robot['y'],
                    '-z', robot['z'],
                ],
            )
        )

        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'world_to_{ns}_odom',
                arguments=[
                    '--x', robot['x'],
                    '--y', robot['y'],
                    '--z', '0.0',
                    '--yaw', '0.0',
                    '--frame-id', 'world',
                    '--child-frame-id', f'{ns}/odom',
                ],
            )
        )

        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{ns}_bridge',
                namespace=ns,
                output='screen',
                arguments=[
                    f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    f'/model/{ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    f'/world/{world_name}/model/{ns}/link/chassis/sensor/lidar/scan'
                    f'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                ],
                remappings=[
                    (f'/model/{ns}/cmd_vel', 'cmd_vel'),
                    (f'/model/{ns}/odometry', 'odom'),
                    (
                        f'/world/{world_name}/model/{ns}/link/chassis/sensor/lidar/scan',
                        'scan',
                    ),
                ],
            )
        )

        if not is_active:
            continue

        actions.append(
            Node(
                package='swarm_control',
                executable='odom_to_tf',
                namespace=ns,
                name='odom_to_tf',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'odom_topic': 'odom'},
                    {'base_frame': f'{ns}/chassis'},
                    {'odom_frame': f'{ns}/odom'},
                ],
            )
        )

        # If your SDF still has lidar at x=0.10 z=0.12, change this back.
        # If you use the centered/debug model, this is correct.
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{ns}_lidar_tf',
                arguments=[
                    '--x', '0.0',
                    '--y', '0.0',
                    '--z', '0.135',
                    '--yaw', '0.0',
                    '--frame-id', f'{ns}/chassis',
                    '--child-frame-id', f'{ns}/chassis/lidar',
                ],
            )
        )

        actions.append(
            Node(
                package='swarm_control',
                executable='swarm_member',
                namespace=ns,
                name='swarm_member',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'publish_rate_hz': 5.0},
                    {'state_timeout_sec': 1.5},
                    {'lock_leader_after_startup': True},
                    {'startup_election_delay_sec': 4.0},
                    {'spawn_x': spawn_x},
                    {'spawn_y': spawn_y},
                ],
            )
        )

        actions.append(
            Node(
                package='swarm_control',
                executable='leader_path_publisher',
                namespace=ns,
                name='leader_path_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'path_topic': '/swarm/leader_path'},
                    {'append_distance': 0.05},
                    {'append_yaw_delta': 0.05},
                    {'max_path_points': 4000},
                    {'frame_id': 'world'},
                    {'spawn_x': spawn_x},
                    {'spawn_y': spawn_y},
                ],
            )
        )

        actions.append(
            Node(
                package='swarm_control',
                executable='formation_manager',
                namespace=ns,
                name='formation_manager',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'spawn_x': spawn_x},
                    {'spawn_y': spawn_y},
                    {'post_election_wait_sec': 4.5},
                ],
            )
        )

        actions.append(
            Node(
                package='swarm_control',
                executable='path_follower',
                namespace=ns,
                name='path_follower',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'cmd_vel_topic': 'cmd_vel_raw'},
                    {'path_topic': '/swarm/leader_path'},
                    {'spawn_x': spawn_x},
                    {'spawn_y': spawn_y},

                    {'lookahead_m': 0.25},
                    {'goal_tolerance_m': 0.06},
                    {'min_path_length_m': 0.25},

                    {'desired_follow_distance_m': 1.15},
                    {'follow_deadband_m': 0.18},

                    # Followers are slightly faster than leader so they can catch up.
                    {'max_linear_speed': 0.14},
                    {'min_linear_speed_when_far': 0.04},
                    {'max_angular_speed': 1.10},
                    {'linear_gain': 0.45},
                    {'angular_gain': 1.55},

                    {'min_robot_distance_m': 0.50},
                    {'slow_robot_distance_m': 0.85},
                    {'too_close_reverse_speed': -0.02},

                    # Catch up earlier.
                    {'far_gap_m': 1.25},
                    {'very_far_gap_m': 1.75},
                    {'catchup_speed_boost': 1.35},

                    {'startup_delay_per_slot_sec': 0.40},
                    {'lock_chain_order': True},
                    {'fallback_to_predecessor': True},

                    {'cross_track_gain': 0.55},
                    {'max_cross_track_correction_rad': 0.35},

                    {'line_hold_enabled': True},
                    {'line_hold_gain': 0.50},
                    {'line_hold_integral_gain': 0.08},
                    {'line_hold_start_delay_sec': 2.0},
                    {'line_hold_max_correction_rad': 0.45},
                    {'line_hold_integral_limit': 0.35},

                    {'resync_enabled': True},
                    {'resync_lateral_error_m': 0.18},
                    {'resync_release_error_m': 0.07},
                    {'resync_angular_boost': 1.8},
                    {'resync_speed_scale': 0.55},
                ],
            )
        )

        actions.append(
            Node(
                package='swarm_control',
                executable='tree_explorer',
                namespace=ns,
                name='tree_explorer',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'cmd_vel_topic': 'cmd_vel_raw'},

                    # Leader speed lower than follower max speed.
                    {'forward_speed': 0.060},
                    {'turn_speed': 0.55},

                    # Obstacle clearance.
                    {'front_blocked_distance': 1.25},
                    {'side_clearance_distance': 0.75},
                    {'side_avoid_turn_gain': 0.30},

                    {'spawn_x': spawn_x},
                    {'spawn_y': spawn_y},
                    {'chain_spacing_m': 0.9},
                    {'formation_tolerance_m': 0.35},

                    {'leader_start_delay_sec': 7.0},
                    {'leader_wait_for_chain': True},

                    # Leader slows/stops much earlier when chain stretches.
                    {'leader_slow_chain_gap_m': 1.45},
                    {'leader_max_chain_gap_m': 1.90},
                    {'leader_wait_turn_allowed': True},

                    # Directional exploration.
                    # 90 = +Y/north in normal ROS coordinates.
                    # If your world uses +X as north, set this to 0.0.
                    {'preferred_heading_deg': 0.0},
                    {'heading_gain': 0.65},
                    {'max_heading_turn': 0.35},

                    # These only work after you add the blocked-mode logic
                    # described below in tree_explorer.py.
                    {'obstacle_escape_enabled': True},
                    {'escape_front_clear_distance': 1.60},
                    {'escape_rejoin_heading_error_deg': 25.0},
                    {'escape_min_time_sec': 2.0},
                ],
            )
        )

        actions.append(
            Node(
                package='swarm_control',
                executable='breadcrumb_manager',
                namespace=ns,
                name='breadcrumb_manager',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'frame_id': 'world'},
                    {'spawn_x': spawn_x},
                    {'spawn_y': spawn_y},
                    {'breadcrumb_distance': 1.0},
                    {'marker_topic': '/swarm/breadcrumb_markers_array'},
                ],
            )
        )

        actions.append(
            Node(
                package='swarm_control',
                executable='cmd_vel_safety_filter',
                namespace=ns,
                name='cmd_vel_safety_filter',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'enabled': True},

                    # Earlier reaction.
                    {'hard_stop_distance': 0.45},
                    {'slowdown_distance': 1.00},
                    {'side_stop_distance': 0.22},
                    {'side_slow_distance': 0.65},

                    {'wall_avoid_gain': 0.04},
                    {'max_safe_linear_speed': 0.14},
                ],
            )
        )

    actions.append(
        Node(
            package='swarm_control',
            executable='line_alignment_monitor',
            name='line_alignment_monitor',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'path_topic': '/swarm/leader_path'},
                {'state_topic': '/swarm/robot_states'},
                {'chain_order_topic': '/swarm/chain_order'},
                {'output_topic': '/swarm/line_alignment'},
                {'check_period_sec': 1.0},

                {'warn_path_lateral_error_m': 0.12},
                {'bad_path_lateral_error_m': 0.22},

                {'warn_neighbor_line_error_m': 0.12},
                {'bad_neighbor_line_error_m': 0.22},

                {'warn_leader_axis_error_m': 0.15},
                {'bad_leader_axis_error_m': 0.30},

                {'warn_lateral_spread_m': 0.15},
                {'bad_lateral_spread_m': 0.28},

                {'warn_yaw_error_rad': 0.18},
                {'bad_yaw_error_rad': 0.35},

                {'ignore_leader_for_path_error': True},
            ],
        )
    )

    return LaunchDescription(actions)
        # SLAM toolbox — leader only onyl for testing commented out
        # if ns == 'robot1':
        #     actions.append(
        #         Node(
        #             package='slam_toolbox',
        #             executable='async_slam_toolbox_node',
        #             namespace=ns, name='slam_toolbox', output='screen',
        #             parameters=[{
        #                 'use_sim_time': True,
        #                 'mode': 'mapping',
        #                 'transform_publish_period': 0.1,
        #                 'odom_frame': f'{ns}/odom',
        #                 'base_frame': f'{ns}/chassis',
        #                 'map_frame': f'{ns}/map',
        #                 'scan_topic': 'scan',
        #                 'map_name': 'map',
        #                 'resolution': 0.08,
        #                 'minimum_travel_distance': 0.10,
        #                 'minimum_travel_heading': 0.10,
        #             }],
        #             remappings=[('scan', 'scan')]
        #         )
        #     )


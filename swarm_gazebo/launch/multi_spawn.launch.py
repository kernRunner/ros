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
        {'name': 'robot1', 'x': '1.2',  'y':  '0.6', 'z': '0.0'},
        {'name': 'robot2', 'x': '1.2',  'y': '-0.6', 'z': '0.0'},
        {'name': 'robot3', 'x': '0.0',  'y':  '0.6', 'z': '0.0'},
        {'name': 'robot4', 'x': '0.0',  'y': '-0.6', 'z': '0.0'},
        {'name': 'robot5', 'x': '-1.2', 'y':  '0.6', 'z': '0.0'},
        {'name': 'robot6', 'x': '-1.2', 'y':  '-0.6', 'z': '0.0'},
    ]

    active_robots = ['robot1', 'robot2', 'robot3', 'robot4', 'robot5', 'robot6']
    actions = []

    # ------------------------------------------------------------
    # Start Gazebo
    # ------------------------------------------------------------
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
        )
    )

    # ------------------------------------------------------------
    # Clock bridge
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # Gazebo true-pose bridge
    #
    # IMPORTANT:
    # These two nodes are global. They must be started ONCE,
    # not once per robot.
    # ------------------------------------------------------------
    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_dynamic_pose_tf_bridge',
            output='screen',
            arguments=[
                f'/world/{world_name}/dynamic_pose/info'
                '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
        )
    )

    actions.append(
        Node(
            package='swarm_control',
            executable='gazebo_pose_bridge',
            name='gazebo_pose_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'gazebo_pose_topic': f'/world/{world_name}/dynamic_pose/info'},
                {'robot_names': active_robots},
            ],
        )
    )

    actions.append(
        Node(
            package='swarm_control',
            executable='relay_tree_manager',
            name='relay_tree_manager',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_names': active_robots},
                {'root_relay_name': 'robot1'},
                {'initial_leader_name': 'robot2'},
                {'split_distance_m': 4.0},
                {'branch_angle_deg': 35.0},
                {'publish_rate_hz': 2.0},
            ],
        )
    )

    # # Optional map transform
    # actions.append(
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='world_to_robot1_map',
    #         arguments=[
    #             '--x', '0.0',
    #             '--y', '-9.0',
    #             '--z', '0.0',
    #             '--yaw', '-0.045',
    #             '--frame-id', 'world',
    #             '--child-frame-id', 'robot1/map',
    #         ],
    #     )
    # )

    # ------------------------------------------------------------
    # Per-robot nodes
    # ------------------------------------------------------------
    for robot in robots:
        ns = robot['name']
        is_active = ns in active_robots
        spawn_x = float(robot['x'])
        spawn_y = float(robot['y'])

        # Spawn robot
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

        # Keep this TF for RViz odom visualization.
        # actions.append(
        #     Node(
        #         package='tf2_ros',
        #         executable='static_transform_publisher',
        #         # name=f'world_to_{ns}_odom',
        #         arguments=[
        #             '--x', robot['x'],
        #             '--y', robot['y'],
        #             '--z', '0.0',
        #             '--yaw', '0.0',
        #             '--frame-id', 'world',
        #             '--child-frame-id', f'{ns}/odom',
        #         ],
        #     )
        # )

        # Per-robot command, odom, and lidar bridges.
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
                executable='ground_truth_to_tf',
                namespace=ns,
                name='ground_truth_to_tf',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_name': ns},
                    {'pose_topic': 'ground_truth_pose'},
                    {'parent_frame': 'world'},
                    {'child_frame': f'{ns}/chassis'},
                ],
            )
        )

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

        # SwarmMember now uses Gazebo true pose.
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
                    {'state_timeout_sec': 3.0},
                    {'lock_leader_after_startup': True},
                    {'startup_election_delay_sec': 6.0},

                    # Ground-truth pose from gazebo_pose_bridge.
                    {'use_ground_truth_pose': True},
                    {'ground_truth_pose_topic': 'ground_truth_pose'},

                    # Fallback only. Ignored after ground truth starts.
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
                    {'append_distance': 0.12},
                    {'append_yaw_delta': 999.0},
                    {'max_path_points': 4000},
                    {'frame_id': 'world'},

                    # Keep zero because RobotState is now world ground truth.
                    {'spawn_x': 0.0},
                    {'spawn_y': 0.0},
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

                    # Keep zero with ground-truth/world RobotState.
                    {'spawn_x': 0.0},
                    {'spawn_y': 0.0},

                    {'post_election_wait_sec': 1.0},
                    {'chain_spacing_m': 1.15},
                    {'formation_heading_deg': 0.0},

                    {'goal_tolerance_m': 0.45},
                    {'yaw_tolerance_rad': 0.60},

                    {'max_linear_speed': 0.20},
                    {'max_angular_speed': 0.55},
                    {'linear_gain': 0.55},
                    {'angular_gain': 0.85},

                    {'hold_ready_after_aligned_sec': 0.25},
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

                    # Kept zero with ground-truth/world RobotState.
                    {'spawn_x': 0.0},
                    {'spawn_y': 0.0},

                    # Chain spacing.
                    {'desired_follow_distance_m': 1.15},
                    {'follow_deadband_m': 0.18},
                    {'hold_gap_deadband_m': 0.14},
                    {'hold_heading_deadband_rad': 0.25},

                    {'speed_match_enabled': True},
                    {'speed_match_gain': 0.85},
                    {'min_creep_speed': 0.025},
                    {'command_smoothing_alpha_linear': 0.35},
                    {'command_smoothing_alpha_angular': 0.30},

                    # Follower speed.
                    {'max_linear_speed': 0.22},
                    {'min_linear_speed_when_far': 0.07},
                    {'max_angular_speed': 1.60},
                    {'linear_gain': 0.75},
                    {'angular_gain': 1.80},

                    # Robot-to-robot safety.
                    {'min_robot_distance_m': 0.50},
                    {'slow_robot_distance_m': 0.85},
                    {'too_close_reverse_speed': -0.02},

                    # Catch-up behavior.
                    {'far_gap_m': 1.45},
                    {'very_far_gap_m': 2.00},
                    {'catchup_speed_boost': 1.50},

                    # Faster startup release.
                    # Old behavior waited for about 95% of spacing.
                    # New behavior starts when predecessor opens 60% of spacing.
                    {'startup_gap_ratio': 0.60},

                    # Lock one stable chain order after formation_ready.
                    {'lock_chain_order': True},


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

                    {'forward_speed': 0.20},
                    {'turn_speed': 0.75},

                    {'front_blocked_distance': 1.10},
                    {'side_clearance_distance': 0.65},
                    {'side_avoid_turn_gain': 0.30},

                    {'spawn_x': 0.0},
                    {'spawn_y': 0.0},

                    {'chain_spacing_m': 1.15},
                    {'formation_tolerance_m': 0.35},

                    {'leader_start_delay_sec': 1.5},
                    {'leader_wait_for_chain': True},

                    {'leader_slow_chain_gap_m': 2.20},
                    {'leader_max_chain_gap_m': 3.00},
                    {'leader_wait_turn_allowed': True},

                    {'preferred_heading_deg': 0.0},
                    {'heading_gain': 0.65},
                    {'max_heading_turn': 0.35},

                    {'obstacle_escape_enabled': True},
                    {'escape_front_clear_distance': 1.80},
                    {'escape_rejoin_heading_error_deg': 20.0},
                    {'escape_min_time_sec': 3.0},
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

                    # Keep zero with ground-truth/world RobotState.
                    {'spawn_x': 0.0},
                    {'spawn_y': 0.0},

                    {'breadcrumb_distance': 1.0},
                    {'marker_topic': '/swarm/breadcrumb_markers_array'},
                ],
            )
        )

        
        # SLAM toolbox — leader only onyl for testing commented out
        if ns == 'robot1':
            actions.append(
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    namespace=ns,
                    name='slam_toolbox',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'mode': 'mapping',
                        'odom_frame': 'world',
                        'base_frame': f'{ns}/chassis',
                        'map_frame': f'{ns}/map',
                        'scan_topic': 'scan',
                        'map_name': 'map',
                        'resolution': 0.08,
                        'minimum_travel_distance': 0.10,
                        'minimum_travel_heading': 0.10,
                    }],
                    remappings=[
                        ('scan', 'scan'),
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

                    {'hard_stop_distance': 0.45},
                    {'slowdown_distance': 0.60},
                    {'side_stop_distance': 0.22},
                    {'side_slow_distance': 0.45},

                    {'wall_avoid_gain': 0.04},
                    {'max_safe_linear_speed': 0.22},
                ],
            )
        )

    # ------------------------------------------------------------
    # Global line monitor
    # ------------------------------------------------------------
    # actions.append(
    #     Node(
    #         package='swarm_control',
    #         executable='line_alignment_monitor',
    #         name='line_alignment_monitor',
    #         output='screen',
    #         parameters=[
    #             {'use_sim_time': True},
    #             {'path_topic': '/swarm/leader_path'},
    #             {'state_topic': '/swarm/robot_states'},
    #             {'chain_order_topic': '/swarm/chain_order'},
    #             {'output_topic': '/swarm/line_alignment'},
    #             {'check_period_sec': 2.0},

    #             {'warn_path_lateral_error_m': 0.12},
    #             {'bad_path_lateral_error_m': 0.22},

    #             {'warn_neighbor_line_error_m': 0.12},
    #             {'bad_neighbor_line_error_m': 0.22},

    #             {'warn_leader_axis_error_m': 0.15},
    #             {'bad_leader_axis_error_m': 0.30},

    #             {'warn_lateral_spread_m': 0.15},
    #             {'bad_lateral_spread_m': 0.28},

    #             {'warn_yaw_error_rad': 0.18},
    #             {'bad_yaw_error_rad': 0.35},

    #             {'ignore_leader_for_path_error': True},
    #         ],
    #     )
    # )

    return LaunchDescription(actions)
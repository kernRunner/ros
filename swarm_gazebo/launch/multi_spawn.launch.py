from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def compact_group(start_id, cx, cy):
    # Spawn robots above uneven terrain so they do not start inside the map.
    # They will fall down onto the collision mesh.
    spawn_z = 0.15

    pts = [
        (0.70, 0.70),
        (0.70, 0.00),
        (0.70, -0.70),

        (0.00, 0.70),
        (0.00, 0.00),
        (0.00, -0.70),

        (-0.70, 0.70),
        (-0.70, 0.00),
        (-0.70, -0.70),
    ]

    return [
        {
            'name': f'robot{start_id + i}',
            'x': str(cx + x),
            'y': str(cy + y),
            'z': str(spawn_z),
        }
        for i, (x, y) in enumerate(pts)
    ]

def later(t, action):
    return TimerAction(period=t, actions=[action])


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('swarm_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_name = 'tree_exploration_test'
    world_file = os.path.join(gazebo_pkg, 'worlds', f'{world_name}.sdf')
    model_file = os.path.join(
        gazebo_pkg,
        'models',
        'swarm_bot_hex.sdf',
    )

    # Single small Gazebo group.
    #
    # We keep the relay-tree, swarm_member, path_follower, and tree_explorer logic.
    # Only the number of robots is reduced.
    group_a = [
        'robot1', 'robot2', 'robot3',
        'robot4', 'robot5', 'robot6',
        'robot7', 'robot8', 'robot9',
    ]

    robots = compact_group(1, 0.0, 0.0)
    active = group_a

    def chain(ns):
        return group_a

    def role_topic(ns):
        return '/swarm/role_assignments_A'

    actions = []

    # ------------------------------------------------------------
    # Start Gazebo
    # ------------------------------------------------------------

    resource_paths = [
        gazebo_pkg,
        os.path.join(gazebo_pkg, 'models'),
        os.path.join(gazebo_pkg, 'worlds'),
    ]

    existing_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing_ign_path:
        resource_paths.append(existing_ign_path)

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        resource_paths.append(existing_gz_path)

    resource_path_value = os.pathsep.join(resource_paths)

    actions.append(SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path_value,
    ))

    actions.append(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path_value,
    ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r --render-engine ogre "{world_file}"'
        }.items(),
    ))

    actions.append(Node(
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
    ))

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_dynamic_pose_tf_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
    ))

    actions.append(Node(
        package='swarm_control',
        executable='gazebo_pose_bridge',
        name='gazebo_pose_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'gazebo_pose_topic': f'/world/{world_name}/dynamic_pose/info'},
            {'robot_names': active},
        ],
    ))

    actions.append(Node(
        package='swarm_control',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'command_topic': '/swarm/mission_command'},
            {'mode_topic': '/swarm/mission_mode'},
            {'publish_rate_hz': 2.0},
            {'default_mode': 'explore'},
        ],
    ))

    common_tree = [
        {'use_sim_time': True},

        # Smaller group, so split earlier than the old 9-robot group.
        # Logic is unchanged; only the demo threshold is reduced.
        {'split_distance_m': 20.0},
        {'branch_angle_deg': 28.0},
        {'publish_rate_hz': 1.0},
        {'single_robot_groups_are_leaders': True},
        {'min_group_size_to_split': 3},
        {'max_branch_depth': 3},
        {'min_group_age_before_split_sec': 8.0},
        {'require_group_stable_before_split': True},
        {'max_pair_gap_for_split_m': 2.5},
        {'min_relay_progress_ratio': 0.80},
        {'relay_selection_mode': 'chain_tail'},
    ]

    actions.append(later(8.0, Node(
        package='swarm_control',
        executable='relay_tree_manager',
        name='relay_tree_manager_A',
        output='screen',
        parameters=[
            {'robot_names': group_a},
            {'initial_leader_name': 'robot1'},
            {'initial_heading_deg': 0.0},
            {'role_assignment_topic': '/swarm/role_assignments_A'},
            {'explicit_chain_order': group_a},
            *common_tree,
        ],
    )))

    actions.append(Node(
        package='swarm_control',
        executable='relay_tree_visualizer',
        name='relay_tree_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'state_topic': '/swarm/robot_states'},
            {'marker_topic': '/swarm/relay_tree_markers'},
            {'frame_id': 'world'},
            {'publish_rate_hz': 5.0},
            {'state_timeout_sec': 4.0},
            {'show_text_labels': True},
            {'text_height': 0.18},
            {'robot_marker_scale': 0.30},
            {'relay_marker_scale': 0.46},
            {'line_width': 0.035},
        ],
    ))

    actions.append(Node(
        package='swarm_control',
        executable='relay_tree_evaluator',
        name='relay_tree_evaluator',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'state_topic': '/swarm/robot_states'},
            {'eval_topic': '/swarm/relay_tree_eval'},
            {'text_topic': '/swarm/relay_tree_eval_text'},
            {'marker_topic': '/swarm/relay_tree_eval_markers'},
            {'frame_id': 'world'},
            {'publish_rate_hz': 1.0},
            {'state_timeout_sec': 4.0},
            {'show_rviz_text': True},
            {'rviz_text_x': -8.0},
            {'rviz_text_y': 8.0},
            {'rviz_text_z': 1.5},
            {'rviz_text_height': 0.30},
            {'max_recent_events_displayed': 8},
            {'max_relay_link_distance_m': 40.0},
        ],
    ))

    for i, r in enumerate(robots):
        ns = r['name']
        d = 1.5 + i * 0.7
        c = chain(ns)
        rt = role_topic(ns)

        actions.append(later(d, Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', ns,
                '-allow_renaming', 'false',
                '-file', model_file,
                '-x', r['x'],
                '-y', r['y'],
                '-z', r['z'],
            ],
        )))
        
        actions.append(later(d + 0.15, Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{ns}_bridge',
            namespace=ns,
            output='screen',
            arguments=[
                f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                f'/model/{ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

                f'/world/{world_name}/model/{ns}/link/chassis/sensor/lidar/scan'
                '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

                f'/world/{world_name}/model/{ns}/link/chassis/sensor/lidar_3d/scan/points'
                '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            remappings=[
                (f'/model/{ns}/cmd_vel', 'cmd_vel'),
                (f'/model/{ns}/odometry', 'odom'),

                (
                    f'/world/{world_name}/model/{ns}/link/chassis/sensor/lidar/scan',
                    'scan_2d',
                ),
                (
                    f'/world/{world_name}/model/{ns}/link/chassis/sensor/lidar_3d/scan/points',
                    'points',
                ),
            ],
        )))

        actions.append(later(d + 0.3, Node(
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
        )))

        actions.append(later(d + 0.45, Node(
            package='swarm_control',
            executable='swarm_member',
            namespace=ns,
            name='swarm_member',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_name': ns},
                {'publish_rate_hz': 1.0},
                {'state_timeout_sec': 5.0},
                {'lock_leader_after_startup': True},
                {'startup_election_delay_sec': 6.0},
                {'use_ground_truth_pose': True},
                {'ground_truth_pose_topic': 'ground_truth_pose'},
                {'use_role_assignments': True},
                {'role_assignment_topic': rt},
                {'spawn_x': float(r['x'])},
                {'spawn_y': float(r['y'])},
            ],
        )))

        actions.append(later(d + 0.6, Node(
            package='swarm_control',
            executable='path_follower',
            namespace=ns,
            name='path_follower',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_name': ns},
                {'cmd_vel_topic': 'cmd_vel'},
                {'mission_command_topic': '/swarm/mission_mode'},

                {'desired_follow_distance_m': 0.95},
                {'follow_deadband_m': 0.25},
                {'hold_gap_deadband_m': 0.22},
                {'hold_heading_deadband_rad': 0.50},

                {'speed_match_enabled': True},
                {'speed_match_gain': 0.85},
                {'min_creep_speed': 0.01},

                {'command_smoothing_alpha_linear': 0.18},
                {'command_smoothing_alpha_angular': 0.12},

                {'max_linear_speed': 0.12},
                {'min_linear_speed_when_far': 0.04},
                {'max_angular_speed': 0.45},
                {'linear_gain': 0.35},
                {'angular_gain': 0.45},

                {'min_robot_distance_m': 0.42},
                {'slow_robot_distance_m': 0.70},
                {'too_close_reverse_speed': -0.015},

                {'far_gap_m': 1.25},
                {'very_far_gap_m': 1.70},
                {'catchup_speed_boost': 1.35},

                {'startup_gap_ratio': 1.05},
                {'lock_chain_order': False},
                {'require_formation_ready': False},

                {'sequential_start_enabled': True},
                {'sequential_slot_delay_sec': 0.35},
                {'sequential_predecessor_move_m': 0.45},
                {'sequential_front_pair_gap_ratio': 0.95},
                {'sequential_front_alignment_tolerance_rad': 0.55},
                {'sequential_self_alignment_tolerance_rad': 0.40},
                {'sequential_prealign_enabled': False},

                {'chain_order_mode': 'explicit'},
                {'explicit_chain_order': c},
            ],
        )))

        actions.append(later(d + 0.65, Node(
            package='swarm_control',
            executable='terrain_scan_filter',
            namespace=ns,
            name='terrain_scan_filter',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_name': ns},
                {'active_only_when_leader': True},
                {'pointcloud_topic': 'points'},
                {'scan_topic': 'scan'},
                {'range_min': 0.25},
                {'range_max': 7.0},
                {'num_ranges': 90},
                {'min_obstacle_height': -0.02},
                {'max_obstacle_height': 0.70},
            ],
        )))

        actions.append(later(d + 0.95, Node(
            package='swarm_control',
            executable='tree_explorer',
            namespace=ns,
            name='tree_explorer',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_name': ns},
                {'cmd_vel_topic': 'cmd_vel'},
                {'mission_command_topic': '/swarm/mission_mode'},
                {'forward_speed': 0.08},
                {'turn_speed': 0.30},
                {'front_blocked_distance': 0.85},
                {'obstacle_escape_enabled': True},
                {'chain_spacing_m': 1.15},
                {'formation_tolerance_m': 0.35},
                {'leader_start_delay_sec': 2.0},
                {'leader_wait_for_chain': False},
                {'leader_slow_chain_gap_m': 2.00},
                {'leader_max_chain_gap_m': 2.80},
                {'leader_wait_turn_allowed': True},
                {'preferred_heading_deg': 0.0},
                {'heading_gain': 0.65},
                {'max_heading_turn': 0.35},
                {'relay_leash_enabled': True},
                {'relay_slow_distance_m': 32.0},
                {'relay_stop_distance_m': 40.0},
                {'relay_stop_turn_allowed': True},
                {'return_home_speed': 0.08},
                {'return_home_arrival_distance_m': 1.00},
                {'return_home_heading_gain': 0.90},
                {'return_home_max_turn': 0.45},
                {'scan_topic': 'scan'},
            ],
        )))

    return LaunchDescription(actions)

# Launches the Gazebo swarm exploration demo.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def compact_group(start_id, cx, cy):
    # Creates a compact 3x3 robot spawn layout.
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
    # Delays startup of an action.
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

    # Initial robot group used by the relay tree.
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
    # Gazebo setup
    # ------------------------------------------------------------

    # Lets Gazebo find worlds and models.
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

    # Starts Gazebo with the selected world.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r --render-engine ogre "{world_file}"'
        }.items(),
    ))

    # Bridges simulation time to ROS.
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

    # Bridges Gazebo model poses.
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_dynamic_pose_tf_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
    ))

    # Converts Gazebo poses into robot pose topics.
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

    # Publishes the current mission mode.
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

    # Shared relay-tree parameters.
    common_tree = [
        {'use_sim_time': True},
        {'split_distance_m': 30.0},
        {'first_split_distance_m': 25.0},
        {'later_split_distance_m': 65.0},

        {'branch_angle_deg': 45.0},
        {'publish_rate_hz': 1.0},
        {'single_robot_groups_are_leaders': True},
        {'min_group_size_to_split': 3},
        {'max_branch_depth': 3},
        {'min_group_age_before_split_sec': 18.0},
        {'require_group_stable_before_split': True},
        {'max_pair_gap_for_split_m': 2.8},
        {'min_relay_progress_ratio': 0.90},
        {'relay_selection_mode': 'chain_tail'},
        {'allow_two_robot_terminal_split': True},
    ]

    # Assigns roles and creates relay-tree splits.
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

    # Publishes RViz markers for the relay tree.
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

    # Publishes relay-tree evaluation/debug data.
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

    # ------------------------------------------------------------
    # Per-robot nodes
    # ------------------------------------------------------------

    for i, r in enumerate(robots):
        ns = r['name']
        d = 1.5 + i * 0.7
        c = chain(ns)
        rt = role_topic(ns)

        # Spawns the robot model in Gazebo.
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

        # Bridges each robot's Gazebo topics to ROS.
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

                f'/world/{world_name}/model/{ns}/link/chassis/sensor/front_camera/image'
                '@sensor_msgs/msg/Image[gz.msgs.Image',

                f'/world/{world_name}/model/{ns}/link/chassis/sensor/front_camera/camera_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
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
                (
                    f'/world/{world_name}/model/{ns}/link/chassis/sensor/front_camera/image',
                    'camera/image_raw',
                ),
                (
                    f'/world/{world_name}/model/{ns}/link/chassis/sensor/front_camera/camera_info',
                    'camera/camera_info',
                ),
            ],
        )))

        # Publishes the 2D lidar frame.
        actions.append(later(d + 0.20, Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{ns}_lidar_tf',
            namespace=ns,
            arguments=[
                '0', '0', '0.085',
                '0', '0', '0',
                f'{ns}/chassis',
                f'{ns}/chassis/lidar',
            ],
        )))

        # Publishes the 3D lidar frame.
        actions.append(later(d + 0.20, Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{ns}_lidar_3d_tf',
            namespace=ns,
            arguments=[
                '0', '0', '0.155',
                '0', '0', '0',
                f'{ns}/chassis',
                f'{ns}/chassis/lidar_3d',
            ],
        )))

        # Publishes the front camera frame.
        actions.append(later(d + 0.20, Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{ns}_front_camera_tf',
            namespace=ns,
            arguments=[
                '0.18', '0', '0.105',
                '0', '0', '0',
                f'{ns}/chassis',
                f'{ns}/chassis/front_camera',
            ],
        )))

        # Publishes robot TF from ground-truth pose.
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

        # Publishes RobotState and applies role assignments.
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

        # Keeps followers spaced in the robot chain.
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

                # Spacing behavior
                {'desired_follow_distance_m': 1.10},
                {'follow_deadband_m': 0.35},
                {'hold_gap_deadband_m': 0.35},
                {'hold_heading_deadband_rad': 0.75},

                # Speed matching
                {'speed_match_enabled': True},
                {'speed_match_gain': 0.70},
                {'min_creep_speed': 0.008},

                # Command smoothing
                {'command_smoothing_alpha_linear': 0.10},
                {'command_smoothing_alpha_angular': 0.05},

                # Motion limits
                {'max_linear_speed': 0.10},
                {'min_linear_speed_when_far': 0.035},
                {'max_angular_speed': 0.26},
                {'linear_gain': 0.28},
                {'angular_gain': 0.28},

                # Catch-up behavior
                {'far_gap_m': 1.40},
                {'very_far_gap_m': 1.90},
                {'catchup_speed_boost': 1.15},

                # Collision safety
                {'min_robot_distance_m': 0.42},
                {'slow_robot_distance_m': 0.70},
                {'too_close_reverse_speed': -0.015},

                # Startup gating
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

                # Chain order
                {'chain_order_mode': 'explicit'},
                {'explicit_chain_order': c},
            ],
        )))

        # Converts 3D lidar points into a filtered obstacle scan.
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

                # Range sampling
                {'range_min': 0.55},
                {'range_max': 7.0},
                {'num_ranges': 90},

                # Obstacle height filter
                {'min_obstacle_height': 0.08},
                {'max_obstacle_height': 1.10},

                # Self-filtering
                {'body_ignore_radius': 0.55},
                {'min_vertical_angle': -0.05},
            ],
        )))

        # Drives leaders during exploration.
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

                # Basic movement
                {'forward_speed': 0.075},
                {'turn_speed': 0.24},
                {'front_blocked_distance': 3.20},
                {'obstacle_escape_enabled': True},

                # Formation startup
                {'chain_spacing_m': 1.15},
                {'formation_tolerance_m': 0.35},
                {'leader_start_delay_sec': 2.0},

                # Leader does not wait for followers.
                {'leader_wait_for_chain': False},
                {'leader_slow_chain_gap_m': 999.0},
                {'leader_max_chain_gap_m': 999.0},
                {'leader_wait_turn_allowed': True},

                # Heading control
                {'preferred_heading_deg': 0.0},
                {'heading_gain': 0.18},
                {'max_heading_turn': 0.24},

                # Leader does not wait for relay distance either.
                # This prevents stopping at relay_stop_distance_m.
                {'relay_leash_enabled': False},
                {'relay_slow_distance_m': 999.0},
                {'relay_stop_distance_m': 999.0},
                {'relay_stop_turn_allowed': True},

                # Return-home behavior
                {'return_home_speed': 0.08},
                {'return_home_arrival_distance_m': 1.00},
                {'return_home_heading_gain': 0.90},
                {'return_home_max_turn': 0.45},

                # Obstacle behavior
                {'scan_topic': 'scan'},
                {'side_clearance_distance': 1.30},
                {'side_avoid_turn_gain': 0.18},
                {'escape_front_clear_distance': 4.00},
                {'escape_min_time_sec': 7.5},
                {'escape_rejoin_heading_error_deg': 25.0},
                {'rejoin_cooldown_sec': 10.0},
            ],
        )))

    # ------------------------------------------------------------
    # Mapping nodes
    # ------------------------------------------------------------

    # Builds a shared 2D lidar map.
    actions.append(Node(
        package='swarm_control',
        executable='swarm_lidar_mapper.py',
        name='swarm_lidar_mapper',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ],
    ))

    # Builds an accumulated 3D point-cloud map.
    actions.append(Node(
        package='swarm_control',
        executable='swarm_3d_cloud_mapper.py',
        name='swarm_3d_cloud_mapper',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_names': active},
            {'cloud_topic_suffix': 'points'},
            {'fixed_frame': 'world'},
            {'map_topic': '/swarm/map_3d'},

            # Bigger = faster/lighter, smaller = more detailed/heavier.
            {'voxel_size': 0.15},

            # Safety caps so RViz does not get overloaded.
            {'max_voxels': 250000},
            {'max_points_per_cloud': 2500},

            # Publish accumulated 3D map once per second.
            {'publish_rate_hz': 1.0},

            # Sensor-frame filtering.
            {'range_min': 0.35},
            {'range_max': 8.0},

            # World-frame height filtering.
            {'world_z_min': -3.0},
            {'world_z_max': 8.0},
        ],
    ))


    # ------------------------------------------------------------
    # World camera bridge disabled for performance.
    # ------------------------------------------------------------

    # actions.append(Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='world_cameras_bridge',
    #     output='screen',
    #     arguments=[
    #         f'/world/{world_name}/model/world_camera_1/link/link/sensor/camera/image'
    #         '@sensor_msgs/msg/Image[gz.msgs.Image',
    #         f'/world/{world_name}/model/world_camera_1/link/link/sensor/camera/camera_info'
    #         '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

    #         f'/world/{world_name}/model/world_camera_2/link/link/sensor/camera/image'
    #         '@sensor_msgs/msg/Image[gz.msgs.Image',
    #         f'/world/{world_name}/model/world_camera_2/link/link/sensor/camera/camera_info'
    #         '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

    #         f'/world/{world_name}/model/world_camera_3/link/link/sensor/camera/image'
    #         '@sensor_msgs/msg/Image[gz.msgs.Image',
    #         f'/world/{world_name}/model/world_camera_3/link/link/sensor/camera/camera_info'
    #         '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    #     ],
    #     remappings=[
    #         (
    #             f'/world/{world_name}/model/world_camera_1/link/link/sensor/camera/image',
    #             '/world_camera_1/image_raw',
    #         ),
    #         (
    #             f'/world/{world_name}/model/world_camera_1/link/link/sensor/camera/camera_info',
    #             '/world_camera_1/camera_info',
    #         ),

    #         (
    #             f'/world/{world_name}/model/world_camera_2/link/link/sensor/camera/image',
    #             '/world_camera_2/image_raw',
    #         ),
    #         (
    #             f'/world/{world_name}/model/world_camera_2/link/link/sensor/camera/camera_info',
    #             '/world_camera_2/camera_info',
    #         ),

    #         (
    #             f'/world/{world_name}/model/world_camera_3/link/link/sensor/camera/image',
    #             '/world_camera_3/image_raw',
    #         ),
    #         (
    #             f'/world/{world_name}/model/world_camera_3/link/link/sensor/camera/camera_info',
    #             '/world_camera_3/camera_info',
    #         ),
    #     ],
    # ))

    return LaunchDescription(actions)
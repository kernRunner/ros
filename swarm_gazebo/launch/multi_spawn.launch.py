from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_pkg     = get_package_share_directory('swarm_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_name = 'tree_exploration_test'
    world_file = os.path.join(gazebo_pkg, 'worlds', f'{world_name}.sdf')
    model_file = os.path.join(gazebo_pkg, 'models', 'swarm_bot.sdf')

    robots = [
        # FRONT ROW — leader elected from here
        {'name': 'robot1', 'x': '1.2', 'y':  '0.6', 'z': '0.0'},
        {'name': 'robot2', 'x': '1.2', 'y': '-0.6', 'z': '0.0'},
        # SECOND ROW
        {'name': 'robot3', 'x': '0.0', 'y':  '0.6', 'z': '0.0'},
        {'name': 'robot4', 'x': '0.0', 'y': '-0.6', 'z': '0.0'},
        # {'name': 'robot5', 'x': '-1.2', 'y':  '0.6', 'z': '0.0'},
        # {'name': 'robot6', 'x': '-1.2', 'y': '-0.6', 'z': '0.0'},
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
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        )
    )

    actions.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_robot1_map',
            arguments=[
                '--x', '0.0', '--y', '-9.0', '--z', '0.0',
                '--yaw', '-0.045',
                '--frame-id', 'world',
                '--child-frame-id', 'robot1/map',
            ]
        )
    )

    for robot in robots:
        ns       = robot['name']
        is_active = ns in active_robots
        spawn_x  = float(robot['x'])
        spawn_y  = float(robot['y'])

        # Spawn in Gazebo
        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', ns, '-allow_renaming', 'false',
                    '-file', model_file,
                    '-x', robot['x'], '-y', robot['y'], '-z', robot['z'],
                ]
            )
        )

        # world -> <ns>/odom static TF
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'world_to_{ns}_odom',
                arguments=[
                    '--x', robot['x'], '--y', robot['y'], '--z', '0.0',
                    '--yaw', '0.0',
                    '--frame-id', 'world',
                    '--child-frame-id', f'{ns}/odom',
                ]
            )
        )

        # ROS <-> Gazebo bridge
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
                    (f'/world/{world_name}/model/{ns}/link/chassis/sensor/lidar/scan', 'scan'),
                ]
            )
        )

        if not is_active:
            continue

        # odom -> TF
        actions.append(
            Node(
                package='swarm_control',
                executable='odom_to_tf',
                namespace=ns, name='odom_to_tf', output='screen',
                parameters=[
                    {'use_sim_time': True}, {'robot_name': ns},
                    {'odom_topic': 'odom'},
                    {'base_frame': f'{ns}/chassis'},
                    {'odom_frame': f'{ns}/odom'},
                ]
            )
        )

        # chassis -> lidar static TF
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{ns}_lidar_tf',
                arguments=[
                    '--x', '0.10', '--y', '0.0', '--z', '0.12',
                    '--yaw', '0.0',
                    '--frame-id', f'{ns}/chassis',
                    '--child-frame-id', f'{ns}/chassis/lidar',
                ]
            )
        )

        # Swarm member (state + election)
        actions.append(
            Node(
                package='swarm_control',
                executable='swarm_member',
                namespace=ns, name='swarm_member', output='screen',
                parameters=[
                    {'use_sim_time': True}, {'robot_name': ns},
                    {'publish_rate_hz': 5.0},
                    {'state_timeout_sec': 1.5},
                    {'lock_leader_after_startup': True},
                    {'startup_election_delay_sec': 2.0},
                    {'spawn_x': spawn_x}, {'spawn_y': spawn_y},
                ]
            )
        )

        # Leader path publisher
        actions.append(
            Node(
                package='swarm_control',
                executable='leader_path_publisher',
                namespace=ns, name='leader_path_publisher', output='screen',
                parameters=[
                    {'use_sim_time': True}, {'robot_name': ns},
                    {'path_topic': '/swarm/leader_path'},
                    {'append_distance': 0.06},
                    {'append_yaw_delta': 0.14},
                    {'max_path_points': 2500},
                    {'frame_id': 'world'},
                    {'spawn_x': spawn_x}, {'spawn_y': spawn_y},
                ]
            )
        )

        # ---------------------------------------------------------------
        # Formation manager — drives robot to its chain slot at startup.
        # Publishes /<ns>/formation_ready (Bool).
        # Slot = position in chain, computed from elected leader at runtime.
        # ---------------------------------------------------------------
        actions.append(
            Node(
                package='swarm_control',
                executable='formation_manager',
                namespace=ns, name='formation_manager', output='screen',
                parameters=[
                    {'use_sim_time': True}, {'robot_name': ns},
                    {'spawn_x': spawn_x}, {'spawn_y': spawn_y},
                    {'slot_spacing_m': 1.8},
                    {'arrival_tolerance_m': 0.20},
                    {'max_linear_speed': 0.10},
                    {'max_angular_speed': 0.75},
                    {'linear_gain': 0.50},
                    {'angular_gain': 1.40},
                    # Time to wait after election before first robot moves.
                    # Needs to be > startup_election_delay_sec in swarm_member.
                    {'post_election_wait_sec': 1.5},
                ]
            )
        )

        # ---------------------------------------------------------------
        # Path follower — idles until /<ns>/formation_ready == True
        # ---------------------------------------------------------------
        actions.append(
            Node(
                package='swarm_control',
                executable='path_follower',
                namespace=ns, name='path_follower', output='screen',
                parameters=[
                    {'use_sim_time': True}, {'robot_name': ns},
                    {'cmd_vel_topic': 'cmd_vel_raw'},
                    {'path_topic': '/swarm/leader_path'},
                    {'spawn_x': spawn_x}, {'spawn_y': spawn_y},
                    {'chain_spacing_m': 1.8},
                    {'chain_stop_distance_m': 0.65},
                    {'chain_slow_distance_m': 1.10},
                    {'chain_missing_speed_scale': 0.5},
                    {'goal_tolerance_m': 0.08},
                    {'max_linear_speed': 0.12},
                    {'max_angular_speed': 0.80},
                    {'linear_gain': 0.55},
                    {'angular_gain': 1.50},
                    {'lookahead_m': 0.0},
                    {'startup_delay_per_slot_sec': 1.5},
                ]
            )
        )

        # Tree explorer
        actions.append(
            Node(
                package='swarm_control',
                executable='tree_explorer',
                namespace=ns, name='tree_explorer', output='screen',
                parameters=[
                    {'use_sim_time': True}, {'robot_name': ns},
                    {'cmd_vel_topic': 'cmd_vel_raw'},
                    {'forward_speed': 0.18}, {'turn_speed': 0.75},
                    {'front_blocked_distance': 0.85},
                    {'spawn_x': spawn_x}, {'spawn_y': spawn_y},
                    {'chain_spacing_m': 1.8},
                    {'formation_tolerance_m': 0.35},
                ]
            )
        )

        # Breadcrumb manager
        actions.append(
            Node(
                package='swarm_control',
                executable='breadcrumb_manager',
                namespace=ns, name='breadcrumb_manager', output='screen',
                parameters=[
                    {'use_sim_time': True}, {'robot_name': ns},
                    {'frame_id': 'world'},
                    {'spawn_x': spawn_x}, {'spawn_y': spawn_y},
                    {'breadcrumb_distance': 1.0},
                    {'marker_topic': '/swarm/breadcrumb_markers_array'},
                ]
            )
        )

        # Safety filter
        actions.append(
            Node(
                package='swarm_control',
                executable='cmd_vel_safety_filter',
                namespace=ns, name='cmd_vel_safety_filter', output='screen',
                parameters=[
                    {'use_sim_time': True}, {'robot_name': ns},
                    {'enabled': True},
                    {'hard_stop_distance': 0.20},
                    {'slowdown_distance': 0.35},
                    {'side_stop_distance': 0.18},
                    {'side_slow_distance': 0.35},
                    {'max_safe_linear_speed': 0.10},
                    {'wall_avoid_gain': 0.03},
                ]
            )
        )

        # SLAM toolbox — leader only
        if ns == 'robot1':
            actions.append(
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    namespace=ns, name='slam_toolbox', output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'mode': 'mapping',
                        'transform_publish_period': 0.1,
                        'odom_frame': f'{ns}/odom',
                        'base_frame': f'{ns}/chassis',
                        'map_frame': f'{ns}/map',
                        'scan_topic': 'scan',
                        'map_name': 'map',
                        'resolution': 0.08,
                        'minimum_travel_distance': 0.10,
                        'minimum_travel_heading': 0.10,
                    }],
                    remappings=[('scan', 'scan')]
                )
            )

    return LaunchDescription(actions)

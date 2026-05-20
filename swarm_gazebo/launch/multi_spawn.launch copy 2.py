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
        {'name': 'robot1', 'x': '1.2', 'y':  '0.6',  'z': '0.0'},
        {'name': 'robot2', 'x': '1.2', 'y': '-0.6',  'z': '0.0'},
        {'name': 'robot3', 'x': '0.0', 'y':  '0.6',  'z': '0.0'},
        {'name': 'robot4', 'x': '0.0', 'y': '-0.6',  'z': '0.0'},
    ]

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

    for robot in robots:
        ns = robot['name']
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
                ]
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
                        'scan'
                    ),
                ]
            )
        )

        # actions.append(
        #     Node(
        #         package='swarm_control',
        #         executable='cmd_vel_safety_filter',
        #         namespace=ns,
        #         name='cmd_vel_safety_filter',
        #         output='screen',
        #         parameters=[
        #             {'use_sim_time': True},
        #             {'robot_name': ns},
        #             {'enabled': True},
        #             {'hard_stop_distance': 0.20},
        #             {'slowdown_distance': 0.35},
        #             {'side_stop_distance': 0.18},
        #             {'side_slow_distance': 0.35},
        #             {'max_safe_linear_speed': 0.12},
        #             {'wall_avoid_gain': 0.03},
        #         ]
        #     )
        # )

        if ns != 'robot1':
            actions.append(
                Node(
                    package='swarm_control',
                    executable='simple_chain_follower',
                    namespace=ns,
                    name='simple_chain_follower',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'robot_name': ns},
                        {'leader_name': 'robot1'},
                        {'cmd_vel_topic': 'cmd_vel_raw'},
                        {'backup_cmd_vel_topic': 'cmd_vel_raw'},
                        {'spawn_x': spawn_x},
                        {'spawn_y': spawn_y},
                        {'leader_spawn_x': 1.2},
                        {'leader_spawn_y': 0.6},
                        {'spacing': 1.8},
                        {'max_linear': 0.10},
                        {'max_angular': 0.80},
                    ]
                )
            )

    return LaunchDescription(actions)
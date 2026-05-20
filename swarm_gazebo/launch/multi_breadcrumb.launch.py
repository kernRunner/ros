from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('swarm_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'empty.sdf')
    model_files = {
        'robot1': os.path.join(gazebo_pkg, 'models', 'swarm_bot_robot1.sdf'),
        'robot2': os.path.join(gazebo_pkg, 'models', 'swarm_bot_robot2.sdf'),
        'robot3': os.path.join(gazebo_pkg, 'models', 'swarm_bot_robot3.sdf'),
    }

    robots = [
        {'name': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.2'},
        {'name': 'robot2', 'x': '-0.9', 'y': '0.0', 'z': '0.2'},
        {'name': 'robot3', 'x': '-1.8', 'y': '0.0', 'z': '0.2'},
    ]

    actions = [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
        )
    ]

    for robot in robots:
        ns = robot['name']

        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', ns,
                    '-allow_renaming', 'false',
                    '-file', model_files[ns],
                    '-x', robot['x'],
                    '-y', robot['y'],
                    '-z', robot['z']
                ]
            )
        )

        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=ns,
                output='screen',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    f'/world/empty/model/{ns}/link/chassis/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/model/{ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                ]
            )
        )

    actions.append(
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='swarm_control',
                    executable='avoid_obstacles',
                    name='robot1_avoid',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'robot_name': 'robot1'}
                    ]
                )
            ]
        )
    )

    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='swarm_control',
                    executable='follower_controller',
                    name='robot2_follower',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'robot_name': 'robot2'},
                        {'leader_name': 'robot1'},
                        {'follow_distance': 0.9},
                        {'max_linear_speed': 0.07},
                        {'max_angular_speed': 0.55},
                    ]
                )
            ]
        )
    )

    return LaunchDescription(actions)
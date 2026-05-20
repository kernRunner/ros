# outdated


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('swarm_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'empty.sdf')
    model_file = os.path.join(gazebo_pkg, 'models', 'swarm_bot.sdf')
    nav2_params = os.path.join(gazebo_pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}'
            }.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', 'robot1',
                '-file', model_file,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.2'
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/robot1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/robot1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/world/empty/model/robot1/link/chassis/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ]
        ),

        Node(
            package='swarm_control',
            executable='odom_to_tf',
            name='robot1_odom_tf',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_name': 'robot1'}
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.10',
                '--y', '0.0',
                '--z', '0.12',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'robot1/chassis',
                '--child-frame-id', 'robot1/chassis/lidar'
            ]
        ),

        Node(
            package='swarm_control',
            executable='cmd_vel_relay',
            name='robot1_cmd_vel_relay',
            output='screen',
            parameters=[
                {'input_topic': '/cmd_vel'},
                {'output_topic': '/model/robot1/cmd_vel'}
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'False',
                'map': '/home/marco/my_map.yaml',
                'use_sim_time': 'True',
                'autostart': 'True',
                'use_composition': 'False',
                'params_file': nav2_params
            }.items()
        ),
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_xacro = PathJoinSubstitution([
        FindPackageShare('swarm_pkg'),
        'urdf',
        'turtlebot3_burger_multi.urdf.xacro'
    ])

    robot1_description = Command([
        'xacro ', robot_xacro, ' namespace:=robot1'
    ])

    robot2_description = Command([
        'xacro ', robot_xacro, ' namespace:=robot2'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot1',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot1_description
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot2',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot2_description
            }]
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'robot1',
                '-topic', '/robot1/robot_description',
                '-robot_namespace', 'robot1',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.01'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'robot2',
                '-topic', '/robot2/robot_description',
                '-robot_namespace', 'robot2',
                '-x', '1.0',
                '-y', '0.0',
                '-z', '0.01'
            ],
            output='screen'
        ),
    ])
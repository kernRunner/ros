from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robots = ['robot1', 'robot2', 'robot3']

    actions = []

    for robot in robots:
        actions.append(
            Node(
                package='swarm_control',
                executable='avoid_obstacles',
                name=f'{robot}_avoid_obstacles',
                output='screen',
                parameters=[{'robot_name': robot}]
            )
        )

    return LaunchDescription(actions)
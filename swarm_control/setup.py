from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'swarm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.todo',
    description='Simple swarm control nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_safety_filter = swarm_control.control.cmd_vel_safety_filter:main',
            'path_follower = swarm_control.control.path_follower:main',
            'ground_truth_to_tf = swarm_control.control.ground_truth_to_tf:main',
            
            'swarm_member = swarm_control.swarm.swarm_member:main',
            'formation_manager = swarm_control.swarm.formation_manager:main',
            'leader_path_publisher = swarm_control.swarm.leader_path_publisher:main',
            'relay_tree_manager = swarm_control.swarm.relay_tree_manager:main',
            'relay_tree_visualizer = swarm_control.swarm.relay_tree_visualizer:main',
            'relay_tree_evaluator = swarm_control.swarm.relay_tree_evaluator:main',
            'mission_controller = swarm_control.coordination.mission_controller:main',

            'tree_explorer = swarm_control.exploration.tree_explorer:main',
            'breadcrumb_manager = swarm_control.exploration.breadcrumb_manager:main',
            'line_alignment_monitor = swarm_control.swarm.line_alignment_monitor:main',
            'gazebo_pose_bridge = swarm_control.control.gazebo_pose_bridge:main',
        
        ],
    },
)
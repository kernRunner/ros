# Installs the swarm_control Python package and registers its ROS 2 nodes.

from setuptools import setup, find_packages
import os
from glob import glob


package_name = 'swarm_control'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),

    # Files installed into the ROS 2 share directory.
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
    maintainer_email='marco.huber@hof-university.de',
    description='Simple swarm control nodes',
    license='Apache-2.0',
    tests_require=['pytest'],

    # ROS 2 command names mapped to Python main() functions.
    entry_points={
        'console_scripts': [
            # Control nodes
            'cmd_vel_safety_filter = swarm_control.control.cmd_vel_safety_filter:main',
            'path_follower = swarm_control.control.path_follower:main',
            'ground_truth_to_tf = swarm_control.control.ground_truth_to_tf:main',
            'gazebo_pose_bridge = swarm_control.control.gazebo_pose_bridge:main',

            # Swarm state and formation nodes
            'swarm_member = swarm_control.swarm.swarm_member:main',
            # Not in use anymore 'formation_manager = swarm_control.swarm.formation_manager:main',
            # Not in use anymore 'leader_path_publisher = swarm_control.swarm.leader_path_publisher:main',
            # Not in use anymore 'line_alignment_monitor = swarm_control.swarm.line_alignment_monitor:main',

            # Relay-tree nodes
            'relay_tree_manager = swarm_control.swarm.relay_tree_manager:main',
            'relay_tree_visualizer = swarm_control.swarm.relay_tree_visualizer:main',
            'relay_tree_evaluator = swarm_control.swarm.relay_tree_evaluator:main',

            # Coordination nodes
            'mission_controller = swarm_control.coordination.mission_controller:main',
            'mock_single_root_four_branches_splitting = swarm_control.coordination.mock_single_root_four_branches_splitting:main',

            # Exploration nodes
            'tree_explorer = swarm_control.exploration.tree_explorer:main',
            'terrain_scan_filter = swarm_control.exploration.terrain_scan_filter:main',
            # Not in use anymore 'breadcrumb_manager = swarm_control.exploration.breadcrumb_manager:main',

            # Mapping nodes
            'swarm_lidar_mapper.py = swarm_control.mapping.swarm_lidar_mapper:main',
            'swarm_3d_cloud_mapper.py = swarm_control.mapping.swarm_3d_cloud_mapper:main',
        ],
    },
)
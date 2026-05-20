from setuptools import find_packages, setup

package_name = 'swarm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/swarm_pkg/launch', [
            'launch/multi_robot.launch.py',
        ]),
        ('share/swarm_pkg/urdf', [
            'urdf/turtlebot3_burger_multi.urdf.xacro',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco',
    maintainer_email='marco@todo.todo',
    description='Swarm robotics package',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [],
    },
)
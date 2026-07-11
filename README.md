# Swarm Exploration with Relay-Tree Coordination

## Overview

This project is a ROS 2 and Gazebo simulation of a multi-robot swarm exploring a Mars-like environment.

The swarm is coordinated using a relay-tree structure. Robots are assigned different roles, such as root relay, relay, group leader, or follower. This allows the swarm to spread into different exploration branches while keeping the overall structure organized and visible.

The project combines robot control, swarm coordination, mapping, and visualization. Gazebo is used for the simulation environment, and RViz is used to display the swarm structure, robot roles, relay links, and mapping results.

## Project Goals

The main goals of this project are:

* Simulate a multi-robot swarm in a Mars-like environment.
* Coordinate robots using a relay-tree structure.
* Assign and update robot roles during exploration.
* Keep follower robots connected to their assigned leader or predecessor.
* Visualize the swarm structure and relay connections in RViz.
* Generate shared 2D and 3D map representations from sensor data.

## Project Structure

The project is divided into three main ROS 2 packages.

### `swarm_control`

This package contains the main swarm logic. It includes the nodes for robot movement, swarm state publishing, relay-tree management, exploration behavior, mapping, and visualization.

The most important files in this package are:

* `swarm_explore.launch.py` — starts the main swarm exploration system.
* `path_follower.py` — controls how follower robots move behind their assigned leader or predecessor.
* `relay_tree_manager.py` — manages the relay-tree structure and assigns robot roles.
* `swarm_member.py` — publishes each robot’s current state to the swarm.

### `swarm_gazebo`

This package contains the simulation environment. It includes the Gazebo world, robot models, terrain models, RViz configuration, and simulation launch files.

This package is responsible for starting and visualizing the simulated environment.

### `swarm_interfaces`

This package contains the custom ROS 2 message definitions used by the swarm.

The main message is `RobotState.msg`, which defines the shared state information for each robot, including position, speed, role, leader ID, group ID, relay information, and branch information.

## System Overview

Each robot publishes its current state to the swarm. The relay-tree manager uses these states to decide which robots should act as relays, group leaders, or followers. The path follower then uses this role information to move each robot according to its assigned position in the swarm.

The system also processes sensor data for mapping and visualization. A 2D occupancy map and a 3D point cloud map can be generated from the robot sensors. RViz is used to show the robot roles, relay-tree links, and the current swarm organization.

## Main ROS Topics

The most important topics are:

* `/swarm/robot_states` — shared state information for all robots.
* `/swarm/role_assignments` — role assignments created by the relay-tree manager.
* `/swarm/mission_command` — input topic for mission commands.
* `/swarm/mission_mode` — current mission mode of the swarm.
* `/swarm/map` — shared 2D occupancy map.
* `/swarm/map_3d` — shared 3D point cloud map.
* `/swarm/relay_tree_markers` — RViz markers for the relay-tree visualization.

## How to Run

After building the workspace and sourcing the setup file, the main swarm exploration system can be started with:

```bash
ros2 launch swarm_gazebo multi_spawn.launch.py
```

An RViz-only demo is also included. This demo does not start Gazebo. It is useful for testing the relay-tree visualization and branch splitting behavior:

```bash
ros2 launch swarm_gazebo rviz_single_root_four_branches_splitting_v2.launch.py
```

Mission commands can be sent through the mission command topic. For example:

```bash
ros2 topic pub --once /swarm/mission_command std_msgs/msg/String "{data: explore}"
```

```bash
ros2 topic pub --once /swarm/mission_command std_msgs/msg/String "{data: stop}"
```

```bash
ros2 topic pub --once /swarm/mission_command std_msgs/msg/String "{data: return_home}"
```


## Notes

This project is designed for simulation and testing. Some parameters are tuned for demonstration in Gazebo and RViz rather than for real-world deployment.

The RViz-only demo is separate from the full Gazebo simulation and is mainly used to test the relay-tree structure and visualization.

## Summary

This project demonstrates how a robot swarm can be organized with a relay-tree structure during exploration. The system combines swarm role assignment, path following, mapping, and visualization to support structured multi-robot exploration in a simulated Mars-like environment.

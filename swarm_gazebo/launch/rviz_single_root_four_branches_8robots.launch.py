from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # RViz-only, no Gazebo:
    #
    # 4 groups * 8 robots + 1 shared root = 33 robots.
    #
    # Shared root:
    #   robot33
    #
    # Initial groups:
    #   A: robot1  ... robot8    heading -90
    #   B: robot9  ... robot16   heading 180
    #   C: robot17 ... robot24   heading 0
    #   D: robot25 ... robot32   heading 90

    heading_a_deg = -90.0
    heading_b_deg = 180.0
    heading_c_deg = 0.0
    heading_d_deg = 90.0

    rviz_config = os.path.join(
        get_package_share_directory('swarm_gazebo'),
        'rviz',
        'relay_tree_18_groups.rviz',
    )

    actions = []

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    ))

    actions.append(Node(
        package='swarm_control',
        executable='mock_single_root_four_branches_8robots',
        name='mock_single_root_four_branches_8robots',
        output='screen',
        parameters=[
            {'state_topic': '/swarm/robot_states'},
            {'publish_rate_hz': 10.0},

            {'forward_speed': 0.08},
            {'follow_speed': 0.13},
            {'follow_distance_m': 0.95},

            {'initial_chain_spacing_m': 0.75},
            {'initial_lateral_spacing_m': 0.26},

            {'heading_a_deg': heading_a_deg},
            {'heading_b_deg': heading_b_deg},
            {'heading_c_deg': heading_c_deg},
            {'heading_d_deg': heading_d_deg},

            {'root_x': 0.0},
            {'root_y': 0.0},

            {'group_a_center_x': 0.0},
            {'group_a_center_y': -2.2},

            {'group_b_center_x': -2.2},
            {'group_b_center_y': 0.0},

            {'group_c_center_x': 2.2},
            {'group_c_center_y': 0.0},

            {'group_d_center_x': 0.0},
            {'group_d_center_y': 2.2},

            # Recursive split behavior.
            {'split_distance_m': 7.0},
            {'branch_angle_deg': 35.0},
            {'min_group_size_to_split': 3},
            {'max_branch_depth': 3},
            {'min_group_age_before_split_sec': 4.0},
        ],
    ))

    actions.append(Node(
        package='swarm_control',
        executable='relay_tree_visualizer',
        name='relay_tree_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'state_topic': '/swarm/robot_states'},
            {'marker_topic': '/swarm/relay_tree_markers'},
            {'frame_id': 'world'},
            {'publish_rate_hz': 5.0},
            {'state_timeout_sec': 4.0},
            {'show_text_labels': True},
            {'text_height': 0.15},
            {'robot_marker_scale': 0.24},
            {'relay_marker_scale': 0.40},
            {'line_width': 0.030},
        ],
    ))

    actions.append(Node(
        package='swarm_control',
        executable='relay_tree_evaluator',
        name='relay_tree_evaluator',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'state_topic': '/swarm/robot_states'},
            {'eval_topic': '/swarm/relay_tree_eval'},
            {'text_topic': '/swarm/relay_tree_eval_text'},
            {'marker_topic': '/swarm/relay_tree_eval_markers'},
            {'frame_id': 'world'},
            {'publish_rate_hz': 2.0},
            {'state_timeout_sec': 4.0},
            {'show_rviz_text': True},
            {'rviz_text_x': -10.0},
            {'rviz_text_y': 9.0},
            {'rviz_text_z': 1.5},
            {'rviz_text_height': 0.22},
            {'max_recent_events_displayed': 10},
            {'max_relay_link_distance_m': 40.0},
        ],
    ))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    ))

    return LaunchDescription(actions)

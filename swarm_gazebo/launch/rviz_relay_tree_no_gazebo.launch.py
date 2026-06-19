from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def later(t, action):
    return TimerAction(period=t, actions=[action])


def generate_launch_description():
    # 18 robots split into 3 relay trees of 6 robots each.
    #
    # Your headings are kept:
    #   Group A: -90 deg
    #   Group B: 180 deg
    #   Group C: 0 deg
    #
    # The mock simulator now also initializes each group facing that heading.

    group_a = [
        'robot1', 'robot2', 'robot3',
        'robot6', 'robot5', 'robot4',
    ]

    group_b = [
        'robot7', 'robot8', 'robot9',
        'robot12', 'robot11', 'robot10',
    ]

    group_c = [
        'robot13', 'robot14', 'robot15',
        'robot18', 'robot17', 'robot16',
    ]

    heading_a_deg = -90.0
    heading_b_deg = 180.0
    heading_c_deg = 0.0

    common_tree = [
        {'use_sim_time': False},

        # Smaller groups should split earlier than the old 9-robot groups.
        {'split_distance_m': 10.0},
        {'branch_angle_deg': 28.0},

        {'publish_rate_hz': 2.0},
        {'single_robot_groups_are_leaders': True},
        {'min_group_size_to_split': 3},
        {'max_branch_depth': 2},
        {'min_group_age_before_split_sec': 8.0},
        {'require_group_stable_before_split': False},
        {'max_pair_gap_for_split_m': 3.5},
        {'min_relay_progress_ratio': 0.80},
        {'relay_selection_mode': 'chain_tail'},
    ]

    rviz_config = os.path.join(
        get_package_share_directory('swarm_gazebo'),
        'rviz',
        'relay_tree_18_groups.rviz',
    )

    actions = []

    # RViz fixed frame should be "map".
    # Markers are published in "world", and this transform connects map -> world.
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    ))

    actions.append(Node(
        package='swarm_control',
        executable='mock_relay_tree_simulator',
        name='mock_relay_tree_simulator',
        output='screen',
        parameters=[
            {'state_topic': '/swarm/robot_states'},
            {'role_assignment_topic_a': '/swarm/role_assignments_A'},
            {'role_assignment_topic_b': '/swarm/role_assignments_B'},
            {'role_assignment_topic_c': '/swarm/role_assignments_C'},

            {'publish_rate_hz': 10.0},
            {'forward_speed': 0.08},
            {'follow_speed': 0.13},
            {'follow_distance_m': 0.95},

            # Compact initial chain.
            {'initial_chain_spacing_m': 0.82},
            {'initial_lateral_spacing_m': 0.30},

            # Groups closer together at startup.
            # A starts slightly below center.
            # B starts upper-left.
            # C starts upper-right.
            {'group_a_center_x': 0.0},
            {'group_a_center_y': -1.8},
            {'group_a_heading_deg': heading_a_deg},

            {'group_b_center_x': -1.8},
            {'group_b_center_y': 1.2},
            {'group_b_heading_deg': heading_b_deg},

            {'group_c_center_x': 1.8},
            {'group_c_center_y': 1.2},
            {'group_c_heading_deg': heading_c_deg},
        ],
    ))

    actions.append(later(1.0, Node(
        package='swarm_control',
        executable='relay_tree_manager',
        name='relay_tree_manager_A',
        output='screen',
        parameters=[
            {'robot_names': group_a},
            {'initial_leader_name': 'robot1'},
            {'initial_heading_deg': heading_a_deg},
            {'role_assignment_topic': '/swarm/role_assignments_A'},
            {'explicit_chain_order': group_a},
            *common_tree,
        ],
    )))

    actions.append(later(1.0, Node(
        package='swarm_control',
        executable='relay_tree_manager',
        name='relay_tree_manager_B',
        output='screen',
        parameters=[
            {'robot_names': group_b},
            {'initial_leader_name': 'robot7'},
            {'initial_heading_deg': heading_b_deg},
            {'role_assignment_topic': '/swarm/role_assignments_B'},
            {'explicit_chain_order': group_b},
            *common_tree,
        ],
    )))

    actions.append(later(1.0, Node(
        package='swarm_control',
        executable='relay_tree_manager',
        name='relay_tree_manager_C',
        output='screen',
        parameters=[
            {'robot_names': group_c},
            {'initial_leader_name': 'robot13'},
            {'initial_heading_deg': heading_c_deg},
            {'role_assignment_topic': '/swarm/role_assignments_C'},
            {'explicit_chain_order': group_c},
            *common_tree,
        ],
    )))

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
            {'text_height': 0.18},
            {'robot_marker_scale': 0.30},
            {'relay_marker_scale': 0.46},
            {'line_width': 0.035},
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
            {'rviz_text_x': -8.0},
            {'rviz_text_y': 7.0},
            {'rviz_text_z': 1.5},
            {'rviz_text_height': 0.24},
            {'max_recent_events_displayed': 8},
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

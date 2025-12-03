import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ===== FGM 파라미터 인자 =====
    fgm_gap_threshold_arg = DeclareLaunchArgument(
        'fgm_gap_threshold', default_value='1.2',
        description='FGM: gap 최소 폭 (m)'
    )
    fgm_bubble_radius_arg = DeclareLaunchArgument(
        'fgm_bubble_radius', default_value='0.5',
        description='FGM: 장애물 주위 버블 반경 (m)'
    )
    fgm_fov_angle_arg = DeclareLaunchArgument(
        'fgm_fov_angle', default_value='180.0',
        description='FGM: 전방 시야 각도 (deg)'
    )

    fgm_min_plan_arg = DeclareLaunchArgument(
        'fgm_min_planning_dist', default_value='2.0'
    )
    fgm_max_plan_arg = DeclareLaunchArgument(
        'fgm_max_planning_dist', default_value='5.0'
    )
    fgm_plan_gain_arg = DeclareLaunchArgument(
        'fgm_planning_gain', default_value='1.0'
    )

    fgm_min_look_arg = DeclareLaunchArgument(
        'fgm_min_lookahead', default_value='1.5'
    )
    fgm_max_look_arg = DeclareLaunchArgument(
        'fgm_max_lookahead', default_value='3.5'
    )
    fgm_look_gain_arg = DeclareLaunchArgument(
        'fgm_lookahead_gain', default_value='0.6'
    )

    fgm_max_speed_arg = DeclareLaunchArgument(
        'fgm_max_speed', default_value='4.0',
        description='FGM 내부 속도 상한'
    )
    fgm_min_speed_arg = DeclareLaunchArgument(
        'fgm_min_speed', default_value='2.0'
    )
    fgm_slow_down_arg = DeclareLaunchArgument(
        'fgm_slow_down_dist', default_value='2.5',
        description='감속 시작 거리 (m)'
    )

    # ===== Pure Pursuit 파라미터 인자 =====
    pp_csv_path_arg = DeclareLaunchArgument(
        'pp_csv_path', default_value='raceline_with_speed.csv',
        description='CSV 기반 주행시 사용할 경로 파일'
    )
    pp_lookahead_min_arg = DeclareLaunchArgument(
        'pp_lookahead_min', default_value='1.0'
    )
    pp_lookahead_gain_arg = DeclareLaunchArgument(
        'pp_lookahead_gain', default_value='0.3'
    )
    pp_wheelbase_arg = DeclareLaunchArgument(
        'pp_wheelbase', default_value='0.33'
    )
    pp_max_speed_arg = DeclareLaunchArgument(
        'pp_max_speed', default_value='4.0'
    )
    pp_use_frenet_path_arg = DeclareLaunchArgument(
        'pp_use_frenet_path', default_value='true'
    )
    pp_frenet_topic_arg = DeclareLaunchArgument(
        'pp_frenet_path_topic', default_value='/fgm_path'
    )

    # ===== FGM Node =====
    fgm_node = Node(
        package='f1tenth_planner',
        executable='fgm_node',
        name='fgm_node',
        output='screen',
        parameters=[{
            'gap_threshold':       LaunchConfiguration('fgm_gap_threshold'),
            'bubble_radius':       LaunchConfiguration('fgm_bubble_radius'),
            'fov_angle':           LaunchConfiguration('fgm_fov_angle'),

            'min_planning_dist':   LaunchConfiguration('fgm_min_planning_dist'),
            'max_planning_dist':   LaunchConfiguration('fgm_max_planning_dist'),
            'planning_gain':       LaunchConfiguration('fgm_planning_gain'),

            'min_lookahead':       LaunchConfiguration('fgm_min_lookahead'),
            'max_lookahead':       LaunchConfiguration('fgm_max_lookahead'),
            'lookahead_gain':      LaunchConfiguration('fgm_lookahead_gain'),

            'max_speed':           LaunchConfiguration('fgm_max_speed'),
            'min_speed':           LaunchConfiguration('fgm_min_speed'),
            'slow_down_dist':      LaunchConfiguration('fgm_slow_down_dist'),
        }]
    )

    # ===== Pure Pursuit Node =====
    pp_node = Node(
        package='f1tenth_planner',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'csv_path':        LaunchConfiguration('pp_csv_path'),
            'lookahead_min':   LaunchConfiguration('pp_lookahead_min'),
            'lookahead_gain':  LaunchConfiguration('pp_lookahead_gain'),
            'wheelbase':       LaunchConfiguration('pp_wheelbase'),
            'max_speed':       LaunchConfiguration('pp_max_speed'),
            'use_frenet_path': LaunchConfiguration('pp_use_frenet_path'),
            'frenet_path_topic': LaunchConfiguration('pp_frenet_path_topic'),
        }]
    )

    return LaunchDescription([
        # Args
        fgm_gap_threshold_arg,
        fgm_bubble_radius_arg,
        fgm_fov_angle_arg,
        fgm_min_plan_arg,
        fgm_max_plan_arg,
        fgm_plan_gain_arg,
        fgm_min_look_arg,
        fgm_max_look_arg,
        fgm_look_gain_arg,
        fgm_max_speed_arg,
        fgm_min_speed_arg,
        fgm_slow_down_arg,
        pp_csv_path_arg,
        pp_lookahead_min_arg,
        pp_lookahead_gain_arg,
        pp_wheelbase_arg,
        pp_max_speed_arg,
        pp_use_frenet_path_arg,
        pp_frenet_topic_arg,

        # Nodes
        fgm_node,
        pp_node,
    ])


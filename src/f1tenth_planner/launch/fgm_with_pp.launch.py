import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # =========================================
    # 1. FGM 파라미터 인자 (튜닝용 포함)
    # =========================================
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

    # [NEW] FGM 튜닝 파라미터들
    fgm_speed_check_fov_deg_arg = DeclareLaunchArgument(
        'fgm_speed_check_fov_deg', default_value='30.0',
        description='속도 제어를 위해 전방을 감지하는 각도 (deg)'
    )
    fgm_required_clearance_arg = DeclareLaunchArgument(
        'fgm_required_clearance', default_value='0.55',
        description='주행시 벽이나 장애물로부터 확보할 물리적 거리 (m)'
    )
    fgm_width_weight_arg = DeclareLaunchArgument(
        'fgm_width_weight', default_value='0.5',
        description='Gap 너비에 대한 가중치'
    )
    fgm_angle_weight_arg = DeclareLaunchArgument(
        'fgm_angle_weight', default_value='5.0',
        description='글로벌 경로와의 각도 차이에 대한 페널티 가중치'
    )
    fgm_steer_weight_arg = DeclareLaunchArgument(
        'fgm_steer_weight', default_value='0.1',
        description='조향각 크기에 대한 페널티 가중치'
    )
    fgm_hysteresis_bonus_arg = DeclareLaunchArgument(
        'fgm_hysteresis_bonus', default_value='2.0',
        description='이전 경로 유지 시 부여하는 보너스 점수'
    )
    fgm_change_threshold_arg = DeclareLaunchArgument(
        'fgm_change_threshold', default_value='0.3',
        description='이전 각도와 유사하다고 판단하는 임계값 (rad)'
    )
    fgm_smoothing_alpha_arg = DeclareLaunchArgument(
        'fgm_smoothing_alpha', default_value='0.4',
        description='조향각 평활화 계수 (0~1)'
    )
    fgm_dynamic_bubble_speed_coeff_arg = DeclareLaunchArgument(
        'fgm_dynamic_bubble_speed_coeff', default_value='0.1',
        description='속도에 따른 버블 반경 증가 계수'
    )

    # Planning & Lookahead & Speed (FGM 내부용)
    fgm_min_plan_arg = DeclareLaunchArgument('fgm_min_planning_dist', default_value='2.0')
    fgm_max_plan_arg = DeclareLaunchArgument('fgm_max_planning_dist', default_value='5.0')
    fgm_plan_gain_arg = DeclareLaunchArgument('fgm_planning_gain', default_value='1.0')

    fgm_min_look_arg = DeclareLaunchArgument('fgm_min_lookahead', default_value='1.5')
    fgm_max_look_arg = DeclareLaunchArgument('fgm_max_lookahead', default_value='3.5')
    fgm_look_gain_arg = DeclareLaunchArgument('fgm_lookahead_gain', default_value='0.6')

    fgm_max_speed_arg = DeclareLaunchArgument('fgm_max_speed', default_value='4.0')
    fgm_min_speed_arg = DeclareLaunchArgument('fgm_min_speed', default_value='2.0')
    fgm_slow_down_arg = DeclareLaunchArgument('fgm_slow_down_dist', default_value='2.5')

    # =========================================
    # 2. Pure Pursuit 파라미터 인자
    # =========================================
    pp_csv_path_arg = DeclareLaunchArgument(
        'pp_csv_path', default_value='raceline_with_speed.csv',
        description='CSV 기반 주행시 사용할 경로 파일'
    )
    pp_lookahead_min_arg = DeclareLaunchArgument('pp_lookahead_min', default_value='1.0')
    pp_lookahead_gain_arg = DeclareLaunchArgument('pp_lookahead_gain', default_value='0.3')
    pp_wheelbase_arg = DeclareLaunchArgument('pp_wheelbase', default_value='0.33')
    
    # [핵심] PP Max Speed 연동
    pp_max_speed_arg = DeclareLaunchArgument(
        'pp_max_speed', default_value='4.0',
        description='Pure Pursuit: 최대 주행 속도 (m/s)'
    )
    
    pp_use_frenet_path_arg = DeclareLaunchArgument('pp_use_frenet_path', default_value='true')
    pp_frenet_topic_arg = DeclareLaunchArgument('pp_frenet_path_topic', default_value='/fgm_path')

    # =========================================
    # 3. 노드 정의
    # =========================================
    
    # FGM Node
    fgm_node = Node(
        package='f1tenth_planner',
        executable='fgm_node',
        name='fgm_node',
        output='screen',
        parameters=[{
            'gap_threshold':       LaunchConfiguration('fgm_gap_threshold'),
            'bubble_radius':       LaunchConfiguration('fgm_bubble_radius'),
            'fov_angle':           LaunchConfiguration('fgm_fov_angle'),

            # 튜닝 파라미터 매핑
            'speed_check_fov_deg': LaunchConfiguration('fgm_speed_check_fov_deg'),
            'required_clearance':  LaunchConfiguration('fgm_required_clearance'),
            'width_weight':        LaunchConfiguration('fgm_width_weight'),
            'angle_weight':        LaunchConfiguration('fgm_angle_weight'),
            'steer_weight':        LaunchConfiguration('fgm_steer_weight'),
            'hysteresis_bonus':    LaunchConfiguration('fgm_hysteresis_bonus'),
            'change_threshold':    LaunchConfiguration('fgm_change_threshold'),
            'smoothing_alpha':     LaunchConfiguration('fgm_smoothing_alpha'),
            'dynamic_bubble_speed_coeff': LaunchConfiguration('fgm_dynamic_bubble_speed_coeff'),

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

    # Pure Pursuit Node
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
            
            # [핵심] Launch Argument 'pp_max_speed'를 노드 파라미터 'max_speed'로 매핑
            'max_speed':       LaunchConfiguration('pp_max_speed'),
            
            'use_frenet_path': LaunchConfiguration('pp_use_frenet_path'),
            'frenet_path_topic': LaunchConfiguration('pp_frenet_path_topic'),
        }]
    )

    return LaunchDescription([
        # Args - FGM Basic
        fgm_gap_threshold_arg,
        fgm_bubble_radius_arg,
        fgm_fov_angle_arg,
        
        # Args - FGM Tuning
        fgm_speed_check_fov_deg_arg,
        fgm_required_clearance_arg,
        fgm_width_weight_arg,
        fgm_angle_weight_arg,
        fgm_steer_weight_arg,
        fgm_hysteresis_bonus_arg,
        fgm_change_threshold_arg,
        fgm_smoothing_alpha_arg,
        fgm_dynamic_bubble_speed_coeff_arg,

        # Args - FGM Path/Speed
        fgm_min_plan_arg,
        fgm_max_plan_arg,
        fgm_plan_gain_arg,
        fgm_min_look_arg,
        fgm_max_look_arg,
        fgm_look_gain_arg,
        fgm_max_speed_arg,
        fgm_min_speed_arg,
        fgm_slow_down_arg,
        
        # Args - Pure Pursuit
        pp_csv_path_arg,
        pp_lookahead_min_arg,
        pp_lookahead_gain_arg,
        pp_wheelbase_arg,
        pp_max_speed_arg,        # 추가됨
        pp_use_frenet_path_arg,
        pp_frenet_topic_arg,

        # Nodes
        fgm_node,
        pp_node,
    ])
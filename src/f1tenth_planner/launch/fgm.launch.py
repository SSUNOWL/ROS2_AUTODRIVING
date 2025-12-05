import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_planner', 
            executable='fgm_node',
            name='fgm_node',
            output='screen',
            parameters=[{
                # --- [1] 안전 및 인식 파라미터 ---
                # 최종 선정된 FGM_FINAL 파라미터
                'fov_angle': 183.4000,                # fgm_fov_angle
                'speed_check_fov_deg': 25.3000,       # fgm_speed_check_fov_deg
                'gap_threshold': 0.8900,              # fgm_gap_threshold
                'bubble_radius': 0.4550,              # fgm_bubble_radius
                'dynamic_bubble_speed_coeff': 0.1520, # fgm_dynamic_bubble_speed_coeff

                # --- [2] 경로 생성 파라미터 (기본 유지) ---
                'min_planning_dist': 2.0,
                'max_planning_dist': 5.0,
                'planning_gain': 1.0,
                'min_lookahead': 1.5,
                'max_lookahead': 3.5,
                'lookahead_gain': 0.6,

                # --- [3] 속도 제어 파라미터 ---
                'max_speed': 4.4000,                 # SPD (pp_max_speed 역할)
                # min_speed, slow_down_dist는 코드 기본값 유지

                # --- [4] 알고리즘 튜닝 파라미터 ---
                'required_clearance': 0.5350,        # fgm_required_clearance
                'width_weight': 0.5900,              # fgm_width_weight
                'angle_weight': 7.6000,              # fgm_angle_weight
                'steer_weight': 0.0610,              # fgm_steer_weight
                'hysteresis_bonus': 1.7000,          # fgm_hysteresis_bonus
                'change_threshold': 0.1980,          # fgm_change_threshold
                'smoothing_alpha': 0.5050            # fgm_smoothing_alpha
            }]
        )
    ])

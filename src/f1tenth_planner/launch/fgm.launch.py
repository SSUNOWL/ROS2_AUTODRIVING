import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            # [주의] 아래 'your_package_name'을 실제 패키지 이름으로 변경해야 합니다.
            package='f1tenth_planner', 
            executable='fgm_node',
            name='fgm_node',
            output='screen',
            parameters=[{
                # --- [1] 안전 및 인식 파라미터 ---
                'fov_angle': 190.5232,           # fgm_fov_angle
                'speed_check_fov_deg': 21.8559,  # fgm_speed_check_fov_deg
                'gap_threshold': 0.9946,         # fgm_gap_threshold
                'bubble_radius': 0.3625,         # fgm_bubble_radius
                'dynamic_bubble_speed_coeff': 0.1703, # fgm_dynamic_bubble_speed_coeff

                # --- [2] 경로 생성 파라미터 (기본값 유지 또는 필요시 추가 수정) ---
                'min_planning_dist': 2.0,
                'max_planning_dist': 5.0,
                'planning_gain': 1.0,
                'min_lookahead': 1.5,
                'max_lookahead': 3.5,
                'lookahead_gain': 0.6,

                # --- [3] 속도 제어 파라미터 ---
                'max_speed': 5.6373,             # fgm_max_speed
                # min_speed, slow_down_dist는 코드 기본값 사용 (필요시 추가)

                # --- [4] 알고리즘 튜닝 파라미터 ---
                'required_clearance': 0.4313,    # fgm_required_clearance
                'width_weight': 0.7846,          # fgm_width_weight
                'angle_weight': 8.4080,          # fgm_angle_weight
                'steer_weight': 0.0472,          # fgm_steer_weight
                'hysteresis_bonus': 1.3784,      # fgm_hysteresis_bonus
                'change_threshold': 0.2005,      # fgm_change_threshold
                'smoothing_alpha': 0.5416        # fgm_smoothing_alpha
            }]
        )
    ])
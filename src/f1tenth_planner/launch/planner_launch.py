from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. FGM Node (Path Generator)
        # 역할: LiDAR를 보고 장애물이 없는 쪽으로 '길(/fgm_path)'을 깔아줍니다.
        Node(
            package='f1tenth_planner',
            executable='fgm_node',
            name='fgm_node',
            output='screen',
            parameters=[{
                'gap_threshold': 1.5,      # 이보다 넓은 틈만 길로 인정
                'bubble_radius': 0.45,     # 장애물 회피 반경
                'planning_distance': 3.0,  # 전방 3m까지 경로 생성
                'fov_angle': 120.0         # 전방 120도만 봄
            }]
        ),

        # 2. Pure Pursuit Node (Path Follower)
        # 역할: '/fgm_path'를 구독해서 따라갑니다.
        Node(
            package='f1tenth_planner',
            executable='pure_pursuit_node',
            name='pure_pursuit',
            output='screen',
            parameters=[{
                'use_frenet_path': True,          # True면 토픽을 구독 (CSV 무시)
                'frenet_path_topic': '/selected_path', # FGM이 만든 길을 구독
                'lookahead_gain': 0.4,
                'max_speed': 5.0,                 # 최대 속도 제한
                'wheelbase': 0.33
            }]
        ),
        Node(
            package='planner_mux',
            executable='planner_mux_node',
            name='local_planner_mux',
            output='screen',
            parameters=[{
                'd_min': 0.45,
                'v_ref': 5.0,
                'jerk_ref': 5.0,
                'track_ref': 0.5,
                'w_speed': 1.0,
                'w_track': 1.0,
                'w_comfort': 1.0,
            }]
        )
    ])
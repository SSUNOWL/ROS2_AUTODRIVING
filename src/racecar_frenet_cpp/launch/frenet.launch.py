import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            # [주의] 현재 작업 중인 패키지 이름으로 변경해주세요 (예: f1tenth_stack)
            package='racecar_frenet_cpp', 
            executable='frenet_local_planner',
            name='frenet_local_planner',
            output='screen',
            parameters=[{
                # 요청하신 파라미터 값 적용
                'max_speed': 6.5,      # spd
                'target_speed': 5.2,   # tgt
                'max_accel': 6.0,     # acc
                'max_curvature': 0.9   # crv
            }]
        )
    ])
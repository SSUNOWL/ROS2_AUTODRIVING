from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 저장 위치는 네가 원하는 경로로 수정
    csv_out = os.path.join(
        os.getcwd(),
        "opponent_traj_recorded.csv"
    )

    return LaunchDescription([
        Node(
            package='racecar_experiments',  # 실제 패키지명으로 수정
            executable='opponent_trajectory_recorder',
            name='opponent_trajectory_recorder',
            output='screen',
            parameters=[
                {
                    'odom_topic': '/opp_racecar/odom',  # 상대차 odom 토픽
                    'csv_path': csv_out,
                    'min_dt': 0.02
                }
            ]
        )
    ])

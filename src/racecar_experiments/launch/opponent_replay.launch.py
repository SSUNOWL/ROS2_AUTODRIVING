from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    csv_default = os.path.join(
        os.getcwd(),
        'spielberg_opponent_scenarioA.csv'  # ← 여기만 네 파일 이름으로
    )

    return LaunchDescription([
        Node(
            package='racecar_experiments',      # 실제 패키지명
            executable='opponent_replay_node',  # 아까 만든 player 노드
            name='opponent_replay_node',
            output='screen',
            parameters=[
                {
                    'csv_path': csv_default,
                    'frame_id': 'map',
                    'child_frame_id': 'opponent_base',
                    'odom_topic': '/opp_racecar/odom',
                    'publish_rate': 50.0
                }
            ]
        )
    ])

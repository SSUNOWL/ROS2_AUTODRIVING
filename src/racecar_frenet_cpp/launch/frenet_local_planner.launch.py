import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 공유 디렉토리 경로를 가져옴
    package_share_directory = get_package_share_directory('racecar_frenet_cpp')

    return LaunchDescription([
        # 런치 파라미터 선언
        DeclareLaunchArgument('max_speed', default_value='5.0', description='Maximum speed'),
        DeclareLaunchArgument('max_accel', default_value='2.0', description='Maximum acceleration'),
        DeclareLaunchArgument('max_curvature', default_value='1.0', description='Maximum curvature'),
        DeclareLaunchArgument('s_horizon', default_value='15.0', description='Planning horizon length'),

        # frenet_local_planner 노드 실행
        Node(
            package='racecar_frenet_cpp',
            executable='frenet_local_planner',
            name='frenet_local_planner',
            output='screen',
            parameters=[os.path.join(package_share_directory, 'config', 'params.yaml')],
        )
    ])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
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
            parameters=[os.path.join(
                get_package_share_directory('racecar_frenet_cpp'), 'config', 'params.yaml'
            )],
            remappings=[('/scan', '/robot/scan'), ('/ego_racecar/odom', '/robot/odom')]
        )
    ])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gym_pkg = FindPackageShare('f1tenth_gym_ros')

    # 기본 맵: playground
    map_name_arg = DeclareLaunchArgument('map_name', default_value='playground')
    map_ext_arg = DeclareLaunchArgument('map_img_ext', default_value='.png')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    # [수정됨] '_map'을 강제로 붙이던 PythonExpression을 삭제하고,
    # 입력받은 map_name을 그대로 사용합니다.
    
    # 지도 런치 포함
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gym_pkg, 'launch', 'map_gym_bridge.py']) 
        ),
        launch_arguments={
            'map_name': LaunchConfiguration('map_name'), # [수정] _map 제거됨
            'map_img_ext': LaunchConfiguration('map_img_ext'),
            'num_agent': '2',
            'use_rviz': LaunchConfiguration('use_rviz')
        }.items()
    )

    return LaunchDescription([
        map_name_arg,
        map_ext_arg,
        use_rviz_arg,
        map_launch
    ])
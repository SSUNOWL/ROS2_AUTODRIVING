import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gym_pkg = FindPackageShare('f1tenth_gym_ros')

    # 인자 정의
    map_name_arg = DeclareLaunchArgument('map_name', default_value='Spielberg')
    map_ext_arg = DeclareLaunchArgument('map_img_ext', default_value='.png')
    
    # [추가] RViz 실행 옵션 (기본값 false -> Headless)
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')

    real_map_name = PythonExpression(["'", LaunchConfiguration('map_name'), "' + '_map'"])

    # 지도 런치 포함
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gym_pkg, 'launch', 'map_gym_bridge.py']) 
        ),
        launch_arguments={
            'map_name': real_map_name,
            'map_img_ext': LaunchConfiguration('map_img_ext'),
            'num_agent': '1',
            'use_rviz': LaunchConfiguration('use_rviz') # [핵심] 인자 전달
        }.items()
    )

    return LaunchDescription([
        map_name_arg,
        map_ext_arg,
        use_rviz_arg, # 인자 등록
        map_launch
    ])
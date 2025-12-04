import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gym_pkg = FindPackageShare('f1tenth_gym_ros')

    # 인자 정의
    map_name_arg = DeclareLaunchArgument('map_name', default_value='playground')
    map_ext_arg = DeclareLaunchArgument('map_img_ext', default_value='.png')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')

    # ==========================================================
    # [핵심 로직 1] 차량 대수 자동 결정
    # playground(장애물) -> 2대, 그 외(레이싱) -> 1대
    # ==========================================================
    num_agent_expr = PythonExpression([
        "'2' if '", LaunchConfiguration('map_name'), "' == 'playground' else '1'"
    ])

    # ==========================================================
    # [핵심 로직 2] 맵 이름 규칙 통일 (_map 접미사 처리)
    # Frenet 실험 때 쓰던 맵들은 뒤에 '_map'이 붙어야 했고,
    # Playground는 그냥 'playground'여야 했던 규칙을 여기서 자동화합니다.
    # ==========================================================
    real_map_name_expr = PythonExpression([
        # 만약 map_name이 playground가 아니면 뒤에 _map을 붙이고, 맞으면 그냥 쓴다.
        "('", LaunchConfiguration('map_name'), "' + '_map') if '", LaunchConfiguration('map_name'), "' != 'playground' else '", LaunchConfiguration('map_name'), "'"
    ])

    # 지도 런치 포함 (gym bridge)
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gym_pkg, 'launch', 'map_gym_bridge.py']) 
        ),
        launch_arguments={
            'map_name': real_map_name_expr,   # 자동 계산된 맵 이름
            'map_img_ext': LaunchConfiguration('map_img_ext'),
            'num_agent': num_agent_expr,      # 자동 계산된 에이전트 수
            'use_rviz': LaunchConfiguration('use_rviz')
        }.items()
    )

    return LaunchDescription([
        map_name_arg,
        map_ext_arg,
        use_rviz_arg,
        map_launch
    ])
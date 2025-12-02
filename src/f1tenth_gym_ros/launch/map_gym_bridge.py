import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gym_package_share = get_package_share_directory('f1tenth_gym_ros')
    
    # 1. Launch Arguments 선언
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='Spielberg_map',
        description='Map name without extension'
    )

    num_agent_arg = DeclareLaunchArgument(
        'num_agent',
        default_value='1',
        description='Number of agents (1: Ego only, 2: Ego + Opponent)'
    )
    
    # [수정] 맵 이미지 확장자 인자 추가
    map_img_ext_arg = DeclareLaunchArgument(
        'map_img_ext',
        default_value='.png',  # 기본값은 .png로 유지하거나 .pgm으로 변경 가능
        description='Map image file extension (.png or .pgm)'
    )

    # 2. 경로 설정
    sim_config_path = os.path.join(gym_package_share, 'config', 'sim.yaml')
    
    map_name_config = LaunchConfiguration('map_name')
    num_agent_config = LaunchConfiguration('num_agent')
    # [수정] 맵 이미지 확장자 설정
    map_img_ext_config = LaunchConfiguration('map_img_ext') 

    # 맵 경로 동적 생성
    map_path_no_ext = PathJoinSubstitution([gym_package_share, 'maps', map_name_config])
    
    # 맵 YAML 파일 경로 (Map Server용)
    map_yaml_file = PathJoinSubstitution([
        gym_package_share, 'maps', 
        PythonExpression(["'", map_name_config, ".yaml'"]) 
    ])

    # Opponent Robot State Publisher 실행 조건
    has_opp_condition = IfCondition(
        PythonExpression([num_agent_config, " > 1"])
    )

    # 3. 노드 정의

    # (1) Gym Bridge
    # [수정] 'map_img_ext' 파라미터에 LaunchConfiguration 연결
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        output='screen',
        parameters=[
            sim_config_path,
            {
                'map_path': map_path_no_ext,
                'map_img_ext': map_img_ext_config, # <--- 이 부분이 수정됨
                'num_agent': num_agent_config
            }
        ]
    )

    # (2) Map Server (이 노드는 .yaml 파일 경로를 사용하므로 수정 필요 없음)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'output': 'screen'},
            {'use_sim_time': True}
        ]
    )
    
    # ... (생략된 다른 노드들: RViz, Lifecycle Manager, Robot State Publishers)

    # (3) RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(gym_package_share, 'launch', 'gym_bridge.rviz')]
    )

    # (4) Lifecycle Manager
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # (5) Robot State Publishers
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(gym_package_share, 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(gym_package_share, 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')],
        condition=has_opp_condition
    )

    return LaunchDescription([
        map_name_arg,
        num_agent_arg,
        map_img_ext_arg, # [수정] 인자 추가
        rviz_node,
        bridge_node,
        nav_lifecycle_node,
        map_server_node,
        ego_robot_publisher,
        opp_robot_publisher
    ])
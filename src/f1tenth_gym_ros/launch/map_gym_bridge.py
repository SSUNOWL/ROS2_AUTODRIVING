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
    
    # [NEW] 1. Launch Arguments 선언
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

    # 2. 경로 설정
    sim_config_path = os.path.join(gym_package_share, 'config', 'sim.yaml')
    
    map_name_config = LaunchConfiguration('map_name')
    num_agent_config = LaunchConfiguration('num_agent')

    # 맵 경로 동적 생성
    map_path_no_ext = PathJoinSubstitution([gym_package_share, 'maps', map_name_config])
    
    # 맵 YAML 파일 경로 (Map Server용)
    map_yaml_file = PathJoinSubstitution([
        gym_package_share, 'maps', 
        PythonExpression(["'", map_name_config, ".yaml'"]) 
    ])

    # [NEW] Opponent Robot State Publisher 실행 조건
    # num_agent가 2 이상일 때만 실행 (문자열 비교)
    has_opp_condition = IfCondition(
        PythonExpression([num_agent_config, " > 1"])
    )

    # 3. 노드 정의

    # (1) Gym Bridge
    # sim.yaml을 로드하고, map_path와 num_agent를 덮어씁니다.
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        output='screen',
        parameters=[
            sim_config_path,
            {
                'map_path': map_path_no_ext,
                'map_img_ext': '.png',
                'num_agent': num_agent_config  # [NEW] 파라미터 전달
            }
        ]
    )

    # (2) Map Server
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

    # [NEW] 조건부 실행 (IfCondition 적용)
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
        rviz_node,
        bridge_node,
        nav_lifecycle_node,
        map_server_node,
        ego_robot_publisher,
        opp_robot_publisher
    ])
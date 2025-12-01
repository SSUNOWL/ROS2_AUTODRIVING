import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gym_pkg = FindPackageShare('f1tenth_gym_ros')
    frenet_pkg = FindPackageShare('racecar_frenet_cpp')
    
    # Launch Arguments
    path_mode_arg = DeclareLaunchArgument('path_mode', default_value='mux')
    map_name_arg = DeclareLaunchArgument('map_name', default_value='Spielberg_map')
    csv_path_arg = DeclareLaunchArgument('csv_path', default_value=os.path.join(os.getcwd(), 'raceline_with_yaw.csv'))
    
    # [NEW] Agent 수 설정 (Time Trial은 기본이 1대)
    num_agent_arg = DeclareLaunchArgument('num_agent', default_value='1', description='Number of agents')

    # 1. 맵 & 시뮬레이터 실행 (파라미터 전달)
    map_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([gym_pkg, 'launch', 'map_gym_bridge.py'])),
        launch_arguments={
            'map_name': LaunchConfiguration('map_name'),
            'num_agent': LaunchConfiguration('num_agent') # [NEW] 전달
        }.items()
    )

    # 2. Frenet Planner
    frenet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([frenet_pkg, 'launch', 'frenet_local_planner.launch.py']))
    )

    # 3. CSV 경로 발행
    static_path_node = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher',
        output='screen',
        parameters=[{'csv_path': LaunchConfiguration('csv_path')}]
    )

    # 4. FGM Node
    fgm_node = Node(
        package='f1tenth_planner',
        executable='fgm_node',
        name='fgm_node',
        output='screen',
        parameters=[{
            'planning_distance': 3.0, 'gap_threshold': 1.0, 'bubble_radius': 0.4,
            'fov_angle': 180.0, 'max_speed': 7.0, 'min_speed': 3.0, 'slow_down_dist': 2.5
        }]
    )

    # 5. Planner Mux Node (레이싱 튜닝)
    mux_node = Node(
        package='planner_mux',
        executable='planner_mux_node',
        name='planner_mux_node',
        output='screen',
        parameters=[{
            'd_min': 0.2, 'v_ref': 8.0, 'jerk_ref': 5.0, 'track_ref': 0.5,
            'w_speed': 5.0, 'w_track': 2.0, 'w_comfort': 0.0, 'w_clearance': 0.5,
            'react_time': 0.05, 'a_brake_max': 10.0, 'max_speed_mux': 10.0, 'max_dv_step_mux': 5.0
        }]
    )

    # 6. Pure Pursuit Node
    pure_pursuit_node = Node(
        package='f1tenth_planner',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'path_mode': LaunchConfiguration('path_mode'),
            'use_frenet_path': True, 'lookahead_gain': 0.4, 'lookahead_min': 1.0
        }],
        remappings=[('/drive', '/ego_racecar/drive'), ('/ego_racecar/odom', '/ego_racecar/odom')]
    )

    # 7. Collision Monitor
    collision_node = Node(
        package='racecar_experiments',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[{'stop_threshold': 0.20, 'go_threshold': 0.25}]
    )

    return LaunchDescription([
        path_mode_arg, map_name_arg, csv_path_arg, num_agent_arg,
        map_bridge_launch, static_path_node, frenet_launch, fgm_node,
        mux_node, pure_pursuit_node, collision_node
    ])
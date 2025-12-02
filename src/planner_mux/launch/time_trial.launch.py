import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction, 
    GroupAction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    frenet_pkg = FindPackageShare('racecar_frenet_cpp')
    
    # --- Launch Arguments ---
    path_mode_arg = DeclareLaunchArgument('path_mode', default_value='mux')
    map_name_arg = DeclareLaunchArgument('map_name', default_value='Spielberg_map')
    
    # CSV 경로 (기본값 유지)
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value=os.path.join(os.getcwd(), 'raceline_with_yaw.csv')
    )
    
    # Pure Pursuit이 추종할 경로 토픽 인수
    pp_path_topic_arg = DeclareLaunchArgument(
        'pp_path_topic', 
        default_value='/selected_path',  # 기본값: MUX 출력
        description='Topic name for Pure Pursuit to follow (e.g., /frenet_path, /fgm_path, /selected_path)'
    )

    # --- 상대차 스폰 관련 Launch Argument 추가 ---
    spawn_opponent_enabled_arg = DeclareLaunchArgument(
        'spawn_opponent_enabled',
        default_value='false',
        description='Enable opponent spawn via static_path_publisher'
    )

    opponent_x_arg = DeclareLaunchArgument(
        'opponent_x',
        default_value='0.0',
        description='Opponent spawn x position in map frame'
    )

    opponent_y_arg = DeclareLaunchArgument(
        'opponent_y',
        default_value='0.0',
        description='Opponent spawn y position in map frame'
    )

    opponent_yaw_arg = DeclareLaunchArgument(
        'opponent_yaw',
        default_value='0.0',
        description='Opponent spawn yaw (rad) in map frame'
    )
    
    # -----------------------------------------------------
    # 0. 초기 정지 명령 (Ego만)
    # -----------------------------------------------------
    yaml_msg_ego = (
        "{header: {stamp: now, frame_id: ego_base_link}, "
        "drive: {steering_angle: 0.0, speed: 0.0}}"
    )

    ego_initial_stop_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-1',
            '/drive',
            'ackermann_msgs/msg/AckermannDriveStamped',
            yaml_msg_ego
        ],
        name='ego_initial_stop_command',
        output='screen'
    )

    # 1. static_path_publisher (CSV 경로 + 상대차 파라미터 발행)
    static_path_node = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher',
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('csv_path'),
            'spawn_opponent_enabled': LaunchConfiguration('spawn_opponent_enabled'),
            'opponent_x': LaunchConfiguration('opponent_x'),
            'opponent_y': LaunchConfiguration('opponent_y'),
            'opponent_yaw': LaunchConfiguration('opponent_yaw'),
        }]
    )

    # 2-1. Frenet Planner
    frenet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([frenet_pkg, 'launch', 'frenet_local_planner.launch.py'])
        )
    )

    # 2-2. FGM Node
    fgm_node = Node(
        package='f1tenth_planner',
        executable='fgm_node',
        name='fgm_node',
        output='screen',
        parameters=[{
            'planning_distance': 3.0,
            'gap_threshold': 1.0,
            'bubble_radius': 0.4,
            'fov_angle': 180.0,
            'max_speed': 7.0,
            'min_speed': 3.0,
            'slow_down_dist': 2.5
        }]
    )

    # 2-3. Planner Mux Node (레이싱 튜닝)
    mux_node = Node(
        package='planner_mux',
        executable='planner_mux_node',
        name='planner_mux_node',
        output='screen',
        parameters=[{
            'd_min': 0.2,
            'v_ref': 8.0,
            'jerk_ref': 5.0,
            'track_ref': 0.5,
            'w_speed': 5.0,
            'w_track': 2.0,
            'w_comfort': 0.0,
            'w_clearance': 0.5,
            'react_time': 0.05,
            'a_brake_max': 10.0,
            'max_speed_mux': 10.0,
            'max_dv_step_mux': 5.0
        }]
    )

    # 2-4. Pure Pursuit Node
    pure_pursuit_node = Node(
        package='f1tenth_planner',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'use_frenet_path': True,
            'lookahead_gain': 0.4,
            'lookahead_min': 1.0,
            # 런치 인수를 매개변수로 전달하여 토픽 지정
            'frenet_path_topic': LaunchConfiguration('pp_path_topic')
        }],
        remappings=[
            ('/ego_racecar/odom', '/ego_racecar/odom')
        ]
    )

    # 2-5. Collision Monitor
    collision_node = Node(
        package='racecar_experiments',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[{'stop_threshold': 0.20, 'go_threshold': 0.25}]
    )
    
    # -----------------------------------------------------
    # 3. TimerAction을 사용하여 실행 순서 제어
    # -----------------------------------------------------

    # 3-0. 런치 시작 즉시 정지 명령 수행
    initial_stop_timer = TimerAction(
        period=0.0,  # t = 0.0 s
        actions=[ego_initial_stop_cmd]
    )

    # 3-1. 그 다음 static_path_publisher 실행
    static_path_timer = TimerAction(
        period=1.0,  # t = 1.0 s
        actions=[static_path_node]
    )

    # 3-2. 5초 뒤에 주요 노드들 시작
    delayed_nodes_group = GroupAction([
        frenet_launch,
        fgm_node,
        mux_node,
        pure_pursuit_node,
        collision_node
    ])

    delay_action = TimerAction(
        period=5.0,  # t = 5.0 s
        actions=[delayed_nodes_group]
    )

    # 최종 LaunchDescription
    return LaunchDescription([
        path_mode_arg,
        map_name_arg,
        csv_path_arg,
        pp_path_topic_arg,
        spawn_opponent_enabled_arg,
        opponent_x_arg,
        opponent_y_arg,
        opponent_yaw_arg,

        # 0: 초기 정지 명령
        initial_stop_timer,

        # 1: static_path_publisher 시작
        static_path_timer,

        # 2: 프레넷/FGM/MUX/PP/Collision 시작
        delay_action,
    ])

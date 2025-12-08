import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction, 
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
    LogInfo
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import GroupAction

def generate_launch_description():
    # ... (상단 패키지 및 인자 설정 코드는 기존과 동일) ...
    exp_pkg = FindPackageShare('racecar_experiments')
    frenet_pkg = FindPackageShare('racecar_frenet_cpp')
    planner_pkg = FindPackageShare('f1tenth_planner')

    map_name_arg = DeclareLaunchArgument('map_name', default_value='playground')
    w_speed_arg = DeclareLaunchArgument('w_speed', default_value='1.0')
    w_track_arg = DeclareLaunchArgument('w_track', default_value='1.0')
    w_comfort_arg = DeclareLaunchArgument('w_comfort', default_value='1.0')
    w_clearance_arg = DeclareLaunchArgument('w_clearance', default_value='1.0')
    w_dynamics_arg = DeclareLaunchArgument('w_dynamics', default_value='1.0')
    d_min_arg = DeclareLaunchArgument('d_min', default_value='0.15')
    opponent_csv_arg = DeclareLaunchArgument('opponent_csv_filename', default_value='bumper_slow_1.csv')

    is_playground = PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'playground'"])
    
    planner_mode_expr = PythonExpression([
        "'MUX_OBSTACLE' if '", LaunchConfiguration('map_name'), "' == 'playground' else 'MUX_RACING'"
    ])

    scenario_name_str = PythonExpression([
        "'Mux_' + '", LaunchConfiguration('map_name'), "' + "
        "'_WS' + '", LaunchConfiguration('w_speed'), "' + "
        "'_WT' + '", LaunchConfiguration('w_track'), "' + "
        "'_WC' + '", LaunchConfiguration('w_comfort'), "' + "
        "'_WCL' + '", LaunchConfiguration('w_clearance'), "' + "
        "'_WD' + '", LaunchConfiguration('w_dynamics'), "' + "
        "'_DM' + '", LaunchConfiguration('d_min'), "' +" 
        "'_Opp_' + '", LaunchConfiguration('opponent_csv_filename'), "'"
    ])

    csv_filename = PythonExpression(["'raceline_' + '", LaunchConfiguration('map_name'), "' + '.csv'"])
    ego_csv_path = PathJoinSubstitution([os.getcwd(), csv_filename])
    opponent_csv_path = PathJoinSubstitution([os.getcwd(), LaunchConfiguration('opponent_csv_filename')])
    
    log_output_dir = os.path.join(os.getcwd(), 'logs')

    # ==============================
    # 3. Nodes Configuration
    # ==============================

    stop_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/drive', 'ackermann_msgs/msg/AckermannDriveStamped',
             "{header: {stamp: now, frame_id: ego_racecar/base_link}, drive: {steering_angle: 0.0, speed: 0.0}}"],
        output='screen'
    )
    stop_opp_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/opp_drive', 'ackermann_msgs/msg/AckermannDriveStamped',
                "{header: {stamp: now, frame_id: opp_racecar/base_link}, drive: {steering_angle: 0.0, speed: 0.0}}"],
        output='screen'
    )

    # [수정] Static Path Publisher를 2번 정의 (재실행 보장을 위해 이름을 다르게)
    # 1차 시도 (1.0초)
    static_path_node_1 = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher_1',
        output='screen',
        parameters=[{
            'csv_path': ego_csv_path,
            'opponent_csv_path': opponent_csv_path,
            'spawn_opponent_enabled': is_playground,
            'topic_name': '/plan',
            'opponent_spawn_topic': '/goal_pose'
        }]
    )

    # 2차 시도 (2.5초 - 확실한 위치 고정용)
    static_path_node_2 = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher_2',
        output='screen',
        parameters=[{
            'csv_path': ego_csv_path,
            'opponent_csv_path': opponent_csv_path,
            'spawn_opponent_enabled': is_playground,
            'topic_name': '/plan',
            'opponent_spawn_topic': '/goal_pose'
        }]
    )

    opponent_pp_node = Node(
        condition=IfCondition(is_playground),
        package='racecar_experiments',
        executable='opponent_pure_pursuit_node',
        name='opponent_pure_pursuit_node',
        output='screen',
        parameters=[{
            'csv_path': opponent_csv_path,
            'odom_topic': '/opp_racecar/odom',
            'drive_topic': '/opp_drive',
            'max_speed': 5.0 
        }]
    )

    collision_node = Node(
        package='racecar_experiments',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[{
            'stop_threshold': 0.20,
            'go_threshold': 0.25
        }]
    )

    mux_logger_node = Node(
        package='racecar_experiments',
        executable='mux_logger', 
        name='mux_logger',
        output='screen',
        parameters=[{
            'output_dir': log_output_dir,
            'scenario_name': scenario_name_str,
            'planner_mode': planner_mode_expr,
            'goal_tolerance': 1.0, 
            'start_safe_dist': 5.0, 
            'stuck_timeout': 5.0, 
            'stuck_dist_thresh': 0.2, 
            'safe_dist_threshold': 0.5 
        }]
    )

    frenet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([frenet_pkg, 'launch', 'frenet.launch.py'])
        )
    )

    fgm_node = Node(
        package='f1tenth_planner',
        executable='fgm_node',
        name='fgm_node',
        output='screen',
        parameters=[{
            'base_frame': 'ego_racecar/base_link',
            # ... (기존 파라미터 유지) ...
            'fov_angle': 183.4000,
            'speed_check_fov_deg': 25.3000,
            'gap_threshold': 0.8900,
            'bubble_radius': 0.4550,
            'dynamic_bubble_speed_coeff': 0.1520,
            'min_planning_dist': 2.0,
            'max_planning_dist': 5.0,
            'planning_gain': 1.0,
            'min_lookahead': 1.5,
            'max_lookahead': 3.5,
            'lookahead_gain': 0.6,
            'max_speed': 4.4000,
            'required_clearance': 0.5350,
            'width_weight': 0.5900,
            'angle_weight': 7.6000,
            'steer_weight': 0.0610,
            'hysteresis_bonus': 1.7000,
            'change_threshold': 0.1980,
            'smoothing_alpha': 0.5050,
            'slow_down_dist': 2.5,
            'drive_topic': '/trash_drive' 
        }]
    )

    mux_node = Node(
        package='planner_mux',
        executable='planner_mux_node',
        name='local_planner_mux',
        output='screen',
        parameters=[{
            'w_speed': LaunchConfiguration('w_speed'),
            'w_track': LaunchConfiguration('w_track'),
            'w_comfort': LaunchConfiguration('w_comfort'),
            'w_clearance': LaunchConfiguration('w_clearance'),
            'w_dynamics': LaunchConfiguration('w_dynamics'),
            'd_min': LaunchConfiguration('d_min'),
        }]
    )

    main_pp_node = Node(
        package='f1tenth_planner',
        executable='pure_pursuit_node',
        name='mux_pure_pursuit',
        output='screen',
        parameters=[{
            'use_frenet_path': True,
            'path_topic': '/selected_path',   
            'drive_topic': '/drive',          
            'lookahead_dist': 1.0,            
            'max_speed': 5.5,                 
            'visualize_lookahead': True
        }],
    )

    exit_on_logger = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mux_logger_node,
            on_exit=[LogInfo(msg="Mux Logger finished."), EmitEvent(event=Shutdown())]
        )
    )

    timeout_action = TimerAction(
        period=100.0,
        actions=[LogInfo(msg="Timeout reached."), EmitEvent(event=Shutdown())]
    )
    conditional_stop_opp = GroupAction(
        condition=IfCondition(is_playground),
        actions=[stop_opp_cmd]
    )

    return LaunchDescription([
        map_name_arg, opponent_csv_arg,
        w_speed_arg, w_track_arg, w_comfort_arg, w_clearance_arg, w_dynamics_arg, d_min_arg,

        stop_cmd,
        conditional_stop_opp,
        
        # [수정] 1차 위치 초기화 (1.0초)
        TimerAction(period=1.0, actions=[static_path_node_1]),
        
        # [수정] 2차 위치 초기화 (2.5초) - 확실하게 적용되도록 재실행
        TimerAction(period=2.5, actions=[static_path_node_2]),

        # 다른 노드들은 위치가 확실히 잡힌 후 (3초 이후) 실행
        TimerAction(period=3.5, actions=[collision_node]),
        TimerAction(period=3.5, actions=[mux_logger_node]),
        
        TimerAction(period=4.5, actions=[frenet_launch, fgm_node]),
        TimerAction(period=5.5, actions=[opponent_pp_node]),
        TimerAction(period=5.5, actions=[mux_node, main_pp_node]),

        exit_on_logger,
        timeout_action
    ])

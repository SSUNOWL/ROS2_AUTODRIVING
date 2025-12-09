import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction, 
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
    GroupAction
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    exp_pkg = FindPackageShare('racecar_experiments')
    frenet_pkg = FindPackageShare('racecar_frenet_cpp')

    # ==============================
    # 1. Arguments Definition
    # ==============================
    
    # Environment Args
    map_name_arg = DeclareLaunchArgument('map_name', default_value='Spielberg')
    opponent_csv_arg = DeclareLaunchArgument('opponent_csv_filename', default_value='bumper_slow_1.csv')
    
    # Frenet Parameters
    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='6.0')
    target_speed_arg = DeclareLaunchArgument('target_speed', default_value='5.0')
    max_accel_arg = DeclareLaunchArgument('max_accel', default_value='5.0')
    max_curvature_arg = DeclareLaunchArgument('max_curvature', default_value='1.0')
    pp_max_speed_arg = DeclareLaunchArgument('pp_max_speed', default_value='6.0')

    # Logic Variables
    is_playground = PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'playground'"])
    
    # Paths
    # 맵 이름에 따라 자차 경로 파일 자동 선택 (예: raceline_Spielberg.csv)
    csv_filename = PythonExpression(["'raceline_' + '", LaunchConfiguration('map_name'), "' + '.csv'"])
    ego_csv_path = PathJoinSubstitution([os.getcwd(), csv_filename])
    opponent_csv_path = PathJoinSubstitution([os.getcwd(), LaunchConfiguration('opponent_csv_filename')])
    
    # Scenario Name Generation (로그 파일용)
    # Playground일 때와 아닐 때 이름 짓는 방식을 통합
    scenario_name_str = PythonExpression([
        "'FrenetUnified_' + '", LaunchConfiguration('map_name'), "' + ",
        "('_Opp_' + '", LaunchConfiguration('opponent_csv_filename'), "') if '", LaunchConfiguration('map_name'), "' == 'playground' else '' + ",
        "'_SPD' + '", LaunchConfiguration('max_speed'), "' + ",
        "'_TGT' + '", LaunchConfiguration('target_speed'), "' + ",
        "'_ACC' + '", LaunchConfiguration('max_accel'), "' + ",
        "'_CRV' + '", LaunchConfiguration('max_curvature'), "' + ",
        "'_PP' + '", LaunchConfiguration('pp_max_speed'), "'"
    ])

    # ==============================
    # 2. Nodes Configuration
    # ==============================

    # 2-1. Stop Commands (초기화)
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
    
    conditional_stop_opp = GroupAction(
        condition=IfCondition(is_playground),
        actions=[stop_opp_cmd]
    )

    # 2-2. Static Path Publisher (위치 초기화 핵심)
    # Mux 로직 차용: 1차(1.0s), 2차(2.5s) 발행으로 위치 고정
    static_path_node_1 = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher_1',
        output='screen',
        parameters=[{
            'csv_path': ego_csv_path,
            'opponent_csv_path': opponent_csv_path,
            'spawn_opponent_enabled': is_playground, # Playground면 상대차도 스폰
            'topic_name': '/plan', # Frenet은 내부적으로 경로를 다시 계산하지만, 초기 위치를 위해 필요
            'opponent_spawn_topic': '/goal_pose'
        }]
    )

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

    # 2-3. Opponent Controller (Playground 전용)
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

    # 2-4. Collision Monitor
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

    # 2-5. Logger (run_logger 사용)
    # Frenet 단독 주행이므로 Mux Logger 대신 run_logger가 적합
    logger_node = Node(
        package='racecar_experiments',
        executable='run_logger', 
        name='run_logger_node',
        output='screen',
        parameters=[{
            'planner_mode': 'FRENET_UNIFIED',
            'output_dir': os.path.join(os.getcwd(), 'logs'),
            'collision_topic': '/experiments/crash_detected',
            'scenario_name': scenario_name_str
        }]
    )

    # 2-6. Frenet Planner Launch
    # Frenet 알고리즘 + Pure Pursuit 실행
    # (주의: Mux는 frenet.launch.py를 쓰지만, 여기선 실행까지 해야하므로 frenet_with_pp 사용)
    frenet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([frenet_pkg, 'launch', 'frenet_with_pp.launch.py'])
        ),
        launch_arguments={
            'max_speed': LaunchConfiguration('max_speed'),
            'target_speed': LaunchConfiguration('target_speed'),
            'max_accel': LaunchConfiguration('max_accel'),
            'max_curvature': LaunchConfiguration('max_curvature'),
            'pp_max_speed': LaunchConfiguration('pp_max_speed'),
            # Frenet이 만든 경로를 따라가도록 설정
            'pp_use_frenet_path': 'true',
            'pp_frenet_path_topic': '/frenet_local_plan',
            # Mux가 아니므로 Drive Topic은 Ego 차량에 직접 연결
            'drive_topic': '/drive' 
        }.items()
    )

    # ==============================
    # 3. Shutdown Logic
    # ==============================
    
    exit_on_logger = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=logger_node,
            on_exit=[LogInfo(msg="Logger finished."), EmitEvent(event=Shutdown())]
        )
    )

    timeout_action = TimerAction(
        period=100.0,
        actions=[LogInfo(msg="Timeout reached."), EmitEvent(event=Shutdown())]
    )

    return LaunchDescription([
        map_name_arg, opponent_csv_arg,
        max_speed_arg, target_speed_arg, max_accel_arg, max_curvature_arg, pp_max_speed_arg,

        stop_cmd,
        conditional_stop_opp,
        
        # 1. 위치 초기화 (1.0초, 2.5초)
        TimerAction(period=1.0, actions=[static_path_node_1]),
        TimerAction(period=2.5, actions=[static_path_node_2]),

        # 2. 모니터링 및 로거 시작 (3.5초)
        TimerAction(period=3.5, actions=[collision_node, logger_node]),
        
        # 3. 주행 로직 시작 (4.5초)
        # 상대 차량(있다면)과 자차 Frenet 동시 시작
        TimerAction(period=4.5, actions=[opponent_pp_node, frenet_launch]),

        exit_on_logger,
        timeout_action
    ])
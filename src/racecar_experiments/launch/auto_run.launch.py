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

def generate_launch_description():
    exp_pkg = FindPackageShare('racecar_experiments')
    frenet_pkg = FindPackageShare('racecar_frenet_cpp')

    # --- Arguments ---
    map_name_arg = DeclareLaunchArgument('map_name', default_value='Spielberg')
    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='6.0')
    target_speed_arg = DeclareLaunchArgument('target_speed', default_value='5.0')
    max_accel_arg = DeclareLaunchArgument('max_accel', default_value='5.0')
    max_curvature_arg = DeclareLaunchArgument('max_curvature', default_value='1.0')

    # --- Path & Names ---
    csv_filename = PythonExpression(["'raceline_' + '", LaunchConfiguration('map_name'), "' + '.csv'"])
    csv_path = PathJoinSubstitution([os.getcwd(), csv_filename])
    
    scenario_name_str = PythonExpression([
        "'", LaunchConfiguration('map_name'), "_' + ",
        "'SPD' + '", LaunchConfiguration('max_speed'), "' + ",
        "'_TGT' + '", LaunchConfiguration('target_speed'), "' + ",
        "'_ACC' + '", LaunchConfiguration('max_accel'), "' + ",
        "'_CRV' + '", LaunchConfiguration('max_curvature'), "'"
    ])

    # --- Nodes ---

    # 1. 초기 정지 명령
    stop_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/drive', 'ackermann_msgs/msg/AckermannDriveStamped',
             "{header: {stamp: now, frame_id: ego_base_link}, drive: {steering_angle: 0.0, speed: 0.0}}"],
        output='screen'
    )

    # 2. Static Path Publisher
    static_path_node = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher',
        output='screen',
        parameters=[{'csv_path': csv_path}]
    )

    # 3. [추가됨] Collision Monitor (충돌 감지)
    # 이 노드가 실행되어야 충돌 시 /experiments/crash_detected 토픽이 발행됩니다.
    collision_node = Node(
        package='racecar_experiments',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[{
            'stop_threshold': 0.20,  # 정지 거리
            'go_threshold': 0.25     # 재개 거리 (노이즈 방지)
        }]
    )

    # 4. Logger Node (run_logger)
    logger_node = Node(
        package='racecar_experiments',
        executable='run_logger', 
        name='run_logger_node',
        output='screen',
        parameters=[{
            'planner_mode': 'FRENET',
            'output_dir': os.path.join(os.getcwd(), 'logs'),
            'collision_topic': '/experiments/crash_detected',
            'scenario_name': scenario_name_str
        }]
    )

    # 5. Planner Launch (Frenet + PP)
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([frenet_pkg, 'launch', 'frenet_with_pp.launch.py'])
        ),
        launch_arguments={
            'max_speed': LaunchConfiguration('max_speed'),
            'target_speed': LaunchConfiguration('target_speed'),
            'max_accel': LaunchConfiguration('max_accel'),
            'max_curvature': LaunchConfiguration('max_curvature'),
            'pp_max_speed': LaunchConfiguration('max_speed'),
            'pp_use_frenet_path': 'true',
            'pp_frenet_path_topic': '/frenet_local_plan'
        }.items()
    )

    # --- Event Handlers (종료 조건) ---

    # 조건 A: Logger 노드가 종료되면(완주/충돌) -> 전체 Shutdown
    exit_on_logger_finish = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=logger_node,
            on_exit=[
                LogInfo(msg="Logger finished. Experiment Complete!"),
                EmitEvent(event=Shutdown())
            ]
        )
    )

    # 조건 B: 100초 타임아웃
    timeout_action = TimerAction(
        period=100.0,
        actions=[
            LogInfo(msg="Timeout (100s) reached. Force Shutdown."),
            EmitEvent(event=Shutdown())
        ]
    )

    # --- Execution Sequence ---
    
    # T=1.0: 경로 발행
    seq_1_path = TimerAction(period=1.0, actions=[static_path_node])
    
    # T=3.0: 모니터링 시스템 시작 (Logger + Collision Monitor)
    # [수정됨] 충돌 모니터도 이때 같이 켜서 감시를 시작합니다.
    seq_2_monitor = TimerAction(period=3.0, actions=[logger_node, collision_node])
    
    # T=5.0: 주행 시작 (Planner)
    seq_3_planner = TimerAction(period=5.0, actions=[planner_launch])

    return LaunchDescription([
        map_name_arg, max_speed_arg, target_speed_arg, max_accel_arg, max_curvature_arg,
        stop_cmd,
        seq_1_path,
        seq_2_monitor, # Logger + Collision Monitor 실행
        seq_3_planner,
        exit_on_logger_finish,
        timeout_action
    ])
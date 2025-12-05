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

def generate_launch_description():
    # ==============================
    # 1. 패키지 경로 설정
    # ==============================
    exp_pkg = FindPackageShare('racecar_experiments')
    frenet_pkg = FindPackageShare('racecar_frenet_cpp')
    planner_pkg = FindPackageShare('f1tenth_planner') # FGM 패키지

    # ==============================
    # 2. Arguments Definition
    # ==============================
    # 공통 설정
    map_name_arg = DeclareLaunchArgument('map_name', default_value='playground')
    
    # Mux Weights (가변 파라미터)
    w_speed_arg = DeclareLaunchArgument('w_speed', default_value='1.0')
    w_track_arg = DeclareLaunchArgument('w_track', default_value='1.0')
    w_comfort_arg = DeclareLaunchArgument('w_comfort', default_value='1.0')
    w_clearance_arg = DeclareLaunchArgument('w_clearance', default_value='1.0')
    w_dynamics_arg = DeclareLaunchArgument('w_dynamics', default_value='1.0')

    # Opponent 설정 (Playground용)
    opponent_csv_arg = DeclareLaunchArgument('opponent_csv_filename', default_value='bumper_slow_1.csv')

    # 조건부 실행 변수
    is_playground = PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'playground'"])
    
    # 시나리오 이름 생성 (로그 파일용)
    scenario_name_str = PythonExpression([
        "'Mux_' + str(", LaunchConfiguration('map_name'), ") + "
        "'_WS' + str(", LaunchConfiguration('w_speed'), ") + "
        "'_WT' + str(", LaunchConfiguration('w_track'), ") + "
        "'_WC' + str(", LaunchConfiguration('w_comfort'), ") + "
        "'_WCL' + str(", LaunchConfiguration('w_clearance'), ") + "
        "'_WD' + str(", LaunchConfiguration('w_dynamics'), ") + "
        "'_Opp_' + str(", LaunchConfiguration('opponent_csv_filename'), ")"
    ])


    # 경로 파일 설정
    csv_filename = PythonExpression(["'raceline_' + '", LaunchConfiguration('map_name'), "' + '.csv'"])
    ego_csv_path = PathJoinSubstitution([os.getcwd(), csv_filename])
    opponent_csv_path = PathJoinSubstitution([os.getcwd(), LaunchConfiguration('opponent_csv_filename')])

    # ==============================
    # 3. Nodes Configuration
    # ==============================

    # 3-1. 초기 정지
    stop_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/drive', 'ackermann_msgs/msg/AckermannDriveStamped',
             "{header: {stamp: now, frame_id: ego_base_link}, drive: {steering_angle: 0.0, speed: 0.0}}"],
        output='screen'
    )

    # 3-2. Static Path Publisher
    static_path_node = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher',
        output='screen',
        parameters=[{
            'csv_path': ego_csv_path,
            'opponent_csv_path': opponent_csv_path,
            'spawn_opponent_enabled': is_playground,
            'topic_name': '/plan',
            'opponent_spawn_topic': '/goal_pose'
        }]
    )

    # 3-3. Opponent Controller (Playground Only)
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

    # 3-4. Collision Monitor
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

    # 3-5. Loggers (조건부)
    run_logger_node = Node(
        condition=UnlessCondition(is_playground),
        package='racecar_experiments',
        executable='run_logger',
        name='run_logger_node',
        output='screen',
        parameters=[{
            'planner_mode': 'MUX_RACING',
            'output_dir': os.path.join(os.getcwd(), 'logs'),
            'collision_topic': '/experiments/crash_detected',
            'scenario_name': scenario_name_str
        }]
    )

    avoid_logger_node = Node(
        condition=IfCondition(is_playground),
        package='racecar_experiments',
        executable='avoid_logger',
        name='avoid_logger_node',
        output='screen',
        parameters=[{
            'planner_mode': 'MUX_OBSTACLE',
            'output_dir': os.path.join(os.getcwd(), 'logs'),
            'collision_topic': '/experiments/crash_detected',
            'scenario_name': scenario_name_str
        }]
    )

    # ==============================
    # 4. Planners (Background)
    # ==============================

    # 4-1. Frenet Planner (Background)
    # [중요] Frenet 내부의 PP가 차를 움직이지 않도록 drive 토픽을 쓰레기통(/trash)으로 보냄
    # Frenet은 '/frenet_local_plan' (Path)만 잘 발행하면 됨
    frenet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([frenet_pkg, 'launch', 'frenet.launch.py'])
        ),
        # [핵심] 여기서 /drive를 Remap하여 실제 주행 간섭 방지
        # launch 파일 내부 구조에 따라 remap 방식이 다를 수 있으나, 
        # 일반적으로 Node 레벨 Remap이 우선 적용됨을 가정
    )

    # 4-2. FGM Planner (Background)
    # [중요] FGM 내부의 구동 명령 무시. '/fgm_path' (Path)만 필요함.
    fgm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([planner_pkg, 'launch', 'fgm.launch.py'])
        )
    )

    # ==============================
    # 5. Mux & Main Controller
    # ==============================

    # 5-1. Mux Node
    # 입력: /frenet_local_plan, /fgm_path
    # 출력: /selected_path
    mux_node = Node(
        package='planner_mux',
        executable='planner_mux_node', # CMakeLists.txt의 실행 파일명 확인 필요
        name='local_planner_mux',
        output='screen',
        parameters=[{
            'w_speed': LaunchConfiguration('w_speed'),
            'w_track': LaunchConfiguration('w_track'),
            'w_comfort': LaunchConfiguration('w_comfort'),
            'w_clearance': LaunchConfiguration('w_clearance'),
            'w_dynamics': LaunchConfiguration('w_dynamics'),
            # Topic 이름이 cpp 기본값과 같다면 생략 가능
            # 'd_min': 0.15, 
            # 'v_ref': 5.0,
        }]
    )

    # 5-2. Main Pure Pursuit Node (Final Executor)
    # 입력: /selected_path (Mux가 선택한 경로)
    # 출력: /drive (실제 차량 제어)
    main_pp_node = Node(
        package='f1tenth_planner',
        executable='pure_pursuit_node', # 혹은 pp_node 등 실제 이름
        name='mux_pure_pursuit',
        output='screen',
        parameters=[{
            'path_topic': '/selected_path',   # [핵심] Mux의 출력 경로를 구독
            'drive_topic': '/drive',          # 실제 구동 토픽
            'lookahead_dist': 1.0,            # 튜닝 필요
            'max_speed': 5.5,                 # Mux에서 속도 프로파일을 생성하므로 충분히 크게
            'visualize_lookahead': True
        }],
        # 만약 frenet/fgm 런치파일이 /drive를 강제로 잡고 있다면
        # 여기서 우선순위 문제가 생길 수 있으므로, 4번 단계에서 Remap이 필수적임.
    )

    # ==============================
    # 6. 종료 조건 및 순서
    # ==============================

    exit_on_run_logger = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=run_logger_node,
            on_exit=[LogInfo(msg="Run Logger finished."), EmitEvent(event=Shutdown())]
        )
    )
    exit_on_avoid_logger = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=avoid_logger_node,
            on_exit=[LogInfo(msg="Avoid Logger finished."), EmitEvent(event=Shutdown())]
        )
    )

    timeout_action = TimerAction(
        period=100.0,
        actions=[LogInfo(msg="Timeout reached."), EmitEvent(event=Shutdown())]
    )

    return LaunchDescription([
        map_name_arg, opponent_csv_arg,
        w_speed_arg, w_track_arg, w_comfort_arg, w_clearance_arg, w_dynamics_arg,
        stop_cmd,
        
        TimerAction(period=1.0, actions=[static_path_node]),
        TimerAction(period=3.0, actions=[collision_node]),
        TimerAction(period=3.0, actions=[run_logger_node, avoid_logger_node]),
        TimerAction(period=4.0, actions=[opponent_pp_node]),
        
        # Planners (Path 생성용)
        TimerAction(period=5.0, actions=[frenet_launch, fgm_launch]),
        
        # Controller (Mux -> PP)
        TimerAction(period=6.0, actions=[mux_node, main_pp_node]),

        exit_on_run_logger,
        exit_on_avoid_logger,
        timeout_action
    ])
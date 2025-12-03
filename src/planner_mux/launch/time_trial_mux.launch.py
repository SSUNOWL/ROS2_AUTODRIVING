import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction, 
    GroupAction,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler, # [New] 이벤트 핸들러 등록
    EmitEvent             # [New] 이벤트 발생 (Shutdown 용)
)
from launch.event_handlers import OnProcessExit # [New] 프로세스 종료 이벤트 감지
from launch.events import Shutdown             # [New] 시스템 종료 이벤트
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # --- Context에서 Argument 값 읽기 ---
    mode = LaunchConfiguration('mode').perform(context)
    profile = LaunchConfiguration('profile').perform(context).lower()
    track = LaunchConfiguration('track').perform(context).lower()
    ego_csv = LaunchConfiguration('ego_csv').perform(context)
    opp_csv = LaunchConfiguration('opp_csv').perform(context)
    pp_topic = LaunchConfiguration('pp_path_topic').perform(context)
    log_dir = LaunchConfiguration('log_dir').perform(context)

    # --- 프로필별 파라미터 설정 (Frenet, FGM, PP 통합 관리) ---
    if profile == 'race':
        max_speed = 6.2
        fgm_lookahead = 3.5
        pp_lookahead_gain = 0.4
        frenet_params = {
            'max_speed': 6.2, 'target_speed': 5.1, 
            'max_accel': 5.0, 'max_curvature': 1.0
        }
    elif profile == 'extreme':
        max_speed = 7.0
        fgm_lookahead = 4.0
        pp_lookahead_gain = 0.35
        frenet_params = {
            'max_speed': 7.0, 'target_speed': 5.5, 
            'max_accel': 6.0, 'max_curvature': 1.2
        }
    else: # balanced
        max_speed = 4.5
        fgm_lookahead = 2.5
        pp_lookahead_gain = 0.5
        frenet_params = {
            'max_speed': 4.5, 'target_speed': 4.0, 
            'max_accel': 3.0, 'max_curvature': 0.8
        }

    # --- Mode 설정 ---
    spawn_opponent = (mode == '2')

    # -----------------------------------------------------
    # 노드 구성
    # -----------------------------------------------------
    frenet_pkg = FindPackageShare('racecar_frenet_cpp')
    
    # 0. 초기 정지 명령
    ego_stop_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/drive', 
             'ackermann_msgs/msg/AckermannDriveStamped', 
             "{header: {stamp: now, frame_id: ego_base_link}, drive: {speed: 0.0}}"],
        output='screen'
    )

    # 1. Static Path Publisher
    static_path_node = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher',
        output='screen',
        parameters=[{
            'csv_path': ego_csv,
            'spawn_opponent_enabled': spawn_opponent,
            'opponent_csv_path': opp_csv,
            'topic_name': '/plan',
            'opponent_spawn_topic': '/goal_pose'
        }]
    )

    # 1-1. Run Logger (데이터 로깅)
    scenario_str = f"{track}_{profile}_mode{mode}"
    
    if not os.path.exists(log_dir):
        try:
            os.makedirs(log_dir)
        except OSError:
            pass 

    run_logger_node = Node(
        package='racecar_experiments',
        executable='run_logger',
        name='run_logger',
        output='screen',
        parameters=[{
            'output_dir': log_dir,
            'scenario_name': scenario_str,
            'planner_mode': 'MUX',
            'goal_tolerance': 0.5,
            'stuck_timeout': 5.0,       # [New] 고립 시간 (기본 5초)
            'stuck_dist_thresh': 0.2    # [New] 고립 판단 반경 (기본 0.2m)
        }]
    )

    # 2. Frenet Planner
    frenet_node_override = Node(
        package='racecar_frenet_cpp',
        executable='frenet_local_planner',
        name='frenet_local_planner',
        output='screen',
        parameters=[
            PathJoinSubstitution([frenet_pkg, 'config', 'params.yaml']),
            frenet_params
        ]
    )

    # 3. FGM Node
    fgm_node = Node(
        package='f1tenth_planner',
        executable='fgm_node',
        name='fgm_node',
        output='screen',
        parameters=[{
            'planning_distance': 3.0,
            'gap_threshold': 1.0,
            'bubble_radius': 0.45,
            'fov_angle': 180.0,
            'max_speed': max_speed,
            'min_speed': 2.0,
            'max_lookahead_dist': fgm_lookahead,
            'slow_down_dist': 2.5
        }]
    )

    # 4. MUX Node (가중치 파라미터 적용)
    mux_node = Node(
        package='planner_mux',
        executable='planner_mux_node',
        name='planner_mux_node',
        output='screen',
        parameters=[{
            'max_speed_mux': max_speed + 2.0,
            'w_speed': LaunchConfiguration('mux_w_speed'),
            'w_track': LaunchConfiguration('mux_w_track'),
            'w_comfort': LaunchConfiguration('mux_w_comfort'),
            'w_safety': LaunchConfiguration('mux_w_safety'),
        }]
    )

    # 5. Pure Pursuit
    pure_pursuit_node = Node(
        package='f1tenth_planner',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'use_frenet_path': True,
            'lookahead_gain': pp_lookahead_gain,
            'lookahead_min': 1.0,
            'max_speed': max_speed,
            'frenet_path_topic': pp_topic
        }],
        remappings=[
            ('/ego_racecar/odom', '/ego_racecar/odom')
        ]
    )

    # 6. Collision Monitor
    collision_node = Node(
        package='racecar_experiments',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[{'stop_threshold': 0.20, 'go_threshold': 0.25}]
    )

    # --- [NEW] 종료 이벤트 핸들러 ---
    # Run Logger가 종료되면(Exit) 시스템 전체를 셧다운(Shutdown)시킴
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=run_logger_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    # --- 실행 순서 제어 ---
    return [
        TimerAction(period=0.0, actions=[ego_stop_cmd]),
        TimerAction(period=1.0, actions=[static_path_node, run_logger_node]),
        TimerAction(period=5.0, actions=[
            frenet_node_override,
            fgm_node,
            mux_node,
            pure_pursuit_node,
            collision_node
        ]),
        # 이벤트 핸들러 등록
        shutdown_handler
    ]

def generate_launch_description():
    return LaunchDescription([
        # 1. 기존 설정들
        DeclareLaunchArgument('mode', default_value='1', description='1: Solo, 2: Vs Opponent'),
        DeclareLaunchArgument('profile', default_value='balanced', description='Profile: balanced|race|extreme'),
        DeclareLaunchArgument('track', default_value='spielberg', description='Track Name'),
        DeclareLaunchArgument('pp_path_topic', default_value='/selected_path', description='Topic for PP'),
        DeclareLaunchArgument('ego_csv', default_value=os.path.join(os.getcwd(), 'raceline_with_yaw.csv')),
        DeclareLaunchArgument('opp_csv', default_value=os.path.join(os.getcwd(), 'opponent_raceline.csv')),
        DeclareLaunchArgument('log_dir', default_value=os.path.join(os.getcwd(), 'logs')),

        # [NEW] MUX 가중치 파라미터 (명령어로 수정 가능)
        DeclareLaunchArgument('mux_w_speed', default_value='5.0', description='Weight for speed optimization'),
        DeclareLaunchArgument('mux_w_track', default_value='2.0', description='Weight for tracking reference path'),
        DeclareLaunchArgument('mux_w_comfort', default_value='0.0', description='Weight for comfort (jerk min)'),
        DeclareLaunchArgument('mux_w_safety', default_value='0.5', description='Weight for safety (obs avoidance)'),

        OpaqueFunction(function=launch_setup)
    ])
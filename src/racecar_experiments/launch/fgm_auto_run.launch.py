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
    # =========================================================
    # 1. 패키지 설정
    # =========================================================
    planner_pkg = FindPackageShare('f1tenth_planner') 
    exp_pkg = FindPackageShare('racecar_experiments')

    # ==============================
    # 2. Arguments Definition
    # ==============================

    map_name_arg = DeclareLaunchArgument('map_name', default_value='playground')
    opponent_csv_arg = DeclareLaunchArgument('opponent_csv_filename', default_value='bumper_slow_1.csv')
    
    # Pure Pursuit Max Speed
    pp_max_speed_arg = DeclareLaunchArgument('pp_max_speed', default_value='4.0')

    # FGM Tuning Parameters
    fgm_gap_threshold_arg = DeclareLaunchArgument('fgm_gap_threshold', default_value='1.2')
    fgm_bubble_radius_arg = DeclareLaunchArgument('fgm_bubble_radius', default_value='0.5')
    fgm_fov_angle_arg = DeclareLaunchArgument('fgm_fov_angle', default_value='180.0')
    fgm_speed_check_fov_deg_arg = DeclareLaunchArgument('fgm_speed_check_fov_deg', default_value='25.0')
    fgm_required_clearance_arg = DeclareLaunchArgument('fgm_required_clearance', default_value='0.55')
    fgm_width_weight_arg = DeclareLaunchArgument('fgm_width_weight', default_value='0.6')
    fgm_angle_weight_arg = DeclareLaunchArgument('fgm_angle_weight', default_value='6.0')
    fgm_steer_weight_arg = DeclareLaunchArgument('fgm_steer_weight', default_value='0.08')
    fgm_hysteresis_bonus_arg = DeclareLaunchArgument('fgm_hysteresis_bonus', default_value='2.0')
    fgm_change_threshold_arg = DeclareLaunchArgument('fgm_change_threshold', default_value='0.25')
    fgm_smoothing_alpha_arg = DeclareLaunchArgument('fgm_smoothing_alpha', default_value='0.5')
    fgm_dynamic_bubble_speed_coeff_arg = DeclareLaunchArgument('fgm_dynamic_bubble_speed_coeff', default_value='0.12')

    # ==============================
    # 3. Path & Logic Generation (Mux와 환경 일치)
    # ==============================
    
    # [수정 1] 맵 이름에 따라 Ego Path CSV 파일 자동 선택 (예: raceline_Spielberg.csv)
    # 기존: 무조건 raceline_playground.csv 였음
    csv_filename = PythonExpression(["'raceline_' + '", LaunchConfiguration('map_name'), "' + '.csv'"])
    ego_csv_path = PathJoinSubstitution([os.getcwd(), csv_filename])
    
    opponent_csv_path = PathJoinSubstitution([os.getcwd(), LaunchConfiguration('opponent_csv_filename')])

    # [수정 2] Playground 여부 판단 변수 (Mux와 동일)
    is_playground = PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'playground'"])

    # 시나리오 이름 생성 (로그용)
    # [수정 3] 맵 이름을 시나리오 이름 맨 앞에 추가
    scenario_name_str = PythonExpression([
        "'FGM_' + '", LaunchConfiguration('map_name'), "' + ",
        "'_Opp_' + '", LaunchConfiguration('opponent_csv_filename'), "' + ",
        "'_FFG' + '", LaunchConfiguration('fgm_fov_angle'), "' +",
        "'_SCD' + '", LaunchConfiguration('fgm_speed_check_fov_deg'), "' +",
        "'_Gap' + '", LaunchConfiguration('fgm_gap_threshold'), "' + ",
        "'_Bub' + '", LaunchConfiguration('fgm_bubble_radius'), "' + ",
        "'_Clr' + '", LaunchConfiguration('fgm_required_clearance'), "' + ",
        "'_WW' + '", LaunchConfiguration('fgm_width_weight'), "' + ",
        "'_AW' + '", LaunchConfiguration('fgm_angle_weight'), "' + ",
        "'_SW' + '", LaunchConfiguration('fgm_steer_weight'), "' + ",
        "'_HB' + '", LaunchConfiguration('fgm_hysteresis_bonus'), "' + ",
        "'_CT' + '", LaunchConfiguration('fgm_change_threshold'), "' + ",
        "'_SA' + '", LaunchConfiguration('fgm_smoothing_alpha'), "' + ",
        "'_DBS' + '", LaunchConfiguration('fgm_dynamic_bubble_speed_coeff'), "' + ",
        "'_SPD' + '", LaunchConfiguration('pp_max_speed'), "'"
    ])

    # ==============================
    # 4. Nodes Configuration
    # ==============================

    # 4-1. 초기 정지 (자차)
    stop_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/drive', 'ackermann_msgs/msg/AckermannDriveStamped',
             "{header: {stamp: now, frame_id: ego_base_link}, drive: {steering_angle: 0.0, speed: 0.0}}"],
        output='screen'
    )

    # [수정 4] 상대 차량 정지는 Playground일 때만 실행 (Mux와 동일)
    stop_opp_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/opp_drive', 'ackermann_msgs/msg/AckermannDriveStamped',
             "{header: {stamp: now, frame_id: opp_base_link}, drive: {steering_angle: 0.0, speed: 0.0}}"],
        output='screen'
    )
    conditional_stop_opp = GroupAction(
        condition=IfCondition(is_playground),
        actions=[stop_opp_cmd]
    )

    # 4-2. Static Path Publisher
    static_path_node = Node(
        package='racecar_experiments',
        executable='static_path_publisher',
        name='static_path_publisher',
        output='screen',
        parameters=[{
            'csv_path': ego_csv_path,          # [수정] 동적 경로 적용됨
            'opponent_csv_path': opponent_csv_path,
            'spawn_opponent_enabled': is_playground, # [수정] Playground일 때만 상대차 스폰
            'topic_name': '/plan',
            'opponent_spawn_topic': '/goal_pose'
        }]
    )

    # 4-3. Opponent Controller
    opponent_pp_node = Node(
        condition=IfCondition(is_playground), # [수정] Playground일 때만 실행
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

    # 4-4. Collision Monitor
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

    # 4-5. Logger (FGM 전용 avoid_logger 유지하되 환경 일치)
    logger_node = Node(
        package='racecar_experiments',
        executable='avoid_logger', 
        name='avoid_logger_node',
        output='screen',
        parameters=[{
            'planner_mode': 'FGM',
            'output_dir': os.path.join(os.getcwd(), 'logs'),
            'collision_topic': '/experiments/crash_detected',
            'scenario_name': scenario_name_str,
            'goal_tolerance': 1.0 # Mux와 비슷하게 조정
        }]
    )

    # 4-6. Main Planner (FGM + PP)
    # fgm_with_pp.launch.py 내부에서 pure_pursuit가 /fgm_path를 따라가도록 설정되어 있음
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([planner_pkg, 'launch', 'fgm_with_pp.launch.py'])
        ),
        launch_arguments={
            'fgm_gap_threshold': LaunchConfiguration('fgm_gap_threshold'),
            'fgm_bubble_radius': LaunchConfiguration('fgm_bubble_radius'),
            'fgm_fov_angle': LaunchConfiguration('fgm_fov_angle'),
            'fgm_speed_check_fov_deg': LaunchConfiguration('fgm_speed_check_fov_deg'),
            'fgm_required_clearance': LaunchConfiguration('fgm_required_clearance'),
            'fgm_width_weight': LaunchConfiguration('fgm_width_weight'),
            'fgm_angle_weight': LaunchConfiguration('fgm_angle_weight'),
            'fgm_steer_weight': LaunchConfiguration('fgm_steer_weight'),
            'fgm_hysteresis_bonus': LaunchConfiguration('fgm_hysteresis_bonus'),
            'fgm_change_threshold': LaunchConfiguration('fgm_change_threshold'),
            'fgm_smoothing_alpha': LaunchConfiguration('fgm_smoothing_alpha'),
            'fgm_dynamic_bubble_speed_coeff': LaunchConfiguration('fgm_dynamic_bubble_speed_coeff'),
            
            'pp_max_speed': LaunchConfiguration('pp_max_speed'),
            'pp_csv_path': ego_csv_path # [수정] 동적 경로 전달
        }.items()
    )

    # ==============================
    # 5. 종료 조건
    # ==============================

    # 조건 A: Logger 종료 (완주/충돌)
    exit_on_logger_finish = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=logger_node,
            on_exit=[LogInfo(msg="Logger finished."), EmitEvent(event=Shutdown())]
        )
    )

    # 조건 B: 타임아웃
    timeout_action = TimerAction(
        period=100.0, # Mux와 동일하게 100초로 여유 있게 설정
        actions=[LogInfo(msg="Timeout (100s) reached. Force Shutdown."), EmitEvent(event=Shutdown())]
    )

    # ==============================
    # 6. 실행 순서
    # ==============================
    
    seq_1 = TimerAction(period=1.0, actions=[static_path_node])
    seq_2 = TimerAction(period=3.0, actions=[logger_node, collision_node])
    # Opponent node는 Timer가 지나도 condition이 False면 실행되지 않음
    seq_3 = TimerAction(period=4.0, actions=[opponent_pp_node]) 
    seq_4 = TimerAction(period=5.0, actions=[planner_launch])

    return LaunchDescription([
        map_name_arg, opponent_csv_arg, pp_max_speed_arg,
        fgm_gap_threshold_arg, fgm_bubble_radius_arg, fgm_fov_angle_arg,
        fgm_speed_check_fov_deg_arg, fgm_required_clearance_arg, fgm_width_weight_arg,
        fgm_angle_weight_arg, fgm_steer_weight_arg, fgm_hysteresis_bonus_arg,
        fgm_change_threshold_arg, fgm_smoothing_alpha_arg, fgm_dynamic_bubble_speed_coeff_arg,
        
        stop_cmd,
        conditional_stop_opp, # [수정] 조건부 실행
        seq_1, seq_2, seq_3, seq_4,
        exit_on_logger_finish,
        timeout_action
    ])
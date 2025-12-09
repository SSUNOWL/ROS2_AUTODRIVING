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
    planner_pkg = FindPackageShare('f1tenth_planner') 
    exp_pkg = FindPackageShare('racecar_experiments')

    # ==============================
    # 1. Arguments Definition
    # ==============================

    map_name_arg = DeclareLaunchArgument('map_name', default_value='playground')
    opponent_csv_arg = DeclareLaunchArgument('opponent_csv_filename', default_value='bumper_slow_1.csv')
    pp_max_speed_arg = DeclareLaunchArgument('pp_max_speed', default_value='4.0')

    # FGM Parameters
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

    # Logic Variables
    is_playground = PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'playground'"])

    # Path Generation
    csv_filename = PythonExpression(["'raceline_' + '", LaunchConfiguration('map_name'), "' + '.csv'"])
    ego_csv_path = PathJoinSubstitution([os.getcwd(), csv_filename])
    opponent_csv_path = PathJoinSubstitution([os.getcwd(), LaunchConfiguration('opponent_csv_filename')])

    # Scenario Name (Log File)
    # Playground일 때만 Opponent 이름을 로그에 포함
    scenario_name_str = PythonExpression([
        "'FGMUnified_' + '", LaunchConfiguration('map_name'), "' + ",
        "('_Opp_' + '", LaunchConfiguration('opponent_csv_filename'), "') if '", LaunchConfiguration('map_name'), "' == 'playground' else '' + ",
        "'_Gap' + '", LaunchConfiguration('fgm_gap_threshold'), "' + ",
        "'_Bub' + '", LaunchConfiguration('fgm_bubble_radius'), "' + ",
        "'_SPD' + '", LaunchConfiguration('pp_max_speed'), "'"
    ])

    # ==============================
    # 2. Nodes Configuration
    # ==============================

    # 2-1. 초기 정지 및 Opponent 정지
    stop_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/drive', 'ackermann_msgs/msg/AckermannDriveStamped',
             "{header: {stamp: now, frame_id: ego_base_link}, drive: {steering_angle: 0.0, speed: 0.0}}"],
        output='screen'
    )
    
    stop_opp_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/opp_drive', 'ackermann_msgs/msg/AckermannDriveStamped',
             "{header: {stamp: now, frame_id: opp_base_link}, drive: {steering_angle: 0.0, speed: 0.0}}"],
        output='screen'
    )
    conditional_stop_opp = GroupAction(
        condition=IfCondition(is_playground),
        actions=[stop_opp_cmd]
    )

    # 2-2. Static Path Publisher (안정적인 위치 초기화를 위한 2회 실행)
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

    # 2-3. Opponent Controller (Playground Only)
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

    # 2-5. Logger (Avoid Logger 사용)
    logger_node = Node(
        package='racecar_experiments',
        executable='avoid_logger', 
        name='avoid_logger_node',
        output='screen',
        parameters=[{
            'planner_mode': 'FGM_UNIFIED',
            'output_dir': os.path.join(os.getcwd(), 'logs'),
            'collision_topic': '/experiments/crash_detected',
            'scenario_name': scenario_name_str,
            'goal_tolerance': 1.0
        }]
    )

    # 2-6. FGM Planner Launch
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
            'pp_csv_path': ego_csv_path 
        }.items()
    )

    # ==============================
    # 3. Execution Sequence
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
        map_name_arg, opponent_csv_arg, pp_max_speed_arg,
        fgm_gap_threshold_arg, fgm_bubble_radius_arg, fgm_fov_angle_arg,
        fgm_speed_check_fov_deg_arg, fgm_required_clearance_arg, fgm_width_weight_arg,
        fgm_angle_weight_arg, fgm_steer_weight_arg, fgm_hysteresis_bonus_arg,
        fgm_change_threshold_arg, fgm_smoothing_alpha_arg, fgm_dynamic_bubble_speed_coeff_arg,

        stop_cmd,
        conditional_stop_opp,
        
        # 1. 위치 초기화 (2회)
        TimerAction(period=1.0, actions=[static_path_node_1]),
        TimerAction(period=2.5, actions=[static_path_node_2]),

        # 2. 모니터링 시작
        TimerAction(period=3.5, actions=[logger_node, collision_node]),
        
        # 3. 주행 시작 (Opponent + FGM)
        TimerAction(period=4.5, actions=[opponent_pp_node, planner_launch]),

        exit_on_logger,
        timeout_action
    ])
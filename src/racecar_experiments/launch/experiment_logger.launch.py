import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # 1. 패키지 이름 정의 (실제 패키지 이름으로 대체해야 함)
    # 예시: 'my_racecar_package'를 사용합니다.
    package_name = 'racecar_experiments' 

    # 2. Launch Argument 정의
    # 로깅 파일 저장 경로
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.join(os.getcwd(), 'experiment_logs'),
        description='Directory to save experiment log files.'
    )
    # 시나리오 이름
    scenario_name_arg = DeclareLaunchArgument(
        'scenario_name',
        default_value='f1_track_test',
        description='Name of the current driving scenario.'
    )
    # 플래너 모드
    planner_mode_arg = DeclareLaunchArgument(
        'planner_mode',
        default_value='LATTICE',
        description='Planner mode (e.g., LATTICE, FRENET, PURE_PURSUIT).'
    )
    
    # [추가] 충돌 토픽 파라미터 정의 (CollisionMonitor의 출력 토픽과 일치시킵니다)
    collision_topic_arg = DeclareLaunchArgument(
        'collision_topic',
        default_value='/experiments/crash_detected',
        description='Topic name for collision signal (std_msgs/Bool).'
    )
    
    # 파라미터 변수 설정
    output_dir = LaunchConfiguration('output_dir')
    scenario_name = LaunchConfiguration('scenario_name')
    planner_mode = LaunchConfiguration('planner_mode')
    collision_topic = LaunchConfiguration('collision_topic')

    # 3. 노드 정의

    # A. RunLogger 노드 (로깅 및 자동 종료)
    run_logger_node = Node(
        package=package_name,
        executable='run_logger', # 실행 파일 이름 (run_logger.cpp의 main 함수)
        name='run_logger',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'scenario_name': scenario_name,
            'planner_mode': planner_mode,
            'collision_topic': collision_topic, # 파라미터로 충돌 토픽 설정
        }]
    )


    # C. CollisionMonitor 노드 (충돌 감지 신호 발행)
    # CollisionMonitor는 '/experiments/crash_detected'를 발행하며, 
    # RunLogger는 이제 이 토픽을 구독하도록 설정되었습니다.
    collision_monitor_node = Node(
        package=package_name,
        executable='collision_monitor', # 실행 파일 이름 (collision_monitor.cpp의 main 함수)
        name='collision_monitor',
        output='screen',
        parameters=[{
            'stop_threshold': 0.20,
            'go_threshold': 0.25,
        }],
        # RunLogger에서 collision_topic 파라미터로 처리했으므로, 
        # CollisionMonitor 노드의 remap은 필요 없습니다.
        # CollisionMonitor는 '/experiments/crash_detected'를 발행하고, 
        # RunLogger는 파라미터를 통해 이를 구독합니다.
    )


    # 4. LaunchDescription 반환
    return LaunchDescription([
        # Launch Arguments
        output_dir_arg,
        scenario_name_arg,
        planner_mode_arg,
        collision_topic_arg, # 새로 추가된 파라미터

        # Nodes
        run_logger_node,
        collision_monitor_node,
    ])
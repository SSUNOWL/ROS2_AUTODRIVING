import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_directory = get_package_share_directory('racecar_frenet_cpp')

    # === Frenet 파라미터 (Launch 인자) ===
    max_speed_arg      = DeclareLaunchArgument(
        'max_speed', default_value='5.0', description='Frenet max speed [m/s]')
    max_accel_arg      = DeclareLaunchArgument(
        'max_accel', default_value='4.0', description='Frenet max accel [m/s^2]')
    max_curvature_arg  = DeclareLaunchArgument(
        'max_curvature', default_value='1.0', description='Frenet max curvature [1/m]')
    s_horizon_arg      = DeclareLaunchArgument(
        's_horizon', default_value='15.0', description='Planning horizon length (if used)')

    # === Pure Pursuit 파라미터 (Launch 인자) ===
    pp_max_speed_arg = DeclareLaunchArgument(
        'pp_max_speed', default_value='4.0', description='Pure Pursuit max speed [m/s]')
    pp_lookahead_min_arg = DeclareLaunchArgument(
        'pp_lookahead_min', default_value='1.0', description='Pure Pursuit min lookahead [m]')
    pp_lookahead_gain_arg = DeclareLaunchArgument(
        'pp_lookahead_gain', default_value='0.3', description='Pure Pursuit lookahead gain')
    pp_csv_path_arg = DeclareLaunchArgument(
        'pp_csv_path', default_value='raceline_with_speed.csv',
        description='CSV path for Pure Pursuit (when not using Frenet path)')
    pp_use_frenet_path_arg = DeclareLaunchArgument(
        'pp_use_frenet_path', default_value='true',
        description='If true, subscribe Frenet path instead of CSV')
    pp_frenet_topic_arg = DeclareLaunchArgument(
        'pp_frenet_path_topic', default_value='/frenet_local_plan',
        description='Topic name for Frenet path (nav_msgs/Path)')

    # === LaunchConfiguration 객체 ===
    max_speed      = LaunchConfiguration('max_speed')
    max_accel      = LaunchConfiguration('max_accel')
    max_curvature  = LaunchConfiguration('max_curvature')
    s_horizon      = LaunchConfiguration('s_horizon')

    pp_max_speed       = LaunchConfiguration('pp_max_speed')
    pp_lookahead_min   = LaunchConfiguration('pp_lookahead_min')
    pp_lookahead_gain  = LaunchConfiguration('pp_lookahead_gain')
    pp_csv_path        = LaunchConfiguration('pp_csv_path')
    pp_use_frenet_path = LaunchConfiguration('pp_use_frenet_path')
    pp_frenet_topic    = LaunchConfiguration('pp_frenet_path_topic')

    # === Frenet Local Planner Node ===
    frenet_node = Node(
        package='racecar_frenet_cpp',
        executable='frenet_local_planner',
        name='frenet_local_planner',
        output='screen',
        parameters=[
            os.path.join(package_share_directory, 'config', 'params.yaml'),
            {
                'max_speed': max_speed,
                'max_accel': max_accel,
                'max_curvature': max_curvature,
                # 실제 코드에서 쓰는 이름이 있으면 s_horizon로 같이 넘겨줌
                's_horizon': s_horizon,
            }
        ],
    )

    # === Pure Pursuit Node ===
    pure_pursuit_node = Node(
        package='f1tenth_planner',   # 네 패키지 이름에 맞게 수정
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[
            {
                'csv_path': pp_csv_path,
                'lookahead_min': pp_lookahead_min,
                'lookahead_gain': pp_lookahead_gain,
                'wheelbase': 0.33,      # 자주 안 바꿀 거면 launch에서 고정해도 됨
                'max_speed': pp_max_speed,
                'use_frenet_path': pp_use_frenet_path,
                'frenet_path_topic': pp_frenet_topic,
            }
        ],
    )

    return LaunchDescription([
        # Declare all args
        max_speed_arg,
        max_accel_arg,
        max_curvature_arg,
        s_horizon_arg,
        pp_max_speed_arg,
        pp_lookahead_min_arg,
        pp_lookahead_gain_arg,
        pp_csv_path_arg,
        pp_use_frenet_path_arg,
        pp_frenet_topic_arg,

        # Nodes
        frenet_node,
        pure_pursuit_node,
    ])

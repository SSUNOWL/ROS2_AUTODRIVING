import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('racecar_frenet_cpp')

    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='BALANCED',
        description='Frenet profile: BALANCED | RACE | EXTREME'
    )

    # 필요하면 나중에 override용 인자도 추가 가능
    # (예: max_speed_override 같은 것)

    return LaunchDescription([
        profile_arg,
        OpaqueFunction(function=launch_setup, args=[pkg_share]),
    ])


def launch_setup(context, pkg_share):
    profile = LaunchConfiguration('profile').perform(context).upper()

    # --- 1) 프로파일별 기본 파라미터 세트 정의 ---

    if profile == 'BALANCED':
        frenet_params = {
            'max_speed': 4.1,
            'target_speed': 4.0,
            'max_accel': 3.0,
            'max_curvature': 0.9,
        }
        pp_max_speed = 4.1

    elif profile == 'RACE':
        frenet_params = {
            'max_speed': 6.2,
            'target_speed': 5.1,
            'max_accel': 5.0,
            'max_curvature': 1.0,
        }
        pp_max_speed = 6.2

    elif profile == 'EXTREME':
        frenet_params = {
            'max_speed': 6.5,
            'target_speed': 5.3,
            'max_accel': 5.5,
            'max_curvature': 1.0,
        }
        pp_max_speed = 6.5

    else:
        # 잘못된 값 들어오면 RACE로 fallback
        frenet_params = {
            'max_speed': 6.2,
            'target_speed': 5.1,
            'max_accel': 5.0,
            'max_curvature': 1.0,
        }
        pp_max_speed = 6.2

    # --- 2) Frenet Local Planner 노드 ---

    frenet_node = Node(
        package='racecar_frenet_cpp',
        executable='frenet_local_planner',
        name='frenet_local_planner',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'params.yaml'),
            frenet_params,
        ],
    )

    # --- 3) Pure Pursuit 노드 (Frenet 경로 추종) ---

    pp_params = {
        'max_speed': pp_max_speed,
        'use_frenet_path': True,
        'frenet_path_topic': '/frenet_local_plan',
        # 필요하면 여기다 lookahead_gain, min_lookahead 등 추가 가능
    }

    pp_node = Node(
        package='f1tenth_planner',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[pp_params],
    )

    return [frenet_node, pp_node]

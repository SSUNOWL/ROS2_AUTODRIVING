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

    return LaunchDescription([
        profile_arg,
        OpaqueFunction(function=launch_setup, args=[pkg_share]),
    ])


def launch_setup(context, pkg_share):
    profile = LaunchConfiguration('profile').perform(context).upper()

    # --- 1) 프로파일별 기본 파라미터 세트 정의 ---
    # 정리 기준:
    # - BALANCED : 두 맵에서 안정 완주 우선 (hairpin 기준으로 보수적)
    # - RACE     : raceline + hairpin 공통 RACE 프로필 (X1He)
    # - EXTREME  : raceline 전용 최속(X1), hairpin에서는 과격

    if profile == 'BALANCED':
        # hairpin에서 처음으로 안정 완주했던 계열(X1Hc 근처)을 베이스로,
        # 너무 느리지 않게만 살짝 조정한 "세이프 모드"
        frenet_params = {
            'max_speed': 5.0,
            'target_speed': 4.0,
            'max_accel': 4.0,
            'max_curvature': 0.9,
        }
        pp_max_speed = 5.0

    elif profile == 'RACE':
        # ✅ 공통 RACE 프로필 (X1He)
        # hairpin: 61.06 s / Max v ≈ 4.7 / a_lat ≈ 12.8 / NO collision
        # raceline: 83.35 s / Max v ≈ 4.6 / a_lat ≈ 8.6 / NO collision
        frenet_params = {
            'max_speed': 5.4,
            'target_speed': 4.7,
            'max_accel': 5.0,
            'max_curvature': 0.9,
        }
        pp_max_speed = 5.4

    elif profile == 'EXTREME':
        # ⚠ raceline 최속(X1) 프로필
        # raceline: 76.69 s / Max v ≈ 5.0 / a_lat ≈ 9.27 (GOAL)
        # hairpin에서는 과격하므로 사용 비권장
        frenet_params = {
            'max_speed': 6.2,
            'target_speed': 5.1,
            'max_accel': 5.0,
            'max_curvature': 1.0,
        }
        pp_max_speed = 6.2

    else:
        # 잘못된 값 들어오면 공통 RACE로 fallback
        frenet_params = {
            'max_speed': 5.4,
            'target_speed': 4.7,
            'max_accel': 5.0,
            'max_curvature': 0.9,
        }
        pp_max_speed = 5.4

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

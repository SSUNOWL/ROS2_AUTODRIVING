import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_planner')

    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='SAFE',
        description='FGM profile: SAFE | BALANCED'
    )

    def launch_for_profile(context):
        profile = LaunchConfiguration('profile').perform(context).upper()

        # ===== 프로파일별 파라미터 세트 =====
        if profile == 'BALANCED':
            params = {
                # --- FGM ---
                'fgm_gap_threshold':      '1.2',
                'fgm_bubble_radius':      '0.55',
                'fgm_fov_angle':          '180.0',

                'fgm_min_planning_dist':  '2.0',
                'fgm_max_planning_dist':  '5.0',
                'fgm_planning_gain':      '1.0',

                'fgm_min_lookahead':      '1.6',
                'fgm_max_lookahead':      '3.6',
                'fgm_lookahead_gain':     '0.6',

                'fgm_max_speed':          '4.0',
                'fgm_min_speed':          '2.0',
                'fgm_slow_down_dist':     '2.8',

                # --- Pure Pursuit ---
                'pp_csv_path':            'raceline_with_speed.csv',
                'pp_lookahead_min':       '1.2',
                'pp_lookahead_gain':      '0.35',
                'pp_wheelbase':           '0.33',
                'pp_max_speed':           '4.0',
                'pp_use_frenet_path':     'true',
                'pp_frenet_path_topic':   '/fgm_path',
            }
        else:  # SAFE
            params = {
                # --- FGM: 더 안전하게 ---
                'fgm_gap_threshold':      '1.3',
                'fgm_bubble_radius':      '0.60',
                'fgm_fov_angle':          '180.0',

                'fgm_min_planning_dist':  '2.0',
                'fgm_max_planning_dist':  '5.0',
                'fgm_planning_gain':      '1.0',

                'fgm_min_lookahead':      '1.6',
                'fgm_max_lookahead':      '3.6',
                'fgm_lookahead_gain':     '0.6',

                'fgm_max_speed':          '3.5',
                'fgm_min_speed':          '2.0',
                'fgm_slow_down_dist':     '3.0',

                # --- Pure Pursuit: 속도도 약간 더 낮게 ---
                'pp_csv_path':            'raceline_with_speed.csv',
                'pp_lookahead_min':       '1.2',
                'pp_lookahead_gain':      '0.35',
                'pp_wheelbase':           '0.33',
                'pp_max_speed':           '3.5',
                'pp_use_frenet_path':     'true',
                'pp_frenet_path_topic':   '/fgm_path',
            }

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'fgm_with_pp.launch.py')
                ),
                launch_arguments=params.items()
            )
        ]

    return LaunchDescription([
        profile_arg,
        OpaqueFunction(function=launch_for_profile),
    ])


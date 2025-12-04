from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context).lower()
    track = LaunchConfiguration("track").perform(context).lower()
    user_csv = LaunchConfiguration("pp_csv_path").perform(context)

    # profile 검사
    if profile not in ["optimal"]:
        print("[FGM Profile] WARNING: Only 'optimal' is supported. Defaulting to optimal.")
        profile = "optimal"

    if track not in ["spielberg", "360"]:
        track = "spielberg"

    # ----------------------------------------------------------
    # 최소 공통 파라미터 (PP 관련 + CSV 선택만 남김)
    # ----------------------------------------------------------
    common_params = {
        "pp_lookahead_min": 1.2,
        "pp_lookahead_gain": 0.36,
        "pp_wheelbase": 0.33,
        "pp_use_frenet_path": True,
        "pp_frenet_path_topic": "/fgm_path",
    }

    # ----------------------------------------------------------
    # CSV 경로 선택
    # ----------------------------------------------------------
    if track == "360":
        default_csv = "180wind.csv"
    else:
        default_csv = "raceline_with_yaw.csv"

    if user_csv and user_csv != "__auto__":
        csv_to_use = user_csv
    else:
        csv_to_use = default_csv

    common_params["pp_csv_path"] = csv_to_use

    # ----------------------------------------------------------
    # ★ optimal: 모든 FGM 파라미터를 여기서 100% 명시
    # ----------------------------------------------------------
    if profile == "optimal":
        optimal = {
            "fgm_fov_angle": 190.5232,
            "fgm_speed_check_fov_deg": 21.8559,

            "fgm_gap_threshold": 0.9946,
            "fgm_bubble_radius": 0.3625,
            "fgm_required_clearance": 0.4313,

            "fgm_width_weight": 0.7846,
            "fgm_angle_weight": 8.4080,
            "fgm_steer_weight": 0.0472,
            "fgm_hysteresis_bonus": 1.3784,
            "fgm_change_threshold": 0.2005,
            "fgm_smoothing_alpha": 0.5416,
            "fgm_dynamic_bubble_speed_coeff": 0.1703,

            "fgm_max_speed": 5.6373,
            "pp_max_speed": 5.6373,
        }
        common_params.update(optimal)

    # ----------------------------------------------------------
    # fgm_with_pp.launch.py include
    # ----------------------------------------------------------
    planner_pkg = FindPackageShare("f1tenth_planner")
    base_launch_path = PathJoinSubstitution(
        [planner_pkg, "launch", "fgm_with_pp.launch.py"]
    )

    launch_args = {
        k: (str(v).lower() if isinstance(v, bool) else str(v))
        for k, v in common_params.items()
    }

    include_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments=launch_args.items(),
    )

    return [include_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="optimal",
            description="Only 'optimal' supported for FGM tuning.",
        ),
        DeclareLaunchArgument(
            "track",
            default_value="spielberg",
            description="Track: spielberg | 360",
        ),
        DeclareLaunchArgument(
            "pp_csv_path",
            default_value="__auto__",
            description="CSV path for PP global reference.",
        ),
        OpaqueFunction(function=launch_setup),
    ])

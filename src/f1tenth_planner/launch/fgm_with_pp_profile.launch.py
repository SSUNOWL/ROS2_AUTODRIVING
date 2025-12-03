from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # --- 런치 인자 읽기 ---
    profile = LaunchConfiguration("profile").perform(context).lower()   # safe / balanced / race
    track = LaunchConfiguration("track").perform(context).lower()       # spielberg / 360
    user_csv = LaunchConfiguration("pp_csv_path").perform(context)      # "__auto__" 또는 사용자 지정 경로

    # 방어 코드
    if profile not in ["safe", "balanced", "race"]:
        profile = "balanced"

    if track not in ["spielberg", "360"]:
        track = "spielberg"

    # --- 공통 FGM / PP 파라미터 (두 맵 공통 베이스) ---
    common_params = {
        # FGM 공통 설정
        "fgm_gap_threshold": 1.15,
        "fgm_bubble_radius": 0.46,
        "fgm_fov_angle": 180.0,
        "fgm_min_planning_dist": 2.0,
        "fgm_max_planning_dist": 4.5,
        "fgm_planning_gain": 1.0,

        "fgm_min_lookahead": 1.8,
        "fgm_max_lookahead": 3.8,   # balanced에서 3.9로 덮어씀
        "fgm_lookahead_gain": 0.55, # balanced에서 0.56으로 덮어씀

        "fgm_min_speed": 2.0,
        "fgm_slow_down_dist": 3.0,

        # Pure Pursuit 공통 설정
        "pp_lookahead_min": 1.2,
        "pp_lookahead_gain": 0.36,
        "pp_wheelbase": 0.33,
        "pp_use_frenet_path": True,
        "pp_frenet_path_topic": "/fgm_path",
    }

    # --- 프로파일별 룩어헤드 보정 (balanced만 살짝 공격적으로) ---
    if profile == "balanced":
        common_params["fgm_max_lookahead"] = 3.9
        common_params["fgm_lookahead_gain"] = 0.56

    # --- 맵/프로파일별 속도 테이블 ---
    speed_table = {
        "spielberg": {
            "safe": 4.8,
            "balanced": 5.2,
            "race": 5.3,
        },
        "360": {
            "safe": 4.1,    # Run3 기반
            "balanced": 4.35,  # Run10 기반
            "race": 4.6,    # Run5 기반
        },
    }

    max_speed = speed_table[track][profile]

    # 속도 계열 파라미터 (FGM + PP 공통)
    common_params["fgm_max_speed"] = max_speed
    common_params["pp_max_speed"] = max_speed

    # --- CSV 경로 선택 로직 ---
    # "__auto__"면 트랙에 따라 기본값 사용, 아니면 사용자가 준 값 그대로 사용
    if track == "360":
        default_csv = "180wind.csv"
    else:
        default_csv = "raceline_with_yaw.csv"

    if user_csv and user_csv != "__auto__":
        csv_to_use = user_csv
    else:
        csv_to_use = default_csv

    common_params["pp_csv_path"] = csv_to_use

    # --- 실제 fgm_with_pp.launch.py 인클루드 ---
    planner_pkg = FindPackageShare("f1tenth_planner")
    base_launch_path = PathJoinSubstitution(
        [planner_pkg, "launch", "fgm_with_pp.launch.py"]
    )

    # launch_arguments에 넣기 위해 문자열로 캐스팅
    launch_args = {
        k: (str(v).lower() if isinstance(v, bool) else str(v))
        for k, v in common_params.items()
    }

    include_fgm_with_pp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments=launch_args.items(),
    )

    return [include_fgm_with_pp]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="balanced",
            description="FGM profile: safe | balanced | race",
        ),
        DeclareLaunchArgument(
            "track",
            default_value="spielberg",
            description="Track: spielberg | 360",
        ),
        DeclareLaunchArgument(
            "pp_csv_path",
            default_value="__auto__",
            description="Path to global CSV. If '__auto__', use raceline_with_yaw.csv (spielberg) or 180wind.csv (360).",
        ),
        OpaqueFunction(function=launch_setup),
    ])

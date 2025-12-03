from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    scenario = DeclareLaunchArgument(
        "scenario_name", default_value="S1"
    )
    mode = DeclareLaunchArgument(
        "planner_mode", default_value="MUX"
    )

    w_speed = DeclareLaunchArgument("w_speed", default_value="3.0")
    w_track = DeclareLaunchArgument("w_track", default_value="2.0")
    w_comfort = DeclareLaunchArgument("w_comfort", default_value="1.5")
    w_safety = DeclareLaunchArgument("w_safety", default_value="3.0")

    mux = Node(
        package="planner_mux",
        executable="planner_mux_node",
        name="planner_mux",
        parameters=[{
            "w_speed": LaunchConfiguration("w_speed"),
            "w_track": LaunchConfiguration("w_track"),
            "w_comfort": LaunchConfiguration("w_comfort"),
            "w_safety": LaunchConfiguration("w_safety"),
        }]
    )

    logger = Node(
        package="racecar_experiments",
        executable="run_logger",
        name="run_logger",
        parameters=[{
            "scenario_name": LaunchConfiguration("scenario_name"),
            "planner_mode": LaunchConfiguration("planner_mode"),
            "output_dir": "experiment_logs"
        }]
    )

    return LaunchDescription([
        scenario, mode,
        w_speed, w_track, w_comfort, w_safety,
        mux,
        logger
    ])

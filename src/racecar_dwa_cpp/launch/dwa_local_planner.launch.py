from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration(
        'params_file',
        default='config/dwa_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the parameter file'
        ),
        Node(
            package='racecar_dwa_cpp',
            executable='dwa_local_planner',
            name='dwa_local_planner',
            output='screen',
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
        )
    ])


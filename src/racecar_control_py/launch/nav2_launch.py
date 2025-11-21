import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 설정 파일 경로 (주의: 빌드 후 install 경로를 참조하므로 재빌드 필요)
    # 여기서는 편의상 소스 경로를 직접 지정하거나, 패키지 install 경로를 사용합니다.
    # 일단은 절대 경로로 지정해서 테스트하는 것이 가장 확실합니다.
    # 아래 'jsun'을 본인 사용자명으로 꼭 확인하세요!
    params_file = '/home/jsun/f1_sim_ws/src/racecar_control_py/config/nav2_params.yaml'
    
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        # Nav2의 핵심 네비게이션 노드들만 실행 (Map Server, AMCL 제외)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': params_file,
                'autostart': 'True',
            }.items(),
        ),
    ])
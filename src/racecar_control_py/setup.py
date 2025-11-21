from setuptools import setup
import os
from glob import glob

package_name = 'racecar_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # [중요] launch 폴더 안의 모든 .py 파일을 설치 경로로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # [중요] config 폴더 안의 모든 .yaml 파일을 설치 경로로 복사
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsun',
    maintainer_email='jsun@todo.todo',
    description='F1Tenth Control Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 아까 만든 키보드 조종 노드 등록
        ],
    },
)
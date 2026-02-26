import os
from glob import glob
from setuptools import setup

package_name = 'environment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')), 
        (os.path.join('share', package_name, 'usd'), glob('usd/*')), # usd 폴더 내의 모든 파일(usd, txt 등) 포함
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='jowenchoi@gmail.com',   
    description='Environment bringup for UR10 project using Nav2 components',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run environment robot_sim_main 명령어로 실행할 수 있도록 등록
            'robot_sim_main = environment.robot_sim_main:main', 
        ],
    },
)
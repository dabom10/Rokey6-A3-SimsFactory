#!/usr/bin/env python3
"""
로봇 이동 노드 실행 launch 파일

사용법:
  ros2 launch robot_move_pkg move_robot.launch.py node_type:=basic
  ros2 launch robot_move_pkg move_robot.launch.py node_type:=pure_pursuit
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch 인자 정의
    node_type_arg = DeclareLaunchArgument(
        'node_type',
        default_value='pure_pursuit',
        description='노드 타입: basic 또는 pure_pursuit'
    )
    
    # 기본 이동 노드
    basic_move_node = Node(
        package='robot_move_pkg',
        executable='move_robot_node',
        name='robot_move_node',
        output='screen',
        condition=LaunchConfiguration('node_type') == 'basic'
    )
    
    # Pure Pursuit 이동 노드
    pure_pursuit_node = Node(
        package='robot_move_pkg',
        executable='pure_pursuit_node',
        name='pure_pursuit_move_node',
        output='screen',
        condition=LaunchConfiguration('node_type') == 'pure_pursuit'
    )
    
    return LaunchDescription([
        node_type_arg,
        basic_move_node,
        pure_pursuit_node,
    ])

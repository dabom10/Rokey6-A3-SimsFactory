# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessIO

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    pkg_dir = get_package_share_directory("environment")

    map_dir = LaunchConfiguration("map", default=os.path.join(pkg_dir, "maps", "jjang_map.yaml"))
    param_dir = LaunchConfiguration("params_file", default=os.path.join(pkg_dir, "params", "jjang_params.yaml"))
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    rviz_config_dir = os.path.join(pkg_dir, "rviz", "jjang_navigation.rviz")

    ld_automatic_goal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, "launch", "jjang_navigation_goal.launch.py"))
    )

    def execute_second_node_if_condition_met(event, second_node_action):
        output = event.text.decode().strip()
        if "[INFO] Starting simulation loop" in output:
            print("👉 [조건 충족] Isaac Sim 로드 완료! 자동 목표 설정 노드를 실행합니다.")
            return second_node_action

    # Isaac Sim 노드 활성화
    isaac_sim_node = Node(
        package='environment',
        executable='robot_sim_main',
        name='robot_sim_main',
        output='screen'
    )

    # [핵심] 라이다(base_scan)와 로봇 베이스(base_footprint)를 연결하는 정적 TF 퍼블리셔
    # Isaac Sim에서 발행하는 라이다 프레임과 Nav2가 기대하는 프레임을 연결합니다
    static_tf_base_to_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_base_to_scan',
        # arguments: [x, y, z, yaw, pitch, roll, parent_frame, child_frame]
        # 라이다가 로봇 중심(base_footprint)에서 위쪽(z=0.2m)에 위치한다고 가정
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_footprint', 'base_scan']
    )

    # [추가] base_link와 base_footprint 연결 (일반적으로 필요)
    static_tf_base_link_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_base_link_to_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=map_dir, description="불러올 맵 파일의 전체 경로"),
            DeclareLaunchArgument("params_file", default_value=param_dir, description="파라미터 파일의 전체 경로"),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="시뮬레이션 시간 동기화 사용 여부"),

            # Isaac Sim 노드 실행
            isaac_sim_node,
            
            # TF 브로드캐스터 노드들 실행
            static_tf_base_to_scan,
            static_tf_base_link_to_footprint,

            # RViz 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
            ),

            # Nav2 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "bringup_launch.py")),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
            ),

            # Isaac Sim이 준비되면 자동 목표 설정 노드 실행
            RegisterEventHandler(
                OnProcessIO(
                    target_action=isaac_sim_node, 
                    on_stdout=lambda event: execute_second_node_if_condition_met(event, ld_automatic_goal)
                )
            ),
        ]
    )
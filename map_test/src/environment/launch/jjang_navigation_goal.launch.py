# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory("environment")

    map_yaml_file = LaunchConfiguration(
        "map_yaml_path",
        default=os.path.join(pkg_dir, "maps", "jjang_map.yaml"),
    )

    goal_text_file = LaunchConfiguration(
        "goal_text_file_path",
        default=os.path.join(pkg_dir, "usd", "jjang_goals.txt"),
    )

    navigation_goal_node = Node(
        name="set_navigation_goal",
        package="isaac_ros_navigation_goal", # 이 노드 자체는 NVIDIA 패키지를 사용하는 것이 맞습니다.
        executable="SetNavigationGoal",
        parameters=[
            {
                "map_yaml_path": map_yaml_file,
                "iteration_count": 3,
                
                # 💡 참고: RandomGoalGenerator는 맵 내 무작위 목표를 생성합니다. 
                # 만약 jjang_goals.txt 안의 좌표를 순서대로 가려면 이 값을 "GoalReader" 등으로 바꿔야 할 수 있습니다.
                "goal_generator_type": "RandomGoalGenerator", 
                
                "action_server_name": "navigate_to_pose",
                "obstacle_search_distance_in_meters": 0.2,
                "goal_text_file_path": goal_text_file,
                
                # 형태: [x, y, z, qx, qy, qz, qw]
                "initial_pose": [-10.13094, 6.21142, 0.1, 0.0, 0.0, 0.0, 1.0],
            }
        ],
        output="screen",
    )

    return LaunchDescription([navigation_goal_node])
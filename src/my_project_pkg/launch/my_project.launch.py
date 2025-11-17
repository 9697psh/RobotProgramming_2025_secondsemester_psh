#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 'waffle' 모델을 사용하도록 환경 변수 설정
    set_turtlebot3_model_env = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL', 'waffle'
    )

    # 패키지 경로
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # 런치 인자
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='4.0')
    y_pose = LaunchConfiguration('y_pose', default='2.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    world = os.path.join(
        get_package_share_directory('my_project_pkg'),
        'worlds',
        'project.world'
    )

    # Gazebo 실행
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'use_sim_time': use_sim_time}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher 실행 (오류 수정된 버전)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'model': 'waffle'}.items()
    )

    # 로봇 생성 (오류 수정된 버전)
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-file', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_waffle', 'model.sdf'),
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )

    # LaunchDescription 조립
    ld = LaunchDescription()
    
    ld.add_action(set_turtlebot3_model_env)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld

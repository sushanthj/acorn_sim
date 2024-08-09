# Copyright (c) 2018 Intel Corporation
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

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    # Get the launch directory
    gps_wpf_dir = get_package_share_directory("nav2_gps_waypoint_follower_demo")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    world = os.path.join(gps_wpf_dir, "worlds", "sonoma_raceway.world")

    xacro_file = os.path.join(gps_wpf_dir, 'urdf', 'robot_sim.urdf.xacro')

    # Validate the Xacro file using the --check option
    validate_xacro = ExecuteProcess(
        cmd=['ros2', 'run', 'xacro', 'xacro', '--check', xacro_file],
        output='screen'
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    models_dir = os.path.join(gps_wpf_dir, "models")

    set_gazebo_model_path_cmd = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", models_dir)

    # Alternative to launch gazebo server and client
    """
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

    # Launch Gazebo, load the world
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')]),
        launch_arguments={'world': world_file}.items(),
    )

    # Launch the Gazebo client (optional, for GUI)
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')]),
    )
    """

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
        }.items()
    )

    # Spawning the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', "acorn", '-topic', '/robot_description'],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # Starting the teleop node
    teleop = Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix='xterm -e',
        name='teleop'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # validate xacro
    ld.add_action(validate_xacro)

    # Launch Gazebo
    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(gazebo)

    # robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_entity)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(teleop)

    return ld

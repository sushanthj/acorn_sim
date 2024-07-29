from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    declare_my_robot_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='four_wheeled_robot',
        description='Robot Types: "four_wheeled_robot",'
    )

    declare_world_name_arg = DeclareLaunchArgument(
        'world',
        default_value='svd_demo',
        description='Available worlds: "svd_demo", "workspace_0"'
    )

    world_path = os.path.join(
            get_package_share_directory('simulation_launch'),
            'worlds',
            LaunchConfiguration('world') + '.world')
    robot_urdf_path = os.path.join(
            get_package_share_directory('simulation_launch'),
            'robots/'+ LaunchConfiguration('robot_name') + '/',
            LaunchConfiguration('robot_name') +'.urdf')

    # Launch gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', LaunchConfiguration('robot_name'),'-file', robot_urdf_path], 
        output='screen'
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[robot_urdf_path]
    )

    # Start Teleop Twist Keyboard
    teleop = Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        name='teleop'
    )

    ld.add_action(declare_my_robot_arg)
    ld.add_action(declare_world_name_arg)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(teleop)

    return ld

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    launch_files_pkg_share_directory = get_package_share_directory('simulation_launch')

    width = LaunchConfiguration('width', default='8')
    height = LaunchConfiguration('height', default='5')
    obstacle_density = LaunchConfiguration('obstacle_density', default='0.1')
    start = LaunchConfiguration('start', default='[0, 0]')
    goal = LaunchConfiguration('goal', default='[4, 4]')
    random_map_seed = LaunchConfiguration('random_map_seed', default=False)
    motion_model_stochasticity = LaunchConfiguration('motion_model_stochasticity', default='0.25')
    default_rviz_config_path = os.path.join(launch_files_pkg_share_directory,
                                            'rviz', 'rviz_config.rviz')
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=default_rviz_config_path
    )
    print("rviz_config_file: ", default_rviz_config_path)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    grid_environment_node = Node(
        package='grid_environment',
        executable='grid_environment',
        name='grid_environment',
        output='screen',
        parameters=[
            {'width': width},
            {'height': height},
            {'obstacle_density': obstacle_density},
            {'start': start},
            {'goal': goal},
            {'random_map_seed': random_map_seed},
            {'motion_model_stochasticity': motion_model_stochasticity}
        ]
    )

    dijkstra_service_node = Node(
        package='dijkstra_service',
        executable='dijkstra_service',
        name='dijkstra_service',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(rviz_node)

    delayed_launch = TimerAction(
        period=5.0,
        actions=[
            grid_environment_node,
            dijkstra_service_node
        ]
    )

    ld.add_action(delayed_launch)

    return ld

import os
import launch
from ament_index_python.packages import get_package_share_directory

import launch_ros.actions

def generate_launch_description():
    package_dir = get_package_share_directory('simulation_launch')
    rviz_config_file = os.path.join(package_dir,'rviz', 'rviz_config.rviz')
    urdf_file = os.path.join(package_dir, 'robots', 'four_wheeled_robot', 'four_wheeled_robot.urdf')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
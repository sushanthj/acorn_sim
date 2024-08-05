---
layout: page
title: Jamboard
permalink: /Jamboard/
nav_order: 3
---

# Notes

1. Build Simulation with simple 4 wheel robot
2. Add Twisted Fields controller to move the robot
3. Improve Robot Visuals (use CAD)
4. Improve Robot Surroundings (Google Maps overlay, real plants)

## Example Models and Worlds

1. [Simple 4 Wheel Robot](https://app.gazebosim.org/OpenRobotics/fuel/models/X2%20Config%207)
2. [Fancy Outdoor World](https://app.gazebosim.org/Penkatron/fuel/worlds/Rubicon%20World)
3. [MP400 URDF Setup](https://github.com/neobotix/neo_simulation2/blob/humble/robots/mp_400/mp_400.urdf)

## Convert XACRO to URDF

```bash
ros2 run xacro xacro -o robot.urdf robot.urdf.xacro
```

- The above script will convert the xacro file to urdf file
- This can also be done in the launch file. Here's two different launch files:

### Launch File for URDF

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_launch')
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')],
        ),
        # Create a joint_state_publisher_gui node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])
```

### Launch File for XACRO

```python
import os
import launch
import xacro
import launch_ros.actions

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_dir = get_package_share_directory('simulation_launch')
    rviz_config_file = os.path.join(package_dir,'rviz', 'rviz_config.rviz')

    # if using xacro it needs to be processed first
    xacro_file = os.path.join(package_dir, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        # Create a joint_state_publisher_gui node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])
```

**NOTE: Continuous Joints are not shown until a joint state is published.**
This can be done using the joint_state_publisher_gui node.

## Process to Convert CAD to SDF and DAE

1. Design CAD and export entire assembly of chassis as one object (one SLDPRT not SLDASSY)
2. Convert the single SLDPRT to STL
3. Use Blender to convert STL to DAE (after adding any necessary colors)

Q. Is there a better way to do the DAE conversion without blender?
Ans. Yes, use FreeCAD to convert STL to DAE but that seems to be more painful

Here's a link which I found useful: [Convert CAD -> STL -> DAE](https://www.youtube.com/watch?v=zGWFojrPoSA)

**TODO:** Make a video of this for future reference

## Integrate DAE and SDF into Robot URDF and World SDF


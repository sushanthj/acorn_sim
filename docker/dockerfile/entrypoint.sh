#!/bin/bash
set -e

# setup ros environment
# sudo apt update
# sudo apt install nano
source "/opt/ros/$ROS_DISTRO/setup.bash"
# colcon build --packages-select grid_environment

exec "$@"
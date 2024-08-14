# dockerfile/Dockerfile

FROM osrf/ros:humble-simulation-jammy

RUN useradd -ms /bin/bash admin

WORKDIR /home/admin

# Switch back to root user to install dependencies
USER root

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-rviz2 \
    xterm \
    tmux \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-tools \
    build-essential \
    nano \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy the package-setup directory into the image
COPY package-setup ./package-setup

# Make the setup script executable
RUN chmod +x ./package-setup/setup_simulation.sh

RUN cd ./package-setup && ./setup_simulation.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/admin/.bashrc

# Make the admin user own their home directory
RUN chown -R admin:admin /home/admin

# Set the entrypoint to bash
ENTRYPOINT ["bash"]

# Simulation Environment for the Acorn Robot

## Setup and Installation

1. Clone Repository:

```bash
git clone --recurse-submodules https://github.com/sushanthj/acorn_sim.git
cd acorn_sim
git submodule update --init --recursive
```

1. Install Docker: [Link](https://docs.docker.com/engine/install/ubuntu/)

2. Build Docker Image and Start Container:

```bash
cd acorn_sim/docker
docker-compose build
docker-compose up -d
```

3. Enter Docker Container:

```bash
./run_sim_docker.sh
```

4. Build Simulation:

```bash
cd sim_ws
colcon build
source install/setup.bash
```

> Note: display driver needs to allow docker to run GUI applications. For Ubuntu, I use the follow command to add docker to the xhost list.

```bash
xhost +local:docker
```

6. Run Simulation:

```bash
ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py

ros2 run nav2_gps_waypoint_follower_demo latlon_waypoint_follower
Ctrl+C # Stop Simulation
```

7. Close Simulation:

```bash
Ctrl+D # Exit Docker Container

docker compose down
```

## Distributed Systems Architecture

![](/docs/images/systems_arch/Acorn_Systems_1.png)

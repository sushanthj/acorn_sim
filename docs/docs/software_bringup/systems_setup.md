---
layout: default
title: Docker
parent: Systems Setup
nav_order: 2
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

# Systems Overview

## Setup and Installation

1. Clone Repository:

```bash
git clone https://github.com/sushanthj/acorn_sim.git
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
colcon build
source install/setup.bash
```

> Note: display driver needs to allow docker to run GUI applications. For Ubuntu, I use the follow command to add docker to the xhost list.

```bash
xhost +local:docker
```

6. Run Simulation:

```bash
ros2 launch simulation_launch simulation_launch.launch.py

Ctrl+C # Stop Simulation
```

7. Close Simulation:

```bash
Ctrl+D # Exit Docker Container

docker compose down
```

## Distributed Systems Architecture

![](/docs/images/systems_arch/Acorn_Systems_1.png)
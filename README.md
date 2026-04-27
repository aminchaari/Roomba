# Roomba ROS2 Simulation

This repository contains a ROS2 Jazzy simulation workspace for a Roomba-style coverage planner.
It includes:

- `coverage_planner` package with room coverage planning
- `ros2-autonomous-explorer` for frontier exploration
- `fields2cover` integration support
- Docker setup for plug-and-play simulation

## 1. Run with Docker

Anyone with Docker can run the simulation without installing ROS2, Nav2, or Gazebo locally.

### Build the Docker image

```bash
cd /root/roomba_ws
./docker_start.sh build
```

or equivalently:

```bash
docker-compose build
```

### Start the container

```bash
./docker_start.sh up
```

or:

```bash
docker-compose up -d
```

### Enter the container shell

```bash
./docker_start.sh shell
```

or:

```bash
docker-compose exec roomba bash
```

## 2. Start the Frontier Exploration Simulation

Inside the running container shell:

```bash
source install/setup.bash
ros2 launch coverage_planner full_pipeline.launch.py
```

This launches Gazebo, SLAM, Nav2, and the frontier exploration stack.

## 3. Start Coverage Sweep

After exploration completes and the map is ready, run:

```bash
./docker_start.sh trigger
```

or inside the container:

```bash
source install/setup.bash
ros2 topic pub -1 /start_coverage std_msgs/Bool "data: true"
```

## 4. Useful Docker Commands

```bash
./docker_start.sh logs      # follow container logs
./docker_start.sh down      # stop the container
./docker_start.sh status    # inspect container/image status
./docker_start.sh clean     # remove image and container data
```

## 5. If You Want to Run Without Docker

If you already have ROS2 Jazzy installed, use the same simulation commands inside the workspace:

```bash
cd /path/to/roomba_ws
source install/setup.bash
ros2 launch coverage_planner full_pipeline.launch.py
```

Then trigger coverage:

```bash
ros2 topic pub -1 /start_coverage std_msgs/Bool "data: true"
```

## 6. Repository Contents

- `Dockerfile` — builds the ROS2 environment and workspace
- `docker-compose.yml` — container orchestration + X11 GUI support
- `docker_start.sh` — wrapper for build/up/shell/launch/trigger commands
- `.dockerignore` — excludes local build artifacts
- `src/` — ROS2 workspace source packages
- `maps/` — saved map files used by the workflow

## 7. Add this to GitHub

This file should be committed to your GitHub repository as `README.md` so new contributors can launch the simulation immediately.

---

Happy simulating! 🚀

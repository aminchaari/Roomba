# ROS2 Roomba Coverage Planner - Docker Setup

Complete plug-and-play Docker environment for the ROS2 Roomba autonomous coverage planner with Gazebo simulation.

## Prerequisites

- **Docker** (v20.10+): [Install Docker](https://docs.docker.com/get-docker/)
- **Docker Compose** (v1.29+): [Install Docker Compose](https://docs.docker.com/compose/install/)
- **X11 server** (Linux/Mac) for GUI visualization
  - **Linux**: X11 is usually pre-installed
  - **Mac**: Install [XQuartz](https://www.xquartz.org/)
  - **Windows**: Use WSL2 with X11 forwarding or VcXsrv

## Quick Start

### 1. Build the Docker Image

```bash
cd /path/to/roomba_ws
docker-compose build
```

**First build may take 10-15 minutes** (includes ROS2 Jazzy, Nav2, SLAM, and all dependencies).

### 2. Start the Container

```bash
docker-compose up -d
```

### 3. Enter the Container

```bash
docker-compose exec roomba bash
```

### 4. Run the Simulation

Inside the container:

```bash
source install/setup.bash
ros2 launch coverage_planner full_pipeline.launch.py
```

The Gazebo simulator window will appear on your display (via X11 forwarding).

### 5. Trigger Coverage Sweep

In another terminal/container window:

```bash
docker-compose exec roomba bash
ros2 topic pub -1 /start_coverage std_msgs/Bool "data: true"
```

Or query coverage algorithm:

```bash
docker-compose exec roomba bash
ros2 param list coverage_planner
```

## Available Commands

### View Container Logs

```bash
docker-compose logs -f roomba
```

### Stop the Container

```bash
docker-compose down
```

### Rebuild and Start Fresh

```bash
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

### Run Tests or Custom Commands

```bash
docker-compose exec roomba colcon test --packages-select coverage_planner
```

## Configuration

### Launch Parameters

The pipeline supports these launch arguments:

```bash
ros2 launch coverage_planner full_pipeline.launch.py \
  headless:=false \
  rviz:=true \
  coverage_algorithm:=grid
```

**Available options:**
- `headless` (true/false): Run Gazebo without GUI
- `rviz` (true/false): Launch RViz2 visualization
- `coverage_algorithm` (grid/fields2cover): Path planning algorithm
- `step_size` (int): Grid row spacing in cells (default: 5)
- `robot_width` (float): Robot width in meters (default: 0.34)

### Environment Variables

Adjust in `docker-compose.yml`:

```yaml
environment:
  - ROS_DOMAIN_ID=0          # ROS2 domain (0-232, isolate multiple sim instances)
  - ROS_LOCALHOST_ONLY=0     # Allow inter-container communication
```

## X11 Display Setup

### Linux

No special setup needed; X11 is usually available:

```bash
docker-compose up
```

### Mac (with XQuartz)

1. Start XQuartz:
   ```bash
   open -a XQuartz
   ```

2. Allow connections from localhost:
   ```bash
   xhost + localhost
   ```

3. Set DISPLAY:
   ```bash
   export DISPLAY=:0
   docker-compose up
   ```

### Windows (WSL2)

Using VcXsrv or WSLg:

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
docker-compose up
```

## Troubleshooting

### "Cannot connect to X11 display"

- Linux: Verify `$DISPLAY` is set: `echo $DISPLAY` (should show something like `:0`)
- Mac: Run `xhost + localhost` before `docker-compose up`
- Windows: Ensure VcXsrv or WSLg is running

### "Permission denied for X11"

```bash
xhost +local:docker
```

### High CPU/Memory Usage

Gazebo with RViz can be resource-intensive. Reduce with:

```bash
ros2 launch coverage_planner full_pipeline.launch.py headless:=true rviz:=false
```

### Build Fails on Slow Internet

Increase timeout in Dockerfile build:

```bash
docker-compose build --progress=plain --no-cache
```

## Development Workflow

### Mount Source for Live Development

Edit `docker-compose.yml`:

```yaml
volumes:
  - ./src:/root/roomba_ws/src  # Add this line
```

Rebuild packages inside container:

```bash
docker-compose exec roomba bash
colcon build --packages-select coverage_planner
```

### Access ROS2 Topics from Host

Use ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=0 to communicate with host ROS2:

```bash
# Host terminal
export ROS_DOMAIN_ID=0
ros2 topic echo /scan
```

## Cleanup

Remove built images and containers:

```bash
docker-compose down -v
docker rmi roomba_coverage_planner:latest
```

## Known Limitations

- **Gazebo Rendering**: Uses software rendering (slow on some systems)
- **Fields2Cover**: Can be slow on large maps (>300×300 cells); use 'grid' for real-time
- **Network**: Container uses `host` mode for easy ROS2 communication

## Architecture Overview

```
Docker Image (Multi-stage):
├── Builder Stage
│   ├── ROS2 Jazzy base
│   ├── Build tools (cmake, colcon, rosdep)
│   ├── Dependencies (Nav2, SLAM, Gazebo, Fields2Cover)
│   └── Build workspace
│
└── Runtime Stage
    ├── ROS2 Jazzy runtime
    ├── Essential packages only
    ├── Pre-built binaries from builder
    └── Maps and configuration
```

**Image size:** ~3.5 GB (includes all ROS2 Jazzy packages)

## Further Documentation

- [ROS2 Jazzy Docs](https://docs.ros.org/en/jazzy/)
- [Nav2 Docs](https://docs.nav2.org/)
- [Gazebo Docs](https://gazebosim.org/)

---

**Happy simulating!** 🤖

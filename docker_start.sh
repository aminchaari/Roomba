#!/bin/bash

# Quick start script for ROS2 Roomba Coverage Planner Docker
# Usage: ./docker_start.sh [build|up|shell|logs|down|clean]

set -e

CONTAINER_NAME="roomba_simulation"
IMAGE_NAME="roomba_coverage_planner:latest"

print_banner() {
    echo "============================================"
    echo "ROS2 Roomba Coverage Planner - Docker"
    echo "============================================"
    echo ""
}

print_help() {
    print_banner
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build       - Build Docker image (run first time)"
    echo "  up          - Start container in background"
    echo "  shell       - Open interactive bash in container"
    echo "  launch      - Launch the full pipeline (inside container)"
    echo "  trigger     - Trigger coverage sweep (inside container)"
    echo "  logs        - View container logs"
    echo "  down        - Stop and remove container"
    echo "  clean       - Remove image and containers"
    echo "  status      - Check container status"
    echo ""
    echo "Examples:"
    echo "  $0 build                 # First time setup"
    echo "  $0 up                    # Start simulation"
    echo "  $0 shell                 # Get bash inside container"
    echo "  $0 launch                # Launch pipeline"
    echo "  $0 down                  # Stop simulation"
    echo ""
}

cmd_build() {
    print_banner
    echo "Building Docker image..."
    docker-compose build
    echo ""
    echo "✓ Build complete! Run '$0 up' to start."
}

cmd_up() {
    print_banner
    echo "Starting container..."
    docker-compose up -d
    echo ""
    sleep 2
    echo "✓ Container started: $CONTAINER_NAME"
    echo ""
    echo "Next steps:"
    echo "  1. Enter shell:  $0 shell"
    echo "  2. Run:          ros2 launch coverage_planner full_pipeline.launch.py"
    echo "  3. Trigger:      $0 trigger"
}

cmd_shell() {
    print_banner
    echo "Entering container shell..."
    docker-compose exec roomba bash
}

cmd_launch() {
    print_banner
    docker-compose exec roomba bash -c "source install/setup.bash && ros2 launch coverage_planner full_pipeline.launch.py"
}

cmd_trigger() {
    print_banner
    echo "Triggering coverage sweep..."
    docker-compose exec roomba bash -c "source install/setup.bash && ros2 topic pub -1 /start_coverage std_msgs/Bool \"data: true\""
}

cmd_logs() {
    docker-compose logs -f roomba
}

cmd_down() {
    print_banner
    echo "Stopping container..."
    docker-compose down
    echo "✓ Container stopped."
}

cmd_clean() {
    print_banner
    echo "WARNING: This will remove the Docker image and containers."
    read -p "Continue? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker-compose down -v
        docker rmi $IMAGE_NAME 2>/dev/null || true
        echo "✓ Cleanup complete."
    fi
}

cmd_status() {
    print_banner
    echo "Container Status:"
    docker-compose ps
    echo ""
    echo "Image Info:"
    docker images | grep roomba || echo "No image found. Run '$0 build' first."
}

# Main
if [ $# -eq 0 ]; then
    print_help
    exit 0
fi

case "$1" in
    build)
        cmd_build
        ;;
    up)
        cmd_up
        ;;
    shell)
        cmd_shell
        ;;
    launch)
        cmd_launch
        ;;
    trigger)
        cmd_trigger
        ;;
    logs)
        cmd_logs
        ;;
    down)
        cmd_down
        ;;
    clean)
        cmd_clean
        ;;
    status)
        cmd_status
        ;;
    *)
        echo "Unknown command: $1"
        print_help
        exit 1
        ;;
esac

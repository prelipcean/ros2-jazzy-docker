# ROS2 Jazzy Development Docker Image

A Docker image for ROS2 Jazzy development based on Ubuntu Noble (24.04).

## Features

- **ROS2 Jazzy** - Full desktop or base installation
- **Development Tools** - colcon, rosdep, vcstool, ros-dev-tools
- **VS Code Server** - Browser-based IDE with C++, CMake, and Hex Editor extensions
- **SSH Server** - Remote access capability
- **Multi-architecture** - Supports both `amd64` and `arm64`
- **GUI Support** - X11 display forwarding for RViz and other GUI tools

## Quick Start

### Using Docker Compose (Recommended)

```bash
# Build and run with GUI support
docker compose up ros2-jazzy

# Build and run with VS Code Server
docker compose up ros2-jazzy-vscode

# Build and run minimal base (no GUI)
docker compose up ros2-jazzy-base

# Run in detached mode
docker compose up -d ros2-jazzy

# Enter running container
docker exec -it ros2_jazzy_dev bash

# Stop containers
docker compose down
```

### Build the Image

```bash
# Build with default settings (desktop install)
docker build -t ros2:jazzy-dev .

# Build with ros-base only (smaller image)
docker build --build-arg ROS_INSTALL_TYPE=ros-base -t ros2:jazzy-base .
```

### Run the Container

```bash
# Interactive shell
docker run --rm -it ros2:jazzy-dev

# With host network and volume mount
docker run --rm -it --name ros2_dev \
    --network host \
    -v ~/ros2_ws:/ros2_ws \
    ros2:jazzy-dev

# With SSH and VS Code Server enabled
docker run --rm -it \
    -p 22:22 \
    -p 8800:8800 \
    -e START_SSH=true \
    -e START_CODE_SERVER=true \
    ros2:jazzy-dev
```

### Access VS Code Server

When running with `START_CODE_SERVER=true`, open your browser and navigate to:
```
http://localhost:8800
```

## Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `UBUNTU_VERSION_TAG` | `noble` | Ubuntu base image tag |
| `PASSWORD` | `temppwd` | Root password for SSH |
| `TARGETARCH` | `amd64` | Target architecture (amd64/arm64) |
| `ROS_DISTRO` | `jazzy` | ROS2 distribution |
| `ROS_INSTALL_TYPE` | `desktop` | Install type (desktop/ros-base) |
| `VS_CODE_SERVER_VERSION` | `4.93.1` | VS Code Server version |
| `VS_CODE_SERVER_PORT` | `8800` | VS Code Server port |

## Environment Variables

| Variable | Description |
|----------|-------------|
| `START_SSH` | Set to `true` to start SSH server on container start |
| `START_CODE_SERVER` | Set to `true` to start VS Code Server on container start |

## Included ROS2 Packages

- `ros-jazzy-desktop` or `ros-jazzy-ros-base`
- `ros-jazzy-ament-cmake`
- `ros-jazzy-ros2-control`
- `ros-jazzy-ros2-controllers`
- `ros-jazzy-xacro`
- `python3-colcon-common-extensions`
- `python3-rosdep`
- `python3-vcstool`

## Directory Structure

```
/ros2_ws/          # ROS2 workspace (WORKDIR)
  └── src/         # Source packages
/workspace/        # Additional workspace
/opt/ros/jazzy/    # ROS2 installation
```

## Testing ROS2

```bash
# Inside the container
source /opt/ros/jazzy/setup.bash

# Run talker demo
ros2 run demo_nodes_cpp talker

# In another terminal, run listener
ros2 run demo_nodes_py listener
```

## GUI Applications (RViz, Gazebo, etc.)

To run GUI applications like RViz from the container, you need to configure display forwarding.

### Check Your Display Settings

First, determine your display server type on the host:

```bash
# Check session type (x11 or wayland)
echo $XDG_SESSION_TYPE

# Check display variable
echo $DISPLAY
```

### X11 Display Forwarding

For X11 sessions (most common):

```bash
# Allow Docker to access X server
xhost +local:docker

# Run container with X11 forwarding
docker run --rm -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    --network host \
    ros2:jazzy-dev

# Test GUI inside container
rviz2
```

### Wayland Display Forwarding

For Wayland sessions:

```bash
# Run container with Wayland support
docker run --rm -it \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    --network host \
    ros2:jazzy-dev
```

### Using Docker Compose with GUI

The `docker-compose.yaml` is pre-configured for X11. Just run:

```bash
# Allow X server access
xhost +local:docker

# Start the container
docker compose up ros2-jazzy

# In another terminal, enter the container
docker exec -it ros2_jazzy_dev bash

# Run RViz
rviz2
```

### Troubleshooting GUI Issues

| Issue | Solution |
|-------|----------|
| `cannot open display` | Run `xhost +local:docker` on host |
| `No protocol specified` | Ensure DISPLAY variable is set correctly |
| Black screen in RViz | Add `-v /dev/dri:/dev/dri` for GPU access |
| Wayland not working | Try XWayland: set `DISPLAY=:0` |

### GPU Acceleration (NVIDIA)

For NVIDIA GPU support, use nvidia-container-toolkit:

```bash
# Install nvidia-container-toolkit first, then:
docker run --rm -it \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    ros2:jazzy-dev
```

## License

MIT License

## Author

Gheorghe Prelipcean <prelipcean.gheorghe@yahoo.com>

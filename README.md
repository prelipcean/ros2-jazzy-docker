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

# Check display variable (usually :0 for X11, :1 for XWayland)
echo $DISPLAY

# Check if XWayland is running (for Wayland sessions)
pgrep -a Xwayland
```

### Quick Start with Docker Compose

#### For Wayland (Hyprland, Sway, GNOME Wayland, KDE Wayland)

Most Wayland compositors run XWayland for X11 app compatibility. ROS2 GUI apps use Qt which works great with XWayland.

```bash
# 1. Install xhost if not already installed
# Arch Linux:
sudo pacman -S xorg-xhost
# Ubuntu/Debian:
sudo apt install x11-xserver-utils
# Fedora:
sudo dnf install xorg-x11-server-utils

# 2. Allow Docker to access X server (run after each reboot)
xhost +local:docker

# 3. Start the container
docker compose up -d ros2-jazzy

# 4. Enter the container and run RViz
docker exec -it ros2_jazzy_dev bash
rviz2
```

#### For X11 Sessions

```bash
# 1. Allow Docker to access X server
xhost +local:docker

# 2. Start the container
docker compose up -d ros2-jazzy

# 3. Enter the container and run RViz
docker exec -it ros2_jazzy_dev bash
rviz2
```

### Persistent xhost Configuration

To avoid running `xhost +local:docker` after every reboot:

#### Hyprland (~/.config/hypr/hyprland.conf)
```bash
exec-once = xhost +local:docker
```

#### Sway (~/.config/sway/config)
```bash
exec xhost +local:docker
```

#### X11 (~/.xinitrc or ~/.xprofile)
```bash
xhost +local:docker
```

#### Systemd User Service
```bash
# Create ~/.config/systemd/user/xhost-docker.service
[Unit]
Description=Allow Docker access to X server
After=graphical-session.target

[Service]
Type=oneshot
ExecStart=/usr/bin/xhost +local:docker
RemainAfterExit=yes

[Install]
WantedBy=graphical-session.target

# Enable it
systemctl --user enable xhost-docker.service
```

### Docker Compose Services

The `docker-compose.yaml` includes three pre-configured services:

| Service | Description | Use Case |
|---------|-------------|----------|
| `ros2-jazzy` | Full desktop with GUI, host network | Development with RViz/Gazebo |
| `ros2-jazzy-vscode` | With VS Code Server on port 8800 | Remote development via browser |
| `ros2-jazzy-base` | Minimal ROS2 base, no GUI | Headless nodes, CI/CD |

```bash
# Run with GUI support (recommended for development)
docker compose up -d ros2-jazzy
docker exec -it ros2_jazzy_dev bash

# Run with VS Code Server
docker compose up -d ros2-jazzy-vscode
# Open http://localhost:8800 in browser

# Run minimal base
docker compose up -d ros2-jazzy-base
docker exec -it ros2_jazzy_base bash
```

### Manual Docker Run Commands

#### Wayland/XWayland
```bash
xhost +local:docker

docker run --rm -it \
    --name ros2_dev \
    --network host \
    -e DISPLAY=${DISPLAY:-:1} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    -v $(pwd)/workspace:/ros2_ws/src \
    ros2:jazzy-dev
```

#### X11
```bash
xhost +local:docker

docker run --rm -it \
    --name ros2_dev \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    -v $(pwd)/workspace:/ros2_ws/src \
    ros2:jazzy-dev
```

### Troubleshooting GUI Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| `cannot open display` | X server access denied | Run `xhost +local:docker` on host |
| `Authorization required, but no authorization protocol specified` | Missing X auth | Run `xhost +local:docker` on host |
| `could not connect to display :1` | Wrong DISPLAY or no xhost | Check `echo $DISPLAY` and run xhost |
| `No Qt platform plugin could be initialized` | Display not accessible | Ensure X11 socket is mounted and xhost is set |
| `wrong permissions on runtime directory /tmp` | Cosmetic warning | Can be ignored, doesn't affect functionality |
| Black screen in RViz | No GPU access | Add `-v /dev/dri:/dev/dri` to mount GPU |
| Slow rendering | Software rendering | Ensure `/dev/dri` is mounted for hardware acceleration |

### GPU Acceleration

#### Intel/AMD (Mesa)
```bash
# Already configured in docker-compose.yaml
# Just ensure /dev/dri is mounted
docker run --rm -it \
    -v /dev/dri:/dev/dri \
    ...
```

#### NVIDIA
```bash
# 1. Install nvidia-container-toolkit
# Arch: yay -S nvidia-container-toolkit
# Ubuntu: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

# 2. Run with GPU support
docker run --rm -it \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    ros2:jazzy-dev
```

### Environment Variables Reference

| Variable | Default | Description |
|----------|---------|-------------|
| `DISPLAY` | `:1` (Wayland) `:0` (X11) | X11 display to use |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain for multi-robot setups |
| `ROS_LOCALHOST_ONLY` | `0` | Restrict to localhost (deprecated) |
| `START_SSH` | `false` | Start SSH server on container start |
| `START_CODE_SERVER` | `false` | Start VS Code Server on container start |

## License

MIT License

## Author

Gheorghe Prelipcean <prelipcean.gheorghe@yahoo.com>

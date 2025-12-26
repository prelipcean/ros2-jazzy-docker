# ROS2 Jazzy Development Docker Image

A complete Docker development environment for ROS2 Jazzy based on Ubuntu Noble (24.04).

## Table of Contents

- [What is ROS2?](#what-is-ros2)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Build Options](#build-options)
- [Docker Compose Services](#docker-compose-services)
- [GUI Applications](#gui-applications-rviz-gazebo-etc)
- [Workspace Development](#workspace-development)
- [ROS2 Commands Cheatsheet](#ros2-commands-cheatsheet)
- [Configuration Reference](#configuration-reference)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## What is ROS2?

**ROS2 (Robot Operating System 2)** is an open-source robotics middleware framework. It provides:

- **Communication Infrastructure**: Publish/subscribe messaging between nodes
- **Hardware Abstraction**: Standard interfaces for sensors and actuators
- **Tools**: Visualization (RViz), simulation (Gazebo), debugging (rqt)
- **Packages**: Thousands of pre-built packages for navigation, manipulation, perception

**ROS2 Jazzy Jalisco** is a Long Term Support (LTS) release, supported until May 2029.

Learn more: [ROS2 Documentation](https://docs.ros.org/en/jazzy/)

---

## Features

| Feature | Description |
|---------|-------------|
| 🤖 **ROS2 Jazzy** | Full desktop or minimal base installation |
| 🛠️ **Development Tools** | colcon, rosdep, vcstool, ros-dev-tools |
| 💻 **VS Code Server** | Browser-based IDE with C++, CMake extensions |
| 🔐 **SSH Server** | Remote terminal access |
| 🖥️ **GUI Support** | X11/XWayland forwarding for RViz, Gazebo |
| 🏗️ **Multi-Architecture** | Supports amd64 and arm64 |
| 📦 **Common Packages** | ros2-control, ros2-controllers, xacro |

---

## Prerequisites

### Required

- **Docker**: Version 20.10 or later
  ```bash
  # Check Docker version
  docker --version
  
  # Install Docker (Ubuntu/Debian)
  curl -fsSL https://get.docker.com | sh
  sudo usermod -aG docker $USER
  ```

- **Docker Compose**: Version 2.0 or later (included with Docker Desktop)
  ```bash
  docker compose version
  ```

### For GUI Applications

- **xhost**: For X11 display access
  ```bash
  # Arch Linux
  sudo pacman -S xorg-xhost
  
  # Ubuntu/Debian
  sudo apt install x11-xserver-utils
  
  # Fedora
  sudo dnf install xorg-x11-server-utils
  ```

---

## Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/prelipcean/ros2-jazzy-docker.git
cd ros2-jazzy-docker
```

### 2. Allow GUI Access (Required for RViz/Gazebo)

```bash
# Run this after each system reboot
xhost +local:docker
```

### 3. Build and Run

```bash
# Build the image and start the container
docker compose up -d ros2-jazzy

# Enter the container
docker exec -it ros2_jazzy_dev bash

# Test ROS2
ros2 run demo_nodes_cpp talker
```

### 4. Run RViz (GUI Test)

```bash
# Option 1: Enter the container first, then run rviz2
docker exec -it ros2_jazzy_dev bash
rviz2

# Option 2: Run directly with environment sourced
docker exec -it ros2_jazzy_dev bash -c "source /opt/ros/jazzy/setup.bash && rviz2"
```

> **Note:** Running `docker exec -it ros2_jazzy_dev rviz2` directly will fail with
> `executable file not found in $PATH` because the ROS2 environment is only sourced
> by the entrypoint when starting the container, not when using `docker exec`.

---

## Build Options

### Using Docker Compose (Recommended)

```bash
# Build and run full desktop environment
docker compose up -d ros2-jazzy

# Build and run with VS Code Server
docker compose up -d ros2-jazzy-vscode

# Build and run minimal base
docker compose up -d ros2-jazzy-base
```

### Using Docker Build Directly

```bash
# Build full desktop image
docker build -t ros2:jazzy-dev .

# Build minimal base image (smaller, no GUI)
docker build --build-arg ROS_INSTALL_TYPE=ros-base -t ros2:jazzy-base .

# Build for ARM64 (Apple Silicon, Raspberry Pi, Jetson)
docker build --platform linux/arm64 --build-arg TARGETARCH=arm64 -t ros2:jazzy-dev .
```

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `UBUNTU_VERSION_TAG` | `noble` | Ubuntu base image (must be 24.04 for Jazzy) |
| `PASSWORD` | `temppwd` | Root password for SSH access |
| `TARGETARCH` | `amd64` | Target architecture: `amd64` or `arm64` |
| `ROS_DISTRO` | `jazzy` | ROS2 distribution name |
| `ROS_INSTALL_TYPE` | `desktop` | `desktop` (full) or `ros-base` (minimal) |
| `VS_CODE_SERVER_VERSION` | `4.93.1` | code-server version |
| `VS_CODE_SERVER_PORT` | `8800` | VS Code Server port |

---

## Docker Compose Services

The `docker-compose.yaml` provides three pre-configured services:

| Service | Container Name | Use Case | Network | GUI |
|---------|---------------|----------|---------|-----|
| `ros2-jazzy` | ros2_jazzy_dev | Development with RViz/Gazebo | Host | ✅ |
| `ros2-jazzy-vscode` | ros2_jazzy_vscode | Remote development via browser | Ports 8800, 2222 | ✅ |
| `ros2-jazzy-base` | ros2_jazzy_base | Headless nodes, CI/CD | Host | ❌ |

### Service Commands

```bash
# Start a service in the background
docker compose up -d ros2-jazzy

# View logs
docker compose logs -f ros2-jazzy

# Enter running container
docker exec -it ros2_jazzy_dev bash

# Stop a service
docker compose stop ros2-jazzy

# Stop and remove all services
docker compose down

# Rebuild after Dockerfile changes
docker compose build --no-cache ros2-jazzy
```

### Using VS Code Server

```bash
# Start VS Code Server service
docker compose up -d ros2-jazzy-vscode

# Open in browser
# http://localhost:8800

# SSH access (password: temppwd)
ssh root@localhost -p 2222
```

---

## GUI Applications (RViz, Gazebo, etc.)

### Understanding Display Servers

```bash
# Check your display server type
echo $XDG_SESSION_TYPE
# Output: "x11" or "wayland"

# Check your DISPLAY variable
echo $DISPLAY
# Output: ":0" (X11) or ":1" (XWayland on Wayland)

# Check if XWayland is running (Wayland only)
pgrep -a Xwayland
```

### Setting Up GUI Access

#### For Wayland (Hyprland, Sway, GNOME Wayland, KDE Wayland)

Most Wayland compositors run XWayland for X11 compatibility. ROS2 GUI apps work great with XWayland.

```bash
# 1. Install xhost
# Arch: sudo pacman -S xorg-xhost
# Ubuntu: sudo apt install x11-xserver-utils

# 2. Allow Docker to access X server (run after each reboot)
xhost +local:docker

# 3. Start container and run RViz
docker compose up -d ros2-jazzy
docker exec -it ros2_jazzy_dev bash
rviz2
```

#### For X11

```bash
# 1. Allow Docker to access X server
xhost +local:docker

# 2. Start container and run RViz
docker compose up -d ros2-jazzy
docker exec -it ros2_jazzy_dev bash
rviz2
```

### Persistent xhost Configuration

To avoid running `xhost +local:docker` after every reboot:

<details>
<summary><b>Hyprland</b> (~/.config/hypr/hyprland.conf)</summary>

```bash
exec-once = xhost +local:docker
```
</details>

<details>
<summary><b>Sway</b> (~/.config/sway/config)</summary>

```bash
exec xhost +local:docker
```
</details>

<details>
<summary><b>X11</b> (~/.xinitrc or ~/.xprofile)</summary>

```bash
xhost +local:docker
```
</details>

<details>
<summary><b>Systemd User Service</b></summary>

Create `~/.config/systemd/user/xhost-docker.service`:
```ini
[Unit]
Description=Allow Docker access to X server
After=graphical-session.target

[Service]
Type=oneshot
ExecStart=/usr/bin/xhost +local:docker
RemainAfterExit=yes

[Install]
WantedBy=graphical-session.target
```

Enable it:
```bash
systemctl --user enable xhost-docker.service
```
</details>

### GPU Acceleration

#### Intel/AMD (Mesa Drivers)

Already configured in docker-compose.yaml with `/dev/dri` mount.

#### NVIDIA

```bash
# 1. Install nvidia-container-toolkit
# Arch: yay -S nvidia-container-toolkit
# Ubuntu: See https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

# 2. Run with GPU support
docker run --rm -it \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    ros2:jazzy-dev
```

---

## Workspace Development

### Directory Structure

```
ros2-jazzy-docker/
├── Dockerfile              # Image definition
├── docker-compose.yaml     # Service configurations
├── scripts/
│   └── entrypoint.sh       # Container startup script
└── workspace/              # ← Your ROS2 packages go here!
    └── my_package/
        ├── package.xml
        ├── CMakeLists.txt
        └── src/
```

### Development Workflow

```bash
# 1. Create a new package (inside container)
cd /ros2_ws/src
ros2 pkg create --build-type ament_cmake my_package

# 2. Edit your code
# Files in ./workspace/ on host = /ros2_ws/src/ in container

# 3. Build the workspace
cd /ros2_ws
colcon build

# 4. Source the workspace (required after every build)
source install/setup.bash

# 5. Run your node
ros2 run my_package my_node
```

### Installing Dependencies

```bash
# Inside the container
cd /ros2_ws

# Install dependencies from package.xml files
rosdep install --from-paths src --ignore-src -y

# Build
colcon build
```

### Useful Build Commands

```bash
# Build a specific package
colcon build --packages-select my_package

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build and show compile commands (for IDEs)
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Clean build
rm -rf build/ install/ log/
colcon build
```

---

## ROS2 Commands Cheatsheet

### Node Management

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /my_node

# Run a node
ros2 run <package_name> <node_name>
```

### Topics

```bash
# List topics
ros2 topic list

# Show topic info
ros2 topic info /topic_name

# Echo topic messages
ros2 topic echo /topic_name

# Publish to topic
ros2 topic pub /topic_name std_msgs/msg/String "{data: 'hello'}"

# Check publish rate
ros2 topic hz /topic_name
```

### Services

```bash
# List services
ros2 service list

# Call a service
ros2 service call /service_name std_srvs/srv/Empty
```

### Parameters

```bash
# List parameters
ros2 param list

# Get parameter
ros2 param get /node_name param_name

# Set parameter
ros2 param set /node_name param_name value
```

### Launch Files

```bash
# Run a launch file
ros2 launch <package_name> <launch_file.py>

# Run with arguments
ros2 launch my_package my_launch.py arg:=value
```

### Building

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_package

# Clean and build
rm -rf build/ install/ log/ && colcon build
```

---

## Configuration Reference

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `DISPLAY` | `:1` | X11 display for GUI apps |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain (0-232) for network isolation |
| `ROS_LOCALHOST_ONLY` | `0` | `1` = localhost only, `0` = allow network |
| `START_SSH` | `false` | Start SSH server on container start |
| `START_CODE_SERVER` | `false` | Start VS Code Server on container start |
| `VS_CODE_SERVER_PORT` | `8800` | VS Code Server web interface port |

### Included ROS2 Packages

- `ros-jazzy-desktop` - Full ROS2 installation (or `ros-jazzy-ros-base`)
- `ros-jazzy-ament-cmake` - CMake build tools
- `ros-jazzy-ros2-control` - Robot control framework
- `ros-jazzy-ros2-controllers` - Standard controllers
- `ros-jazzy-xacro` - XML macro language for URDF

### Container Directories

| Path | Description |
|------|-------------|
| `/ros2_ws` | ROS2 workspace (WORKDIR) |
| `/ros2_ws/src` | Source packages (mounted from `./workspace/`) |
| `/opt/ros/jazzy` | ROS2 installation |
| `/workspace` | Additional workspace for non-ROS files |

---

## Troubleshooting

### GUI Issues

| Issue | Solution |
|-------|----------|
| `cannot open display` | Run `xhost +local:docker` on host |
| `Authorization required, but no authorization protocol specified` | Run `xhost +local:docker` on host |
| `could not connect to display :1` | Check `echo $DISPLAY` and ensure xhost is set |
| `No Qt platform plugin could be initialized` | Ensure X11 socket is mounted and xhost is set |
| `wrong permissions on runtime directory /tmp` | Cosmetic warning, can be ignored |
| Black screen in RViz | Ensure `/dev/dri` is mounted for GPU access |
| Slow rendering | Ensure `/dev/dri` is mounted for hardware acceleration |

### ROS2 Issues

| Issue | Solution |
|-------|----------|
| `ros2: command not found` | Run `source /opt/ros/jazzy/setup.bash` |
| `executable file not found in $PATH` when using `docker exec` | Use `docker exec -it container bash` first, or `docker exec -it container bash -c "source /opt/ros/jazzy/setup.bash && command"` |
| Nodes can't find each other | Ensure same `ROS_DOMAIN_ID` and network mode |
| Package not found after build | Run `source /ros2_ws/install/setup.bash` |
| `rosdep install` fails | Run `rosdep update` first |

### Docker Issues

| Issue | Solution |
|-------|----------|
| Permission denied on `/dev/dri` | Add user to `video` and `render` groups |
| Container exits immediately | Check `docker logs ros2_jazzy_dev` |
| Can't connect to SSH | Ensure `START_SSH=true` and correct port |

---

## License

MIT License

## Author

Gheorghe Prelipcean <prelipcean.gheorghe@yahoo.com>

## Links

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [GitHub Repository](https://github.com/prelipcean/ros2-jazzy-docker)
- [Docker Hub](https://hub.docker.com/) (optional: publish your image)

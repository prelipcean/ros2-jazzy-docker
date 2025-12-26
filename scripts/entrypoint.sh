#!/bin/bash
#===============================================================================
# ROS2 Jazzy Docker Container Entrypoint Script
#===============================================================================
# This script runs when the container starts. It:
# 1. Sources the ROS2 environment so ros2 commands work
# 2. Sources your workspace if you've built one
# 3. Optionally starts SSH server for remote access
# 4. Optionally starts VS Code Server for browser-based IDE
# 5. Executes any command passed to the container, or starts bash
#===============================================================================

set -e  # Exit immediately if a command fails

#-------------------------------------------------------------------------------
# Source ROS2 Environment
#-------------------------------------------------------------------------------
# This makes ros2 commands available (ros2 run, ros2 topic, etc.)
# The setup.bash file sets up PATH, PYTHONPATH, and other environment variables
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "✓ Sourced ROS2 ${ROS_DISTRO} environment"
fi

#-------------------------------------------------------------------------------
# Source Workspace (if built)
#-------------------------------------------------------------------------------
# If you've built packages with 'colcon build', source them to use your nodes
# The install/setup.bash is created after running 'colcon build' in /ros2_ws
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
    echo "✓ Sourced workspace at /ros2_ws"
fi

#-------------------------------------------------------------------------------
# Start SSH Server (optional)
#-------------------------------------------------------------------------------
# Enable remote SSH access to the container
# Set START_SSH=true environment variable to enable
# Connect with: ssh root@<host-ip> -p 22 (or port 2222 for ros2-jazzy-vscode)
# Default password: temppwd (change with PASSWORD build arg)
if [ "$START_SSH" = "true" ]; then
    /usr/sbin/sshd
    echo "✓ SSH server started on port 22"
fi

#-------------------------------------------------------------------------------
# Start VS Code Server (optional)
#-------------------------------------------------------------------------------
# Enable browser-based VS Code IDE
# Set START_CODE_SERVER=true environment variable to enable
# Access at: http://localhost:8800 (or configured VS_CODE_SERVER_PORT)
if [ "$START_CODE_SERVER" = "true" ]; then
    code-server --bind-addr 0.0.0.0:${VS_CODE_SERVER_PORT:-8800} --auth none &
    echo "✓ VS Code Server started at http://0.0.0.0:${VS_CODE_SERVER_PORT:-8800}"
fi

#-------------------------------------------------------------------------------
# Execute Command or Start Interactive Shell
#-------------------------------------------------------------------------------
# If arguments are passed (e.g., docker run ros2:jazzy-dev ros2 topic list)
# execute them. Otherwise, start an interactive bash shell.
if [ $# -eq 0 ]; then
    echo ""
    echo "ROS2 ${ROS_DISTRO} Development Environment Ready!"
    echo "Run 'ros2 --help' to get started."
    echo ""
    exec /bin/bash
else
    exec "$@"
fi

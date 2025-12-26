#===============================================================================
#                    ROS2 Jazzy Development Docker Image
#===============================================================================
# 
# This Dockerfile creates a complete ROS2 Jazzy development environment with:
#   - ROS2 Jazzy (desktop or base installation)
#   - Development tools (colcon, rosdep, vcstool)
#   - VS Code Server for browser-based IDE
#   - SSH server for remote access
#   - Common ROS2 packages (ros2-control, xacro, etc.)
#
# Build:
#   docker build -t ros2:jazzy-dev .
#   docker build --build-arg ROS_INSTALL_TYPE=ros-base -t ros2:jazzy-base .
#
# Run:
#   docker run --rm -it ros2:jazzy-dev
#
# For more information, see README.md
#
#===============================================================================

#===============================================================================
# BUILD ARGUMENTS (Customize these when building)
#===============================================================================

# Ubuntu base image version
# ROS2 Jazzy requires Ubuntu 24.04 (Noble Numbat)
# See: https://docs.ros.org/en/jazzy/Installation.html
ARG UBUNTU_VERSION_TAG=noble

# Root password for SSH access
# WARNING: Change this for production use!
# Better: Use SSH keys instead of password authentication
ARG PASSWORD="temppwd"

# wget arguments for downloading files
# -q: quiet, --show-progress: show progress bar, --progress=bar:force:noscroll: force progress bar
ARG WGET_ARGS="-q --show-progress --progress=bar:force:noscroll"

# Target CPU architecture
# Supported: amd64 (x86_64), arm64 (aarch64/Apple Silicon)
# Docker automatically sets this with --platform flag
ARG TARGETARCH=amd64

# ROS2 Distribution name
# See: https://docs.ros.org/en/rolling/Releases.html
ARG ROS_DISTRO=jazzy

# ROS2 Installation type:
#   - desktop: Full installation with RViz, Gazebo, demos (~2GB)
#   - ros-base: Minimal installation, CLI tools only (~500MB)
ARG ROS_INSTALL_TYPE=desktop

# VS Code Server (code-server) version
# See releases: https://github.com/coder/code-server/releases
ARG VS_CODE_SERVER_VERSION=4.93.1

# Port for VS Code Server web interface
ARG VS_CODE_SERVER_PORT=8800

# VS Code Extension versions
# C/C++ IntelliSense: https://github.com/microsoft/vscode-cpptools/releases
ARG VS_CODE_EXT_CPPTOOLS_VERSION=1.22.10
# Hex Editor: https://marketplace.visualstudio.com/items?itemName=ms-vscode.hexeditor
ARG VS_CODE_EXT_HEX_EDITOR_VERSION=1.11.1
# CMake Tools: https://github.com/microsoft/vscode-cmake-tools/releases
ARG VS_CODE_EXT_CMAKETOOLS_VERSION=1.19.52

#===============================================================================
# BASE IMAGE
#===============================================================================
FROM ubuntu:${UBUNTU_VERSION_TAG}

# Re-declare ARGs after FROM (Docker requirement)
# ARG values don't persist across FROM instructions
ARG PASSWORD
ARG WGET_ARGS
ARG TARGETARCH
ARG ROS_DISTRO
ARG ROS_INSTALL_TYPE
ARG VS_CODE_SERVER_VERSION
ARG VS_CODE_SERVER_PORT
ARG VS_CODE_EXT_CPPTOOLS_VERSION
ARG VS_CODE_EXT_HEX_EDITOR_VERSION
ARG VS_CODE_EXT_CMAKETOOLS_VERSION

# Use bash for RUN commands (needed for [[ ]] syntax and source command)
SHELL ["/bin/bash", "-c"]

# Validate target architecture
RUN valid_archs="amd64 arm64"; \
    if [[ " $valid_archs " == *" $TARGETARCH "* ]]; then \
        echo "✓ Architecture $TARGETARCH is supported."; \
    else \
        echo "✗ Error: Unsupported architecture: $TARGETARCH"; \
        echo "  Valid architectures are: $valid_archs"; \
        exit 1; \
    fi

# Prevent apt from prompting for user input during package installation
ENV DEBIAN_FRONTEND=noninteractive

#===============================================================================
# SYSTEM LOCALE CONFIGURATION
#===============================================================================
# ROS2 requires UTF-8 locale for proper string handling
# Without this, you may see encoding errors in node names and messages
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

#===============================================================================
# BASE DEVELOPMENT PACKAGES
#===============================================================================
# These are common development tools, not ROS2-specific
RUN apt-get -y update && \
    apt-get install --no-install-recommends -y \
        build-essential \
        cmake \
        pkg-config \
        git \
        ca-certificates \
        curl \
        wget \
        gnupg \
        openssh-server \
        net-tools \
        sudo \
        file \
        lsb-release \
        dos2unix \
        unzip \
        xz-utils \
        python3 \
        python3-pip \
        python3-venv \
        vim \
        mc \
        tree \
        tmux \
        software-properties-common

#===============================================================================
# ROS2 REPOSITORY SETUP
#===============================================================================
# Add the official ROS2 apt repository to get ROS2 packages

# Enable Ubuntu Universe repository (contains dependencies)
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe

# Install ros2-apt-source package (provides GPG keys and apt sources)
# This is the official way to add ROS2 repositories as of 2024
# See: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
RUN apt-get update && apt-get install -y curl && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

#===============================================================================
# ROS2 DEVELOPMENT TOOLS
#===============================================================================
# These tools are used for building and managing ROS2 packages
# - colcon: Build tool for ROS2 packages
# - rosdep: Installs package dependencies
# - vcstool: Manages multiple git repositories
RUN apt-get update && apt-get install -y ros-dev-tools

#===============================================================================
# ROS2 INSTALLATION
#===============================================================================
# Install ROS2 Jazzy (desktop or base depending on ROS_INSTALL_TYPE)
# 
# desktop includes:
#   - RViz (3D visualization)
#   - Gazebo (simulation) 
#   - Demo nodes and tutorials
#   - rqt tools (GUI debugging)
#
# ros-base includes:
#   - Core libraries (rclcpp, rclpy)
#   - Message packages (std_msgs, geometry_msgs, etc.)
#   - CLI tools (ros2 topic, ros2 node, etc.)
RUN apt-get update && apt-get upgrade -y && \
    if [ "$ROS_INSTALL_TYPE" = "desktop" ]; then \
        echo "Installing ROS2 ${ROS_DISTRO} Desktop (full installation)..." && \
        apt-get install -y ros-${ROS_DISTRO}-desktop; \
    else \
        echo "Installing ROS2 ${ROS_DISTRO} Base (minimal installation)..." && \
        apt-get install -y ros-${ROS_DISTRO}-ros-base; \
    fi

#===============================================================================
# ADDITIONAL ROS2 PACKAGES
#===============================================================================
# Common packages used in robotics development
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro

#===============================================================================
# ROSDEP INITIALIZATION
#===============================================================================
# rosdep is used to install dependencies for ROS2 packages
# 'rosdep init' creates /etc/ros/rosdep/sources.list.d/
# 'rosdep update' downloads the dependency database
# || true: Don't fail if already initialized (idempotent)
RUN rosdep init || true && \
    rosdep update --rosdistro ${ROS_DISTRO}

#===============================================================================
# USER CONFIGURATION
#===============================================================================
# Set root password for SSH access
# WARNING: For production, use SSH keys instead!
RUN echo "root:${PASSWORD}" | chpasswd

#===============================================================================
# CLEANUP
#===============================================================================
# Remove apt cache to reduce image size
# This can save 100-500MB depending on packages installed
RUN apt-get clean -y && \
    apt-get autoremove --purge -y && \
    rm -rf /var/lib/apt/lists/*

#===============================================================================
# DIRECTORY STRUCTURE
#===============================================================================
# /ros2_ws/src - Your ROS2 packages go here
# /workspace   - Additional workspace for non-ROS files
RUN mkdir -p /ros2_ws/src && \
    mkdir -p /workspace

#===============================================================================
# SSH SERVER CONFIGURATION
#===============================================================================
# Configure SSH for remote access to the container
# Useful for development from another machine or IDE remote development

# Create SSH runtime directory
RUN mkdir -p /var/run/sshd && \
    chmod 0755 /var/run/sshd

# Allow root login via SSH (required since we run as root)
# WARNING: For production, create a non-root user!
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Expose SSH port
EXPOSE 22

#===============================================================================
# ENVIRONMENT CONFIGURATION
#===============================================================================
# Midnight Commander dark theme (file manager)
ENV MC_SKIN=dark

# Store ROS2 distribution for entrypoint script
ENV ROS_DISTRO=${ROS_DISTRO}

# Add ROS2 setup to bashrc for interactive shells
# This ensures ros2 commands work when you 'docker exec -it <container> bash'
RUN echo "" >> /root/.bashrc && \
    echo "# ROS2 Environment Setup" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# Source workspace if built (uncomment after running colcon build)" >> /root/.bashrc && \
    echo "# source /ros2_ws/install/setup.bash" >> /root/.bashrc

#===============================================================================
# VS CODE SERVER INSTALLATION
#===============================================================================
# code-server provides a browser-based VS Code IDE
# Access at http://localhost:8800 when START_CODE_SERVER=true

ENV VS_CODE_SERVER_VERSION=${VS_CODE_SERVER_VERSION}
ENV VS_CODE_SERVER_PORT=${VS_CODE_SERVER_PORT}

# Install code-server
RUN cd /tmp && \
    wget ${WGET_ARGS} https://code-server.dev/install.sh && \
    chmod +x install.sh && \
    bash install.sh --version ${VS_CODE_SERVER_VERSION}

# Download VS Code extensions
# Note: Using curl -fsSL to properly follow GitHub redirects
# -f: fail silently on HTTP errors
# -s: silent mode
# -S: show errors
# -L: follow redirects
RUN cd /tmp && \
    if [ "$TARGETARCH" = "amd64" ]; then \
        curl -fsSL https://github.com/microsoft/vscode-cpptools/releases/download/v${VS_CODE_EXT_CPPTOOLS_VERSION}/cpptools-linux-x64.vsix -o cpptools.vsix; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
        curl -fsSL https://github.com/microsoft/vscode-cpptools/releases/download/v${VS_CODE_EXT_CPPTOOLS_VERSION}/cpptools-linux-arm64.vsix -o cpptools.vsix; \
    else \
        echo "Unsupported architecture: $TARGETARCH"; \
        exit 1; \
    fi && \
    curl -fsSL https://github.com/microsoft/vscode-cmake-tools/releases/download/v${VS_CODE_EXT_CMAKETOOLS_VERSION}/cmake-tools.vsix -o cmake-tools.vsix && \
    curl -fsSL --compressed https://marketplace.visualstudio.com/_apis/public/gallery/publishers/ms-vscode/vsextensions/hexeditor/${VS_CODE_EXT_HEX_EDITOR_VERSION}/vspackage -o hexeditor.vsix

# Install VS Code extensions
RUN cd /tmp && \
    code-server --install-extension cpptools.vsix && \
    code-server --install-extension cmake-tools.vsix && \
    code-server --install-extension hexeditor.vsix

# Clean up installation files
RUN cd /tmp && \
    rm -f install.sh cpptools.vsix cmake-tools.vsix hexeditor.vsix

#===============================================================================
# WORKING DIRECTORY
#===============================================================================
# Set the default directory when entering the container
WORKDIR /ros2_ws

#===============================================================================
# ENTRYPOINT SCRIPT
#===============================================================================
# Copy and set up the entrypoint script
# The entrypoint script:
#   1. Sources ROS2 environment
#   2. Sources your workspace if built
#   3. Optionally starts SSH and VS Code servers
#   4. Runs your command or starts bash
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    dos2unix /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#===============================================================================
# EXPOSED PORTS
#===============================================================================
# VS Code Server web interface
EXPOSE ${VS_CODE_SERVER_PORT}

# ROS2 DDS Discovery ports (for multi-machine communication)
# These are used by Fast DDS (default RMW) for node discovery
EXPOSE 7400/udp
EXPOSE 7401/udp

#===============================================================================
# HEALTHCHECK
#===============================================================================
# Verify ROS2 is working by checking if ros2 command exists
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 --help > /dev/null || exit 1

#===============================================================================
# IMAGE METADATA
#===============================================================================
LABEL maintainer="Gheorghe Prelipcean <prelipcean.gheorghe@yahoo.com>"
LABEL description="ROS2 Jazzy Development Environment"
LABEL ros.distro="jazzy"
LABEL ubuntu.version="24.04"
LABEL org.opencontainers.image.source="https://github.com/prelipcean/ros2-jazzy-docker"

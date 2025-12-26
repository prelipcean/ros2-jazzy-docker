### ROS2 Jazzy Development Docker Image ###

## SETTINGS ##

# Ubuntu base image version (ROS2 Jazzy requires Ubuntu Noble 24.04)
ARG UBUNTU_VERSION_TAG=noble

# Root password
ARG PASSWORD="temppwd"

# wget arguments
ARG WGET_ARGS="-q --show-progress --progress=bar:force:noscroll"

# Target Architecture
ARG TARGETARCH=amd64

# ROS2 Distribution
ARG ROS_DISTRO=jazzy

# ROS2 Install type: desktop or ros-base
ARG ROS_INSTALL_TYPE=desktop

# VS Code Server version
ARG VS_CODE_SERVER_VERSION=4.93.1
ARG VS_CODE_SERVER_PORT=8800
ARG VS_CODE_EXT_CPPTOOLS_VERSION=1.22.10
ARG VS_CODE_EXT_HEX_EDITOR_VERSION=1.11.1
ARG VS_CODE_EXT_CMAKETOOLS_VERSION=1.19.52

## BASE IMAGE ##
FROM ubuntu:${UBUNTU_VERSION_TAG}

# Redeclare arguments after FROM
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

# Set default shell during Docker image build to bash
SHELL ["/bin/bash", "-c"]

# Check if the target architecture is either x86_64 (amd64) or arm64 (aarch64)
RUN valid_archs="amd64 arm64"; \
    if [[ " $valid_archs " == *" $TARGETARCH "* ]]; then \
        echo "Architecture $TARGETARCH is supported."; \
    else \
        echo "Error: Unsupported architecture: $TARGETARCH. Valid architectures are: $valid_archs"; \
        exit 1; \
    fi

# Set non-interactive frontend for apt-get to skip any user confirmations
ENV DEBIAN_FRONTEND=noninteractive

#-------------------------------------------------------------------------------
# Initialize system locale (required by ROS2)
#-------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

#-------------------------------------------------------------------------------
# Install base development packages
#-------------------------------------------------------------------------------
RUN apt-get -y update && \
    apt-get install --no-install-recommends -y \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        dos2unix \
        file \
        git \
        gnupg \
        lsb-release \
        mc \
        net-tools \
        openssh-server \
        pkg-config \
        python3 \
        python3-pip \
        python3-venv \
        software-properties-common \
        sudo \
        tmux \
        tree \
        unzip \
        vim \
        wget \
        xz-utils

#-------------------------------------------------------------------------------
# Enable required repositories for ROS2
#-------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe

# Install ros2-apt-source package to configure ROS 2 repositories
RUN apt-get update && apt-get install -y curl && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

#-------------------------------------------------------------------------------
# Install ROS2 Development Tools
#-------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y ros-dev-tools

#-------------------------------------------------------------------------------
# Install ROS2 Jazzy
#-------------------------------------------------------------------------------
RUN apt-get update && apt-get upgrade -y && \
    if [ "$ROS_INSTALL_TYPE" = "desktop" ]; then \
        apt-get install -y ros-${ROS_DISTRO}-desktop; \
    else \
        apt-get install -y ros-${ROS_DISTRO}-ros-base; \
    fi

#-------------------------------------------------------------------------------
# Install additional ROS2 packages (common dependencies)
#-------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro

#-------------------------------------------------------------------------------
# Initialize rosdep
#-------------------------------------------------------------------------------
RUN rosdep init || true && \
    rosdep update --rosdistro ${ROS_DISTRO}

#-------------------------------------------------------------------------------
# Set root password
#-------------------------------------------------------------------------------
RUN echo "root:${PASSWORD}" | chpasswd

#-------------------------------------------------------------------------------
# Clean up stale packages
#-------------------------------------------------------------------------------
RUN apt-get clean -y && \
    apt-get autoremove --purge -y && \
    rm -rf /var/lib/apt/lists/*

#-------------------------------------------------------------------------------
# Set up directories
#-------------------------------------------------------------------------------
RUN mkdir -p /ros2_ws/src && \
    mkdir -p /workspace

#-------------------------------------------------------------------------------
# Set up SSH server
#-------------------------------------------------------------------------------
RUN mkdir -p /var/run/sshd && \
    chmod 0755 /var/run/sshd

# Allow root login via SSH
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Expose SSH port
EXPOSE 22

#-------------------------------------------------------------------------------
# Use the "dark" theme for Midnight Commander
#-------------------------------------------------------------------------------
ENV MC_SKIN=dark

#-------------------------------------------------------------------------------
# ROS2 Environment Setup
#-------------------------------------------------------------------------------
ENV ROS_DISTRO=${ROS_DISTRO}

# Add ROS2 sourcing to bashrc for interactive shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "# Uncomment the following line if you have a workspace" >> /root/.bashrc && \
    echo "# source /ros2_ws/install/setup.bash" >> /root/.bashrc

#-------------------------------------------------------------------------------
# VS Code Server
#-------------------------------------------------------------------------------
# Set VS Code Server environment variables
ENV VS_CODE_SERVER_VERSION=${VS_CODE_SERVER_VERSION}
ENV VS_CODE_SERVER_PORT=${VS_CODE_SERVER_PORT}

# Install VS Code Server
RUN cd /tmp && \
    wget ${WGET_ARGS} https://code-server.dev/install.sh && \
    chmod +x install.sh && \
    bash install.sh --version ${VS_CODE_SERVER_VERSION}

# Download VS Code extensions (using curl -L to follow redirects properly)
RUN cd /tmp && \
    if [ "$TARGETARCH" = "amd64" ]; then \
        curl -fsSL https://github.com/microsoft/vscode-cpptools/releases/download/v${VS_CODE_EXT_CPPTOOLS_VERSION}/cpptools-linux-x64.vsix -o cpptools.vsix; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
        curl -fsSL https://github.com/microsoft/vscode-cpptools/releases/download/v${VS_CODE_EXT_CPPTOOLS_VERSION}/cpptools-linux-arm64.vsix -o cpptools.vsix; \
    else \
        echo "Unsupported architecture"; \
        exit 1; \
    fi && \
    curl -fsSL https://github.com/microsoft/vscode-cmake-tools/releases/download/v${VS_CODE_EXT_CMAKETOOLS_VERSION}/cmake-tools.vsix -o cmake-tools.vsix && \
    curl -fsSL --compressed https://marketplace.visualstudio.com/_apis/public/gallery/publishers/ms-vscode/vsextensions/hexeditor/${VS_CODE_EXT_HEX_EDITOR_VERSION}/vspackage -o hexeditor.vsix

# Install extensions
RUN cd /tmp && \
    code-server --install-extension cpptools.vsix && \
    code-server --install-extension cmake-tools.vsix && \
    code-server --install-extension hexeditor.vsix

# Clean up VS Code installation files
RUN cd /tmp && \
    rm -f install.sh cpptools.vsix cmake-tools.vsix hexeditor.vsix

#-------------------------------------------------------------------------------
# Copy configuration files (optional - uncomment if you have these files)
#-------------------------------------------------------------------------------
# COPY scripts/vs_code.code-workspace /vs_code.code-workspace

#-------------------------------------------------------------------------------
# Set working directory
#-------------------------------------------------------------------------------
WORKDIR /ros2_ws

#-------------------------------------------------------------------------------
# Entrypoint
#-------------------------------------------------------------------------------
# Create entrypoint script
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Source ROS2 environment' >> /entrypoint.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Source workspace if it exists' >> /entrypoint.sh && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then' >> /entrypoint.sh && \
    echo '    source /ros2_ws/install/setup.bash' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Start SSH service if requested' >> /entrypoint.sh && \
    echo 'if [ "$START_SSH" = "true" ]; then' >> /entrypoint.sh && \
    echo '    /usr/sbin/sshd' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Start code-server if requested' >> /entrypoint.sh && \
    echo 'if [ "$START_CODE_SERVER" = "true" ]; then' >> /entrypoint.sh && \
    echo '    code-server --bind-addr 0.0.0.0:${VS_CODE_SERVER_PORT:-8800} --auth none &' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Execute command or start bash' >> /entrypoint.sh && \
    echo 'if [ $# -eq 0 ]; then' >> /entrypoint.sh && \
    echo '    exec /bin/bash' >> /entrypoint.sh && \
    echo 'else' >> /entrypoint.sh && \
    echo '    exec "$@"' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#-------------------------------------------------------------------------------
# Expose ports
#-------------------------------------------------------------------------------
# VS Code Server port
EXPOSE ${VS_CODE_SERVER_PORT}

# ROS2 DDS Discovery ports (optional, for multi-machine setups)
EXPOSE 7400/udp
EXPOSE 7401/udp

#-------------------------------------------------------------------------------
# Labels
#-------------------------------------------------------------------------------
LABEL maintainer="Gheorghe Prelipcean <prelipcean.gheorghe@yahoo.com>"
LABEL description="ROS2 Jazzy Development Environment"
LABEL ros.distro="jazzy"
LABEL ubuntu.version="24.04"

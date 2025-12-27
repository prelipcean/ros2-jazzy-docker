# 🐳 Docker Complete Cheatsheet

A comprehensive Docker reference guide with examples, best practices, and commonly used commands.

---

## 📋 Table of Contents

1. [Installation & Setup](#installation--setup)
2. [Basic Commands](#basic-commands)
3. [Images](#images)
4. [Containers](#containers)
5. [Volumes & Bind Mounts](#volumes--bind-mounts)
6. [Networking](#networking)
7. [Dockerfile Reference](#dockerfile-reference)
8. [Docker Compose](#docker-compose)
9. [Docker Registry](#docker-registry)
10. [GPU Support (NVIDIA)](#gpu-support-nvidia)
11. [System Management](#system-management)
12. [Debugging & Troubleshooting](#debugging--troubleshooting)
13. [Best Practices](#best-practices)
14. [ROS2 Specific Examples](#ros2-specific-examples)

---

## Installation & Setup

### Install Docker on Ubuntu/Debian

```bash
# Remove old versions
sudo apt remove docker docker-engine docker.io containerd runc

# Install prerequisites
sudo apt update
sudo apt install ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add current user to docker group (avoid using sudo)
sudo usermod -aG docker $USER
newgrp docker  # Apply group changes without logout
```

### Verify Installation

```bash
# Check Docker version
docker --version
# Output: Docker version 24.0.x, build xxxxxxx

# Detailed version info
docker version

# System-wide information
docker info

# Run hello-world to verify
docker run hello-world
```

---

## Basic Commands

### Help & Information

```bash
# Get help for any command
docker --help
docker <command> --help

# Examples
docker run --help
docker build --help
docker compose --help
```

### Common Options Reference

| Option | Description |
|--------|-------------|
| `--rm` | Automatically remove container when it exits |
| `-it` | Interactive mode with TTY (terminal) |
| `-d` | Detached mode (run in background) |
| `--name` | Assign a name to the container |
| `-p` | Port mapping (host:container) |
| `-v` | Volume/bind mount |
| `-e` | Set environment variable |
| `--network` | Connect to a network |
| `--restart` | Restart policy |
| `--privileged` | Give extended privileges |

---

## Images

### Pulling Images

```bash
# Pull an image from Docker Hub
docker pull python                    # Latest tag
docker pull python:3.11              # Specific version
docker pull python:3.11-slim         # Slim variant
docker pull python:3.11-alpine       # Alpine variant (smallest)

# Pull ROS2 images
docker pull ros:jazzy-ros-core       # Minimal ROS2
docker pull ros:jazzy-ros-base       # Base installation
docker pull ros:humble-ros-base      # ROS2 Humble
docker pull ros:humble-desktop       # Full desktop with RViz

# Pull from other registries
docker pull ghcr.io/user/image:tag   # GitHub Container Registry
docker pull mcr.microsoft.com/dotnet/sdk:8.0  # Microsoft Registry
```

### Listing Images

```bash
# List all images
docker images
docker image ls

# List with specific format
docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}"

# List image IDs only
docker images -q

# List dangling images (untagged)
docker images -f "dangling=true"

# Filter images
docker images --filter "reference=ros*"
```

### Building Images

```bash
# Build from Dockerfile in current directory
docker build -t myimage:latest .

# Build with specific Dockerfile
docker build -f Dockerfile_slim -t hello-python:slim .

# Build with build arguments
docker build --build-arg VERSION=1.0 -t myapp:1.0 .

# Build without cache (fresh build)
docker build --no-cache -t myimage:latest .

# Build for specific platform
docker build --platform linux/amd64 -t myimage:amd64 .
docker build --platform linux/arm64 -t myimage:arm64 .

# Multi-platform build (requires buildx)
docker buildx build --platform linux/amd64,linux/arm64 -t myimage:multi .
```

### Managing Images

```bash
# Tag an image
docker tag source_image:tag target_image:tag
docker tag myapp:latest myregistry.com/myapp:v1.0

# Remove an image
docker rmi python:3.11
docker rmi -f image_id         # Force remove

# Remove all unused images
docker image prune             # Dangling only
docker image prune -a          # All unused

# Image history (see layers)
docker history myimage:latest

# Inspect image details
docker inspect myimage:latest

# Save image to tar file
docker save -o myimage.tar myimage:latest

# Load image from tar file
docker load -i myimage.tar

# Export flattened image (from container)
docker export container_name > container.tar
```

---

## Containers

### Running Containers

```bash
# Basic run
docker run python python --version

# Interactive container with bash
docker run -it python bash

# Interactive with auto-cleanup
docker run --rm -it python bash

# Named container
docker run --rm --name my_container -it python bash

# Detached (background) mode
docker run -d --name my_server nginx

# With port mapping
docker run -d -p 8080:80 nginx          # Host:Container
docker run -d -p 127.0.0.1:8080:80 nginx # Bind to localhost only
docker run -d -P nginx                    # Map all exposed ports

# With environment variables
docker run -e MY_VAR=value -e DEBUG=true myapp

# From env file
docker run --env-file ./env.list myapp

# With restart policy
docker run -d --restart unless-stopped nginx  # Restart on failure/reboot
docker run -d --restart always nginx          # Always restart
docker run -d --restart on-failure:3 nginx    # Restart max 3 times

# With resource limits
docker run -m 512m --cpus="1.5" myapp        # 512MB RAM, 1.5 CPUs
docker run --memory-swap 1g myapp            # Including swap

# With working directory
docker run -w /app myimage

# With user
docker run -u 1000:1000 myimage             # UID:GID
docker run -u $(id -u):$(id -g) myimage     # Current user
```

### Managing Running Containers

```bash
# List running containers
docker ps

# List all containers (including stopped)
docker ps -a

# List with custom format
docker ps --format "table {{.ID}}\t{{.Names}}\t{{.Status}}\t{{.Ports}}"

# List container IDs only
docker ps -q

# Execute command in running container
docker exec -it my_container bash
docker exec my_container ls /app
docker exec -u root my_container apt update    # As root

# Attach to running container
docker attach my_container
# Detach without stopping: Ctrl+P, Ctrl+Q

# View container logs
docker logs my_container
docker logs -f my_container            # Follow (tail)
docker logs --tail 100 my_container    # Last 100 lines
docker logs --since 1h my_container    # Last hour
docker logs -t my_container            # With timestamps

# Container resource usage
docker stats
docker stats my_container

# Container processes
docker top my_container
```

### Container Lifecycle

```bash
# Stop container (graceful, SIGTERM)
docker stop my_container
docker stop -t 30 my_container         # 30 sec timeout

# Kill container (force, SIGKILL)
docker kill my_container

# Restart container
docker restart my_container

# Pause/unpause container
docker pause my_container
docker unpause my_container

# Remove stopped container
docker rm my_container

# Force remove running container
docker rm -f my_container

# Remove all stopped containers
docker container prune

# Remove all containers (stopped and running)
docker rm -f $(docker ps -aq)
```

### Creating Images from Containers

```bash
# Commit container changes to new image
docker commit my_container myimage:modified

# Commit with message
docker commit -m "Added custom config" my_container myimage:v2

# Commit with author
docker commit -a "John Doe" my_container myimage:v2
```

### Copy Files

```bash
# Copy from container to host
docker cp my_container:/path/to/file ./local/path

# Copy from host to container
docker cp ./local/file my_container:/path/to/destination

# Copy entire directory
docker cp my_container:/app ./backup
```

---

## Volumes & Bind Mounts

### Types of Storage

1. **Volumes**: Managed by Docker, stored in `/var/lib/docker/volumes/`
2. **Bind Mounts**: Direct mapping to host filesystem
3. **tmpfs**: Stored in memory only (Linux)

### Bind Mounts

```bash
# Basic bind mount (host:container)
docker run -v /home/user/project:/app myimage

# Read-only bind mount
docker run -v /home/user/config:/config:ro myimage

# Mount current directory
docker run -v $(pwd):/app myimage
docker run -v "$PWD":/app myimage

# Multiple mounts
docker run \
  -v /home/user/code:/code \
  -v /home/user/data:/data \
  myimage

# Example: ROS2 workspace mount
docker run --rm -it \
  -v /home/george/Workspace/docker/work:/work \
  ros:humble-ros-base bash
```

### Named Volumes

```bash
# Create a volume
docker volume create mydata

# List volumes
docker volume ls

# Inspect volume
docker volume inspect mydata

# Use named volume
docker run -v mydata:/app/data myimage

# Remove volume
docker volume rm mydata

# Remove all unused volumes
docker volume prune
```

### Mount Syntax (--mount flag)

```bash
# Bind mount with --mount
docker run --mount type=bind,source=/host/path,target=/container/path myimage

# Volume with --mount
docker run --mount type=volume,source=mydata,target=/data myimage

# Read-only mount
docker run --mount type=bind,source=/host/path,target=/container/path,readonly myimage

# tmpfs mount
docker run --mount type=tmpfs,target=/tmp,tmpfs-size=100m myimage
```

---

## Networking

### Network Types

| Type | Description |
|------|-------------|
| `bridge` | Default. Containers on same bridge can communicate |
| `host` | Use host's network stack directly |
| `none` | No networking |
| `overlay` | Multi-host networking (Docker Swarm) |
| `macvlan` | Assign MAC address to container |

### Managing Networks

```bash
# List networks
docker network ls

# Create network
docker network create mynetwork

# Create with specific driver
docker network create --driver bridge mybridge

# Create with subnet
docker network create --subnet 172.20.0.0/16 mynetwork

# Inspect network
docker network inspect bridge

# Remove network
docker network rm mynetwork

# Prune unused networks
docker network prune
```

### Connecting Containers to Networks

```bash
# Run with specific network
docker run --network mynetwork myimage

# Use host networking (no isolation)
docker run --network host myimage

# Example: ROS2 with host networking (for DDS discovery)
docker run --rm --name my_ros --network host \
  -v /home/george/Workspace/docker/work:/work \
  -it ros:humble-ros-base bash

# Connect running container to network
docker network connect mynetwork my_container

# Disconnect from network
docker network disconnect mynetwork my_container

# Run with static IP
docker run --network mynetwork --ip 172.20.0.100 myimage
```

### Container Communication

```bash
# Containers on same network can communicate by name
# Container 1
docker run -d --name db --network mynetwork postgres

# Container 2 can reach db by hostname "db"
docker run -it --network mynetwork myapp ping db

# Legacy linking (deprecated, use networks instead)
docker run --link db:database myapp
```

---

## Dockerfile Reference

### Complete Dockerfile Example

```dockerfile
#===============================================================================
# Example Multi-Stage Dockerfile with Best Practices
#===============================================================================

# Build stage
FROM python:3.11-slim AS builder

# Set working directory
WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Copy and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir --user -r requirements.txt

# Production stage
FROM python:3.11-slim AS production

# Labels for metadata
LABEL maintainer="you@example.com"
LABEL version="1.0"
LABEL description="My Python Application"

# Create non-root user
RUN useradd -m -s /bin/bash appuser

# Set working directory
WORKDIR /app

# Copy dependencies from builder
COPY --from=builder /root/.local /home/appuser/.local

# Copy application code
COPY --chown=appuser:appuser . .

# Set environment variables
ENV PATH=/home/appuser/.local/bin:$PATH
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# Expose port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8000/health || exit 1

# Switch to non-root user
USER appuser

# Default command
CMD ["python", "app.py"]
```

### Dockerfile Instructions Reference

| Instruction | Description | Example |
|-------------|-------------|---------|
| `FROM` | Base image | `FROM python:3.11-slim` |
| `WORKDIR` | Set working directory | `WORKDIR /app` |
| `COPY` | Copy files | `COPY . .` |
| `ADD` | Copy files (supports URLs, tar extraction) | `ADD app.tar.gz /app` |
| `RUN` | Execute command | `RUN apt-get update` |
| `ENV` | Set environment variable | `ENV NODE_ENV=production` |
| `ARG` | Build-time variable | `ARG VERSION=1.0` |
| `EXPOSE` | Document port | `EXPOSE 8080` |
| `USER` | Set user | `USER appuser` |
| `CMD` | Default command | `CMD ["python", "app.py"]` |
| `ENTRYPOINT` | Container entrypoint | `ENTRYPOINT ["./entrypoint.sh"]` |
| `VOLUME` | Create mount point | `VOLUME /data` |
| `LABEL` | Add metadata | `LABEL version="1.0"` |
| `HEALTHCHECK` | Container health check | `HEALTHCHECK CMD curl localhost` |
| `SHELL` | Override default shell | `SHELL ["/bin/bash", "-c"]` |
| `STOPSIGNAL` | Set stop signal | `STOPSIGNAL SIGTERM` |

### Multi-Stage Build Tips

```dockerfile
# Name stages for clarity
FROM node:18 AS frontend-builder
RUN npm run build

FROM golang:1.21 AS backend-builder
RUN go build -o server

# Final minimal image
FROM alpine:3.18
COPY --from=frontend-builder /app/dist /static
COPY --from=backend-builder /app/server /server
CMD ["/server"]
```

---

## Docker Compose

### Docker Compose File Reference

```yaml
# docker-compose.yaml
version: "3.9"  # Optional in recent versions

services:
  # Web application service
  web:
    build:
      context: .
      dockerfile: Dockerfile
      args:
        BUILD_ENV: production
    image: myapp:latest
    container_name: myapp_web
    hostname: web
    restart: unless-stopped
    
    # Port mapping
    ports:
      - "8080:80"
      - "443:443"
    
    # Environment
    environment:
      - DATABASE_URL=postgres://db:5432/myapp
      - DEBUG=false
    env_file:
      - .env
    
    # Volumes
    volumes:
      - ./app:/app
      - data:/app/data
    
    # Dependencies
    depends_on:
      db:
        condition: service_healthy
      redis:
        condition: service_started
    
    # Networking
    networks:
      - frontend
      - backend
    
    # Resource limits
    deploy:
      resources:
        limits:
          cpus: '0.5'
          memory: 512M
        reservations:
          cpus: '0.25'
          memory: 256M
    
    # Health check
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s

  # Database service
  db:
    image: postgres:15
    container_name: myapp_db
    restart: unless-stopped
    environment:
      POSTGRES_DB: myapp
      POSTGRES_USER: user
      POSTGRES_PASSWORD: password
    volumes:
      - postgres_data:/var/lib/postgresql/data
    networks:
      - backend
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U user -d myapp"]
      interval: 10s
      timeout: 5s
      retries: 5

  # Redis cache
  redis:
    image: redis:alpine
    container_name: myapp_redis
    restart: unless-stopped
    networks:
      - backend

# Named volumes
volumes:
  data:
  postgres_data:

# Networks
networks:
  frontend:
  backend:
```

### Docker Compose Commands

```bash
# Start services
docker compose up                    # Foreground
docker compose up -d                 # Detached (background)
docker compose up --build            # Rebuild images first
docker compose up -d --force-recreate  # Recreate containers

# Specify compose file
docker compose -f docker-compose.yaml up -d
docker compose -f docker-compose.prod.yaml up -d

# Start specific service
docker compose up -d web

# Stop services
docker compose down                  # Stop and remove containers
docker compose down -v               # Also remove volumes
docker compose down --rmi all        # Also remove images

# Pause/unpause
docker compose pause
docker compose unpause

# View logs
docker compose logs
docker compose logs -f               # Follow
docker compose logs web              # Specific service

# List containers
docker compose ps

# Execute command in service
docker compose exec web bash
docker compose exec -u root web bash

# Run one-off command
docker compose run --rm web python manage.py migrate

# Scale service
docker compose up -d --scale web=3

# Pull images
docker compose pull

# Build images
docker compose build
docker compose build --no-cache

# View config (merged and validated)
docker compose config
```

### Docker Compose with GUI (X11)

```yaml
# docker-compose.yaml for GUI applications
services:
  gui-app:
    image: myguiapp:latest
    environment:
      - DISPLAY=${DISPLAY:-:0}
      - XDG_RUNTIME_DIR=/tmp
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ~/.Xauthority:/root/.Xauthority:ro
    network_mode: host
    devices:
      - /dev/dri:/dev/dri  # GPU acceleration
```

```bash
# Allow X11 access before running
xhost +local:docker

# Check your display settings
echo $XDG_SESSION_TYPE   # x11 or wayland
echo $DISPLAY            # :0 or :1
```

---

## Docker Registry

### Docker Hub

```bash
# Login to Docker Hub
docker login
docker login -u username

# Push image to Docker Hub
docker tag myimage:latest username/myimage:latest
docker push username/myimage:latest

# Pull from Docker Hub
docker pull username/myimage:latest

# Logout
docker logout
```

### Private Registry (Self-Hosted)

```bash
# Start a local registry
docker run -d -p 5000:5000 --name registry registry:2

# With persistent storage (recommended)
docker run -d -p 5000:5000 --name registry \
  -v registry_data:/var/lib/registry \
  registry:2

# Check registry catalog
curl http://localhost:5000/v2/_catalog

# Tag and push to local registry
docker tag myimage:latest localhost:5000/myimage:latest
docker push localhost:5000/myimage:latest

# Pull from local registry
docker pull localhost:5000/myimage:latest
```

### Accessing Private Registry from Another Machine

```bash
# On the client machine, configure insecure registry
sudo vim /etc/docker/daemon.json
```

```json
{
  "insecure-registries": ["192.168.1.101:5000"]
}
```

```bash
# Restart Docker
sudo systemctl restart docker
# or
sudo service docker restart

# Now pull from remote registry
curl http://192.168.1.101:5000/v2/_catalog
docker pull 192.168.1.101:5000/myimage:latest
```

### Registry with Authentication

```bash
# Create password file
mkdir auth
docker run --entrypoint htpasswd registry:2 -Bbn user password > auth/htpasswd

# Start registry with auth
docker run -d -p 5000:5000 --name registry \
  -v registry_data:/var/lib/registry \
  -v $(pwd)/auth:/auth \
  -e REGISTRY_AUTH=htpasswd \
  -e REGISTRY_AUTH_HTPASSWD_REALM="Registry Realm" \
  -e REGISTRY_AUTH_HTPASSWD_PATH=/auth/htpasswd \
  registry:2

# Login to private registry
docker login localhost:5000
```

---

## GPU Support (NVIDIA)

### Install NVIDIA Container Toolkit

```bash
# Add NVIDIA repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker

# Verify configuration
cat /etc/docker/daemon.json

# Restart Docker
sudo systemctl restart docker
```

### Using GPU in Containers

```bash
# Check CUDA version on host
nvidia-smi

# Run with GPU access
docker run --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi

# Specific GPU
docker run --gpus '"device=0"' nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
docker run --gpus '"device=0,1"' nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi

# GPU in docker-compose
# docker-compose.yaml
services:
  gpu-app:
    image: nvidia/cuda:12.2.0-base-ubuntu22.04
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```

### Custom GPU Dockerfile

```dockerfile
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04

# Install Python and dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch with CUDA support
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

WORKDIR /app
CMD ["python3"]
```

---

## System Management

### Disk Usage

```bash
# Show Docker disk usage
docker system df
docker system df -v          # Verbose

# Prune unused data
docker system prune          # Containers, networks, dangling images
docker system prune -a       # Also unused images
docker system prune -a --volumes  # Also volumes

# Prune with filter
docker system prune --filter "until=24h"
```

### Individual Prune Commands

```bash
# Remove stopped containers
docker container prune

# Remove unused networks
docker network prune

# Remove dangling images
docker image prune

# Remove all unused images
docker image prune -a

# Remove unused volumes
docker volume prune
```

### Docker Daemon

```bash
# Restart Docker service
sudo systemctl restart docker
sudo service docker restart

# Start/stop Docker
sudo systemctl start docker
sudo systemctl stop docker

# Enable at boot
sudo systemctl enable docker

# View Docker logs
sudo journalctl -u docker
sudo journalctl -u docker -f   # Follow
```

### Docker Configuration

```bash
# Docker daemon config location
/etc/docker/daemon.json

# Example daemon.json
{
  "storage-driver": "overlay2",
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "insecure-registries": [],
  "dns": ["8.8.8.8", "8.8.4.4"]
}

# After changing, restart Docker
sudo systemctl restart docker
```

---

## Debugging & Troubleshooting

### Container Debugging

```bash
# View container logs
docker logs container_name
docker logs -f container_name    # Follow
docker logs --tail 50 container_name

# Inspect container
docker inspect container_name
docker inspect -f '{{.State.Status}}' container_name
docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' container_name

# View processes in container
docker top container_name

# Resource usage
docker stats container_name

# Enter container as root (even if USER specified)
docker exec -u root -it container_name bash

# View file system changes
docker diff container_name

# Export container filesystem for analysis
docker export container_name > container_fs.tar
```

### Image Debugging

```bash
# Inspect image
docker inspect image_name

# View image history/layers
docker history image_name
docker history --no-trunc image_name

# Analyze image layers (requires dive tool)
# Install: https://github.com/wagoodman/dive
dive image_name
```

### Network Debugging

```bash
# Inspect network
docker network inspect bridge

# Test connectivity from container
docker exec container_name ping google.com
docker exec container_name curl http://other_container:8080

# View container ports
docker port container_name

# Check DNS resolution
docker exec container_name nslookup other_container
```

### Common Issues & Solutions

```bash
# Permission denied on mounted volume
# Solution: Match UID/GID or use :z/:Z for SELinux
docker run -u $(id -u):$(id -g) -v /host:/container myimage
docker run -v /host:/container:z myimage  # SELinux

# Container immediately exits
# Solution: Keep it running with tail or sleep
docker run -d myimage tail -f /dev/null
docker run -d myimage sleep infinity

# Can't connect to Docker daemon
# Solution: Check if Docker is running
sudo systemctl status docker
sudo systemctl start docker

# Out of disk space
# Solution: Prune unused data
docker system prune -a --volumes

# DNS issues inside container
# Solution: Set custom DNS
docker run --dns 8.8.8.8 myimage
# Or configure in daemon.json
```

---

## Best Practices

### Dockerfile Best Practices

```dockerfile
# 1. Use specific base image tags (not latest)
FROM python:3.11-slim  # Good
# FROM python:latest   # Avoid

# 2. Use multi-stage builds to reduce image size
FROM node:18 AS builder
RUN npm run build

FROM nginx:alpine
COPY --from=builder /app/dist /usr/share/nginx/html

# 3. Combine RUN commands to reduce layers
RUN apt-get update && apt-get install -y \
    package1 \
    package2 \
    && rm -rf /var/lib/apt/lists/*

# 4. Order instructions from least to most frequently changing
COPY package.json .
RUN npm install
COPY . .  # Source code changes frequently

# 5. Use .dockerignore
# Create .dockerignore file:
# .git
# node_modules
# *.log
# .env

# 6. Run as non-root user
RUN useradd -m appuser
USER appuser

# 7. Use COPY instead of ADD (unless you need tar extraction)
COPY . .

# 8. Set proper HEALTHCHECK
HEALTHCHECK --interval=30s CMD curl -f http://localhost/ || exit 1
```

### Security Best Practices

```bash
# 1. Scan images for vulnerabilities
docker scout cves myimage:latest
# Or use Trivy
trivy image myimage:latest

# 2. Run as non-root
docker run -u 1000:1000 myimage

# 3. Use read-only filesystem where possible
docker run --read-only myimage

# 4. Limit capabilities
docker run --cap-drop ALL --cap-add NET_BIND_SERVICE myimage

# 5. Use secrets for sensitive data (not env vars for passwords)
# docker-compose.yaml
services:
  app:
    secrets:
      - db_password
secrets:
  db_password:
    file: ./secrets/db_password.txt

# 6. Keep images updated
docker pull myimage:latest

# 7. Sign images
docker trust sign myimage:latest
```

### Performance Best Practices

```bash
# 1. Use appropriate restart policies
docker run --restart unless-stopped myimage

# 2. Set resource limits
docker run -m 512m --cpus="1.0" myimage

# 3. Use health checks
docker run --health-cmd="curl -f localhost" myimage

# 4. Use build cache effectively
docker build --cache-from myimage:latest .

# 5. Clean up in single RUN statement
RUN apt-get update && apt-get install -y pkg \
    && rm -rf /var/lib/apt/lists/*
```

---

## ROS2 Specific Examples

### Running ROS2 Container with GUI

```bash
# Allow X11 access
xhost +local:docker

# Run with display support and networking
docker run --rm -it \
  --name ros2_dev \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e XDG_RUNTIME_DIR=/tmp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/.Xauthority:/root/.Xauthority:ro \
  -v /home/george/Workspace/docker/work:/work \
  ros:humble-desktop bash

# Inside container, run RViz
source /opt/ros/humble/setup.bash
rviz2
```

### ROS2 Multi-Robot Setup

```yaml
# docker-compose.yaml
services:
  robot1:
    image: ros:humble-ros-base
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=1
    command: ros2 run demo_nodes_cpp talker

  robot2:
    image: ros:humble-ros-base
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=1
    command: ros2 run demo_nodes_cpp listener
```

### Building ROS2 Workspace in Container

```bash
# Run container with workspace mounted
docker run --rm -it \
  -v /path/to/ros2_ws:/ros2_ws \
  ros:humble-ros-base bash

# Inside container
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Reference

### Most Used Commands

```bash
# Images
docker pull image:tag
docker build -t name:tag .
docker images
docker rmi image

# Containers
docker run -it --rm image bash
docker ps -a
docker exec -it container bash
docker logs -f container
docker stop/start/restart container
docker rm container

# Volumes
docker volume create/ls/rm
docker run -v host:container

# Networks
docker network create/ls/rm
docker run --network name

# Cleanup
docker system prune -a --volumes

# Compose
docker compose up -d
docker compose down
docker compose logs -f
```

### Environment Variables

| Variable | Description |
|----------|-------------|
| `DOCKER_HOST` | Docker daemon socket |
| `DOCKER_TLS_VERIFY` | Enable TLS verification |
| `DOCKER_CERT_PATH` | Path to TLS certs |
| `DOCKER_BUILDKIT` | Enable BuildKit |
| `COMPOSE_FILE` | Compose file path |
| `COMPOSE_PROJECT_NAME` | Project name prefix |

---

## Additional Resources

- [Docker Official Documentation](https://docs.docker.com/)
- [Docker Hub](https://hub.docker.com/)
- [Dockerfile Reference](https://docs.docker.com/engine/reference/builder/)
- [Docker Compose Reference](https://docs.docker.com/compose/compose-file/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
- [ROS2 Docker Images](https://hub.docker.com/_/ros)

---

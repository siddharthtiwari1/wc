# Docker Cheat Sheet for Wheelchair ROS Development
Quick reference for daily Docker commands

---

## Container Lifecycle

```bash
# Pull image
docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0

# Run container (interactive, auto-remove)
docker run -it --rm IMAGE_NAME

# Run with name
docker run -it --name my_container IMAGE_NAME

# Run in background
docker run -d IMAGE_NAME

# Stop container
docker stop CONTAINER_NAME

# Start stopped container
docker start CONTAINER_NAME

# Remove container
docker rm CONTAINER_NAME

# Remove all stopped containers
docker container prune
```

---

## Common Run Configurations

### Basic Development

```bash
docker run -it --rm \
  --name wheelchair_dev \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash
```

### With GUI Support (Gazebo, RViz)

```bash
xhost +local:docker  # Run first!

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair_2_base_gazebo:v3.0 \
  /bin/bash
```

### With Hardware Access (USB, Cameras)

```bash
docker run -it --rm \
  --privileged \
  -v /dev:/dev \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash
```

### With GPU (CUDA, ML)

```bash
docker run -it --rm \
  --gpus all \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/humble_gpu:v3.0 \
  /bin/bash
```

### Complete (All Features)

```bash
xhost +local:docker

docker run -it --rm \
  --name wheelchair_dev \
  --network host \
  --privileged \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair_2_base_gazebo:v3.0 \
  /bin/bash
```

---

## Working with Running Containers

```bash
# List running containers
docker ps

# List all containers
docker ps -a

# Execute command in running container
docker exec CONTAINER_NAME ls /workspace

# Open terminal in running container
docker exec -it CONTAINER_NAME /bin/bash

# View container logs
docker logs CONTAINER_NAME

# Follow logs (live)
docker logs -f CONTAINER_NAME

# Copy file from container to host
docker cp CONTAINER_NAME:/path/in/container /path/on/host

# Copy file from host to container
docker cp /path/on/host CONTAINER_NAME:/path/in/container

# View container resource usage
docker stats

# Inspect container details
docker inspect CONTAINER_NAME
```

---

## Images

```bash
# List images
docker images

# Pull image
docker pull IMAGE_NAME:TAG

# Remove image
docker rmi IMAGE_NAME

# Remove all unused images
docker image prune

# Remove all images (CAREFUL!)
docker image prune -a

# View image history
docker history IMAGE_NAME

# Tag an image
docker tag OLD_NAME:TAG NEW_NAME:TAG
```

---

## Cleanup

```bash
# Remove stopped containers
docker container prune

# Remove unused images
docker image prune

# Remove unused volumes
docker volume prune

# Remove everything unused (CAREFUL!)
docker system prune -a

# Check disk usage
docker system df
```

---

## Wheelchair Project Images

| Image | Purpose | Size | Use Case |
|-------|---------|------|----------|
| `humble:v3.0` | ROS2 Humble base | ~2GB | Basic ROS development |
| `humble_gpu:v3.0` | With CUDA | ~4GB | AI/ML workloads |
| `humble_harmonic:v3.0` | With Gazebo | ~5GB | Simulation |
| `wheelchair2_base:v3.0` | Hardware drivers | ~3GB | Real wheelchair |
| `wheelchair_2_base_gazebo:v3.0` | Complete dev | ~6GB | Full development |
| `wheelchair2_base_jetson:v3.0` | ARM64 Jetson | ~3GB | Onboard computer |

### Pull Commands

```bash
# Base
docker pull ghcr.io/smart-wheelchair-rrc/humble:v3.0

# Hardware
docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0

# Simulation
docker pull ghcr.io/smart-wheelchair-rrc/wheelchair_2_base_gazebo:v3.0

# Jetson
docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_base_jetson:v3.0
```

---

## Common Flags

| Flag | Purpose | Example |
|------|---------|---------|
| `-it` | Interactive terminal | `docker run -it ubuntu` |
| `--rm` | Auto-remove on exit | `docker run --rm ubuntu` |
| `-d` | Detached (background) | `docker run -d nginx` |
| `--name` | Give container a name | `docker run --name myapp ubuntu` |
| `-v` | Mount volume | `docker run -v /host:/container ubuntu` |
| `-e` | Set environment variable | `docker run -e VAR=value ubuntu` |
| `-p` | Port mapping | `docker run -p 8080:80 nginx` |
| `--network` | Network mode | `docker run --network host ubuntu` |
| `--privileged` | Full device access | `docker run --privileged ubuntu` |
| `--gpus` | GPU access | `docker run --gpus all nvidia/cuda` |
| `--user` | Run as specific user | `docker run --user 1000:1000 ubuntu` |

---

## ROS2 Workflow Inside Container

```bash
# 1. Enter container
docker exec -it wheelchair_dev /bin/bash

# 2. Navigate to workspace
cd /workspace

# 3. Build
colcon build --symlink-install

# 4. Source
source install/setup.bash

# 5. Run
ros2 launch my_package my_launch.py

# 6. Debug (in another terminal)
docker exec -it wheelchair_dev /bin/bash
ros2 topic echo /some_topic
ros2 node list
ros2 topic list
```

---

## Troubleshooting Commands

```bash
# Check Docker is running
sudo systemctl status docker

# Start Docker
sudo systemctl start docker

# Check you're in docker group
groups | grep docker

# View container logs for errors
docker logs CONTAINER_NAME

# Inspect container details
docker inspect CONTAINER_NAME

# Check container processes
docker top CONTAINER_NAME

# Enter container as root (for debugging)
docker exec -it --user root CONTAINER_NAME /bin/bash

# Check disk space
docker system df

# Find container IP address
docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' CONTAINER_NAME
```

---

## GitHub Container Registry

```bash
# Login
echo $GITHUB_TOKEN | docker login ghcr.io -u USERNAME --password-stdin

# Logout
docker logout ghcr.io

# Pull private image
docker pull ghcr.io/smart-wheelchair-rrc/IMAGE:TAG
```

---

## Quick Scripts

### start_dev.sh

```bash
#!/bin/bash
docker run -it --rm \
  --name wheelchair_dev \
  --network host \
  -v ~/wheelchair_ws:/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash
```

### start_gazebo.sh

```bash
#!/bin/bash
xhost +local:docker
docker run -it --rm \
  --name wheelchair_gazebo \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair_2_base_gazebo:v3.0 \
  /bin/bash
```

### attach.sh

```bash
#!/bin/bash
docker exec -it wheelchair_dev /bin/bash
```

### cleanup.sh

```bash
#!/bin/bash
docker stop $(docker ps -aq)
docker container prune -f
docker image prune -f
docker system prune -f
```

---

## Emergency Commands

```bash
# Kill all running containers
docker kill $(docker ps -q)

# Remove all containers
docker rm $(docker ps -aq)

# Remove all images
docker rmi $(docker images -q)

# Nuclear option (removes EVERYTHING)
docker system prune -a --volumes -f
```

---

## Useful Aliases (Add to ~/.bashrc)

```bash
# Docker aliases
alias dps='docker ps'
alias dpsa='docker ps -a'
alias di='docker images'
alias drm='docker rm'
alias drmi='docker rmi'
alias dex='docker exec -it'
alias dlog='docker logs'
alias dprune='docker system prune -f'

# Wheelchair specific
alias wheelchair-dev='docker run -it --rm --name wheelchair_dev -v ~/wheelchair_ws:/workspace ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 /bin/bash'
alias wheelchair-attach='docker exec -it wheelchair_dev /bin/bash'
```

After adding, run: `source ~/.bashrc`

---

## Environment Variables

Useful variables to set inside containers:

```bash
# ROS2
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RCUTILS_COLORIZED_OUTPUT=1

# Display
export DISPLAY=:0
export QT_X11_NO_MITSHM=1

# Build
export MAKEFLAGS="-j$(nproc)"
```

---

## Network Debugging

```bash
# Check container's IP
docker inspect wheelchair_dev | grep IPAddress

# Test connectivity from host to container
ping CONTAINER_IP

# Check open ports
docker port CONTAINER_NAME

# Use host network (simplest for ROS)
docker run --network host ...
```

---

**Save this file for quick reference!**

Print it out or keep it open in a terminal window while working.

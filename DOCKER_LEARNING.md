# Docker Learning Guide for Beginners
## From Zero to Running ROS Containers

**Created**: 2025-11-22
**Purpose**: Learn Docker basics for wheelchair robotics development
**Target Repository**: [Smart-Wheelchair-RRC/DockerForDevelopment](https://github.com/Smart-Wheelchair-RRC/DockerForDevelopment)

---

## Part 1: What is Docker? (The Absolute Basics)

### The Problem Docker Solves

**Scenario**: You want to run a robotics application that needs:
- Ubuntu 22.04
- ROS2 Humble
- Realsense SDK
- Gazebo simulator
- Specific library versions

**Without Docker**: You'd need to:
- Install everything on your machine
- Risk breaking your system
- Can't easily share your setup with teammates
- "Works on my machine" syndrome

**With Docker**: You get a **container** - think of it as a lightweight, isolated mini-computer that:
- Runs inside your computer
- Has its own filesystem and programs
- Can be easily shared and reproduced
- Won't mess up your main system

### Core Concepts (Simple Definitions)

| Term | What It Is | Real-World Analogy |
|------|------------|-------------------|
| **Image** | A blueprint/template for a container | Recipe for a dish |
| **Container** | A running instance of an image | The actual cooked dish |
| **Dockerfile** | Instructions to build an image | Recipe card with steps |
| **Registry** | Storage for images (like GitHub for Docker) | Recipe book library |
| **Volume** | Shared folder between host and container | Shared USB drive |

### Visual: How Docker Works

```
Your Computer (Host)
├── Operating System
└── Docker Engine
    ├── Container 1 (Ubuntu + ROS2)
    │   ├── Your robot code
    │   └── All dependencies
    ├── Container 2 (Testing environment)
    └── Container 3 (Gazebo simulator)
```

---

## Part 2: Essential Docker Commands

### Installation Check

```bash
# Check if Docker is installed
docker --version

# Test Docker is working
docker run hello-world
```

### Working with Images

```bash
# Pull (download) an image from a registry
docker pull ubuntu:22.04

# List images on your computer
docker images

# Remove an image
docker rmi <image-name>
```

### Working with Containers

```bash
# Run a container (creates and starts it)
docker run ubuntu:22.04

# Run a container with interactive terminal
docker run -it ubuntu:22.04 /bin/bash

# List running containers
docker ps

# List all containers (including stopped)
docker ps -a

# Stop a container
docker stop <container-id>

# Remove a container
docker rm <container-id>

# Start an existing container
docker start <container-id>

# Execute command in running container
docker exec -it <container-id> /bin/bash
```

### Key Flags to Remember

| Flag | Purpose | Example |
|------|---------|---------|
| `-it` | Interactive terminal | `docker run -it ubuntu bash` |
| `-d` | Run in background (detached) | `docker run -d nginx` |
| `-p` | Port mapping (host:container) | `docker run -p 8080:80 nginx` |
| `-v` | Volume mounting | `docker run -v /host/path:/container/path` |
| `--name` | Give container a name | `docker run --name myapp ubuntu` |
| `--rm` | Auto-remove when stopped | `docker run --rm ubuntu` |

---

## Part 3: Understanding the Wheelchair ROS Repository

### Why This Repository Exists

Your teammates created **pre-built Docker images** so everyone has:
- ✅ Identical development environments
- ✅ All ROS2 and wheelchair dependencies installed
- ✅ No "setup hell" on each machine
- ✅ Works on x86_64 (laptops) and ARM64 (Jetson boards)

### Available Images (Simplified)

```
1. humble (Base)
   └── Ubuntu 22.04 + ROS2 Humble
       └── For basic ROS development

2. humble_gpu (GPU Support)
   └── Base + NVIDIA CUDA
       └── For AI/ML workloads

3. humble_harmonic (Simulation)
   └── GPU + Gazebo Harmonic
       └── For testing in simulator

4. wheelchair2_base (Hardware)
   └── humble + Realsense + Livox LiDAR drivers
       └── For real wheelchair hardware

5. wheelchair_2_base_gazebo (Full Dev)
   └── Everything above combined
       └── Complete development environment

6. humble_jetson (Embedded)
   └── ARM64 for Jetson boards
       └── For onboard wheelchair computer
```

### Image Dependency Chain

```
humble (base image)
├── wheelchair2_base
└── humble_gpu
    └── humble_harmonic
        └── wheelchair_2_base_gazebo
```

---

## Part 4: Hands-On Practice

### Exercise 1: Pull and Run a Basic ROS Image

**Goal**: Get the humble image and run a container

```bash
# Step 1: Authenticate to GitHub Container Registry
# (You'll need a GitHub Personal Access Token)
echo $GITHUB_TOKEN | docker login ghcr.io -u YOUR_GITHUB_USERNAME --password-stdin

# Step 2: Pull the image (specific version, not "latest")
docker pull ghcr.io/smart-wheelchair-rrc/humble:v3.0

# Step 3: Run it interactively
docker run -it --rm ghcr.io/smart-wheelchair-rrc/humble:v3.0 /bin/bash

# Step 4: Inside the container, test ROS2
source /opt/ros/humble/setup.bash
ros2 topic list

# Step 5: Exit (Ctrl+D or type 'exit')
exit
```

### Exercise 2: Mount Your Code into Container

**Goal**: Work on code on your computer but run it inside the container

```bash
# Create a workspace on your host
mkdir -p ~/wheelchair_ws/src
cd ~/wheelchair_ws

# Run container with volume mount
docker run -it --rm \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/humble:v3.0 \
  /bin/bash

# Inside container, your files are at /workspace
cd /workspace
ls -la
```

### Exercise 3: Run with GUI Support (for Gazebo)

**Goal**: Display GUI applications from container

```bash
# Allow X11 forwarding
xhost +local:docker

# Run with display forwarding
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/humble_harmonic:v3.0 \
  /bin/bash

# Inside container, you can now run GUI apps
gazebo
```

### Exercise 4: GPU Access (for CUDA/ML)

```bash
# Run with GPU support (requires nvidia-docker)
docker run -it --rm \
  --gpus all \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/humble_gpu:v3.0 \
  /bin/bash

# Test GPU is accessible
nvidia-smi
```

---

## Part 5: Common Workflows

### Daily Development Workflow

```bash
# 1. Start your development container
docker run -it --rm \
  --name wheelchair_dev \
  -v ~/wheelchair_ws:/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash

# 2. Inside container, build your code
cd /workspace
colcon build

# 3. Source and run
source install/setup.bash
ros2 launch your_package your_launch.py

# 4. Open another terminal to the same container
docker exec -it wheelchair_dev /bin/bash
```

### Testing in Gazebo Simulation

```bash
# Use the full gazebo image
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair_2_base_gazebo:v3.0 \
  /bin/bash

# Launch your simulation
ros2 launch wheelchair_gazebo simulation.launch.py
```

### Deploying to Jetson (Wheelchair Computer)

```bash
# On Jetson board, pull ARM64 image
docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_base_jetson:v3.0

# Run with hardware access
docker run -it --rm \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base_jetson:v3.0 \
  /bin/bash
```

---

## Part 6: Understanding Dockerfiles

### What's in a Dockerfile?

A Dockerfile in the repository looks like this:

```dockerfile
# Start from a base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create a user (not root)
RUN useradd -m -s /bin/bash rosuser

# Set working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
```

### Common Dockerfile Commands

| Command | Purpose | Example |
|---------|---------|---------|
| `FROM` | Base image to start from | `FROM ubuntu:22.04` |
| `RUN` | Execute command during build | `RUN apt-get install ros` |
| `COPY` | Copy files from host to image | `COPY myfile.txt /app/` |
| `ENV` | Set environment variable | `ENV ROS_DISTRO=humble` |
| `WORKDIR` | Set working directory | `WORKDIR /workspace` |
| `USER` | Switch to user | `USER rosuser` |
| `CMD` | Default command when container starts | `CMD ["/bin/bash"]` |
| `ENTRYPOINT` | Command that always runs | `ENTRYPOINT ["/ros_entrypoint.sh"]` |

---

## Part 7: Troubleshooting Common Issues

### Issue 1: Permission Denied

```bash
# Problem: Can't access files in mounted volume
# Solution: Run as your user ID
docker run -it --rm \
  --user $(id -u):$(id -g) \
  -v ~/wheelchair_ws:/workspace \
  IMAGE_NAME
```

### Issue 2: Can't Connect to Docker Daemon

```bash
# Check Docker is running
sudo systemctl status docker

# Start Docker
sudo systemctl start docker

# Add yourself to docker group (no sudo needed)
sudo usermod -aG docker $USER
# Then log out and back in
```

### Issue 3: Display Not Working (GUI)

```bash
# Allow X11 connections
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Run with proper display settings
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  IMAGE_NAME
```

### Issue 4: Out of Disk Space

```bash
# Remove unused containers
docker container prune

# Remove unused images
docker image prune

# Remove everything unused (CAREFUL!)
docker system prune -a
```

---

## Part 8: Quick Reference Card

### Most Used Commands

```bash
# Pull an image
docker pull ghcr.io/smart-wheelchair-rrc/humble:v3.0

# Run interactive container
docker run -it --rm IMAGE_NAME

# List running containers
docker ps

# Stop container
docker stop CONTAINER_ID

# Execute command in running container
docker exec -it CONTAINER_NAME /bin/bash

# Remove all stopped containers
docker container prune

# View container logs
docker logs CONTAINER_ID
```

### Typical Development Command

```bash
docker run -it --rm \
  --name my_dev_container \
  -v ~/wheelchair_ws:/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash
```

---

## Part 9: Next Steps

### Learning Path

1. ✅ **Day 1-2**: Understand concepts, run basic containers
2. ✅ **Day 3-4**: Mount volumes, work with your code inside containers
3. 📅 **Day 5-6**: Build and modify Dockerfiles
4. 📅 **Week 2**: Use docker-compose for multi-container setups
5. 📅 **Week 3+**: CI/CD integration, advanced networking

### Resources

- [Official Docker Tutorial](https://docs.docker.com/get-started/)
- [ROS2 Docker Tutorial](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### Practice Challenges

1. **Challenge 1**: Pull the humble image and list all available ROS2 packages
2. **Challenge 2**: Create a simple ROS2 node on your host, mount it into a container, and run it
3. **Challenge 3**: Launch a Gazebo simulation from a container
4. **Challenge 4**: Connect two containers together (ROS2 discovery across containers)

---

## Part 10: Getting Help

### When You're Stuck

1. **Read error messages carefully** - Docker errors are usually descriptive
2. **Check logs**: `docker logs CONTAINER_ID`
3. **Google the error** - Docker has excellent community support
4. **Ask your team** - They built these images for you!

### Useful Debugging Commands

```bash
# Inspect image details
docker inspect IMAGE_NAME

# See image build history
docker history IMAGE_NAME

# Monitor container resource usage
docker stats

# View container processes
docker top CONTAINER_ID
```

---

## Summary: The 5 Things You MUST Know

1. **Images are templates, containers are running instances**
2. **Use `-v` to share files between host and container**
3. **Always specify version tags** (not "latest") for reproducibility
4. **Use `--rm` flag to auto-cleanup test containers**
5. **Your team's images have everything pre-installed** - just pull and run!

---

**Remember**: Docker is just a tool to package and run software consistently. Don't let it intimidate you. Start simple, experiment, and you'll catch up quickly!

**First Command to Run Right Now**:
```bash
docker run -it --rm ubuntu:22.04 /bin/bash
```

Play around inside that container, then exit. That's Docker! 🚀

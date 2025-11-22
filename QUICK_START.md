# Docker Quick Start for Wheelchair ROS Development
## Get Running in 10 Minutes

**Target**: Complete beginners who need to catch up FAST
**Goal**: Get you running ROS2 containers for wheelchair development TODAY

---

## Step 1: Install Docker (5 minutes)

### On Ubuntu/Debian Linux:

```bash
# Update package list
sudo apt-get update

# Install Docker
sudo apt-get install -y docker.io

# Start Docker service
sudo systemctl start docker
sudo systemctl enable docker

# Add yourself to docker group (no more sudo!)
sudo usermod -aG docker $USER

# IMPORTANT: Log out and log back in for group changes to take effect
```

### Verify Installation:

```bash
docker --version
# Should output: Docker version XX.X.X

docker run hello-world
# Should download and run a test container
```

---

## Step 2: Authenticate to GitHub Container Registry (2 minutes)

Your team's Docker images are private on GitHub. You need to log in.

### Create GitHub Personal Access Token:

1. Go to: https://github.com/settings/tokens
2. Click "Generate new token (classic)"
3. Give it a name: "Docker Access"
4. Check these scopes:
   - ✅ `read:packages`
   - ✅ `write:packages`
   - ✅ `repo`
5. Click "Generate token"
6. **COPY THE TOKEN** (you'll never see it again!)

### Login to Registry:

```bash
# Replace YOUR_USERNAME and YOUR_TOKEN
echo YOUR_TOKEN | docker login ghcr.io -u YOUR_USERNAME --password-stdin

# Should see: Login Succeeded
```

---

## Step 3: Pull Your First Image (1 minute)

```bash
# Pull the base ROS2 Humble image
docker pull ghcr.io/smart-wheelchair-rrc/humble:v3.0

# This downloads ~2GB, wait for it to complete
```

---

## Step 4: Run Your First Container (30 seconds)

```bash
# Run it!
docker run -it --rm ghcr.io/smart-wheelchair-rrc/humble:v3.0 /bin/bash

# You're now INSIDE the container!
# Your prompt will change to something like: rosuser@abc123:/$
```

### Test ROS2 is Working:

```bash
# Inside the container, run:
source /opt/ros/humble/setup.bash
ros2 topic list

# You should see ROS2 topics!
```

### Exit the Container:

```bash
exit
# Or press Ctrl+D
```

**🎉 CONGRATULATIONS!** You just ran your first Docker container with ROS2!

---

## Step 5: Create Your Workspace (1 minute)

```bash
# On your host machine (not in container)
mkdir -p ~/wheelchair_ws/src
cd ~/wheelchair_ws
```

This is where you'll put your ROS2 packages.

---

## Step 6: Run Container with Your Workspace Mounted (30 seconds)

```bash
docker run -it --rm \
  --name wheelchair_dev \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash

# Now your ~/wheelchair_ws folder is accessible inside the container at /workspace
```

### Verify:

```bash
# Inside container:
cd /workspace
ls -la
# You should see 'src' folder

# Create a test file
touch test.txt

# Exit container
exit

# On host:
ls ~/wheelchair_ws
# You should see test.txt!
```

**KEY CONCEPT**: Files you create on your host appear in the container and vice versa!

---

## Step 7: Understanding Which Image to Use

| Task | Image to Use | Command |
|------|-------------|---------|
| **Basic ROS2 development** | `humble:v3.0` | `docker pull ghcr.io/smart-wheelchair-rrc/humble:v3.0` |
| **Real wheelchair hardware** | `wheelchair2_base:v3.0` | `docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0` |
| **Gazebo simulation** | `wheelchair_2_base_gazebo:v3.0` | `docker pull ghcr.io/smart-wheelchair-rrc/wheelchair_2_base_gazebo:v3.0` |
| **Jetson boards** | `wheelchair2_base_jetson:v3.0` | `docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_base_jetson:v3.0` |
| **AI/ML with GPU** | `humble_gpu:v3.0` | `docker pull ghcr.io/smart-wheelchair-rrc/humble_gpu:v3.0` |

---

## Step 8: Your Daily Development Command

Save this command - you'll use it every day:

```bash
#!/bin/bash
# File: start_dev.sh

docker run -it --rm \
  --name wheelchair_dev \
  --network host \
  -v ~/wheelchair_ws:/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash
```

### Make it executable:

```bash
# Create the script
nano start_dev.sh
# Paste the command above, save (Ctrl+X, Y, Enter)

chmod +x start_dev.sh

# Now you can start development with:
./start_dev.sh
```

---

## Step 9: Open Multiple Terminals to Same Container

You often need multiple terminals (one for launch, one for debugging, etc.)

### Terminal 1: Start container

```bash
docker run -it \
  --name wheelchair_dev \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash
```

### Terminal 2: Attach to running container

```bash
docker exec -it wheelchair_dev /bin/bash
```

### Terminal 3, 4, 5...: Same command

```bash
docker exec -it wheelchair_dev /bin/bash
```

---

## Step 10: Clean Up When Done

```bash
# Stop the container (from another terminal)
docker stop wheelchair_dev

# Or use --rm flag when starting (auto-cleanup on exit)
docker run -it --rm ...

# Remove all stopped containers
docker container prune

# Remove unused images (frees disk space)
docker image prune
```

---

## Common Tasks Reference

### Building ROS2 Code in Container

```bash
# Inside container:
cd /workspace
colcon build --symlink-install

source install/setup.bash

ros2 launch my_package my_launch.py
```

### Running Gazebo Simulation

```bash
# Allow GUI applications
xhost +local:docker

# Start container with GUI support
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair_2_base_gazebo:v3.0 \
  /bin/bash

# Inside container:
gazebo
```

### Accessing Real Hardware (USB devices)

```bash
# Run with privileged mode (gives access to all devices)
docker run -it --rm \
  --privileged \
  -v /dev:/dev \
  -v ~/wheelchair_ws:/workspace \
  ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0 \
  /bin/bash

# Inside container, you can now access /dev/ttyUSB0, cameras, etc.
```

---

## Troubleshooting

### "Permission denied" when running docker

```bash
# Did you log out and back in after adding yourself to docker group?
# Check if you're in the group:
groups | grep docker

# If not, run again and RESTART your computer:
sudo usermod -aG docker $USER
```

### "Cannot connect to X server" for GUI

```bash
# Run this before starting container:
xhost +local:docker
```

### Container can't access internet

```bash
# Use --network host flag:
docker run -it --rm --network host IMAGE_NAME
```

### Out of disk space

```bash
# Clean up unused Docker data:
docker system prune -a

# Warning: This removes all stopped containers and unused images!
```

---

## What to Learn Next

Now that you can run containers, learn:

1. ✅ **Week 1**: Get comfortable with these commands
2. 📚 **Week 2**: Read DOCKER_LEARNING.md for deeper understanding
3. 🏗️ **Week 3**: Learn to modify Dockerfiles for custom needs
4. 🚀 **Week 4**: Set up docker-compose for multi-container systems

---

## Emergency Cheat Sheet

```bash
# Start development
docker run -it --rm -v ~/wheelchair_ws:/workspace ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0

# List running containers
docker ps

# Stop container
docker stop <container-name>

# Open another terminal to running container
docker exec -it <container-name> /bin/bash

# Clean up
docker container prune
docker image prune

# Check disk usage
docker system df
```

---

## Getting Help

1. **Check logs**: `docker logs <container-id>`
2. **Read the error message** - Docker is pretty clear about problems
3. **Ask your team** - They've likely solved this before
4. **Read DOCKER_LEARNING.md** - Comprehensive guide in this repo

---

**YOU'RE READY!**

Don't overthink it. Docker is just a way to run your code in a consistent environment.

**Start with this**:
```bash
docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0
docker run -it --rm -v ~/wheelchair_ws:/workspace ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v3.0
```

Everything else is just variations of this pattern.

You'll catch up to your competition faster than you think! 💪

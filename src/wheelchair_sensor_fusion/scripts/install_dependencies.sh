#!/bin/bash
###############################################################################
# Wheelchair Sensor Fusion - Dependency Installation Script
#
# This script installs all required dependencies for the sensor fusion package
#
# Author: Siddharth Tiwari
# Email: s24035@students.iitmandi.ac.in
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Wheelchair Sensor Fusion Setup${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Detect ROS2 distribution
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS_DISTRO not set. Attempting to detect...${NC}"

    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        export ROS_DISTRO="jazzy"
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        export ROS_DISTRO="humble"
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}Error: No ROS2 installation found${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}Using ROS2 Distribution: ${ROS_DISTRO}${NC}"
echo ""

# Update package lists
echo -e "${GREEN}[1/6] Updating package lists...${NC}"
sudo apt update

# Install ROS2 dependencies
echo -e "${GREEN}[2/6] Installing ROS2 packages...${NC}"
sudo apt install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-rviz2

# Install system dependencies
echo -e "${GREEN}[3/6] Installing system dependencies...${NC}"
sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-sklearn \
    python3-colcon-common-extensions \
    git \
    wget \
    curl

# Install RealSense SDK if not already installed
echo -e "${GREEN}[4/6] Checking RealSense SDK...${NC}"
if ! dpkg -l | grep -q librealsense2; then
    echo -e "${YELLOW}Installing RealSense SDK...${NC}"

    # Add Intel server to apt repositories
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
        sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list

    sudo apt update
    sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev

    echo -e "${GREEN}RealSense SDK installed successfully${NC}"
else
    echo -e "${GREEN}RealSense SDK already installed${NC}"
fi

# Install Python packages
echo -e "${GREEN}[5/6] Installing Python packages...${NC}"

# Check if CUDA is available
if command -v nvidia-smi &> /dev/null; then
    echo -e "${GREEN}NVIDIA GPU detected - Installing PyTorch with CUDA support${NC}"
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
else
    echo -e "${YELLOW}No NVIDIA GPU detected - Installing CPU-only PyTorch${NC}"
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
fi

# Install Ultralytics YOLO
pip3 install ultralytics

# Install additional Python dependencies
pip3 install \
    opencv-python \
    numpy \
    scikit-learn \
    matplotlib \
    pyyaml

# Download YOLO model
echo -e "${GREEN}[6/6] Downloading YOLOv11 model...${NC}"
MODELS_DIR="$HOME/.cache/ultralytics"
mkdir -p "$MODELS_DIR"

if [ ! -f "$MODELS_DIR/yolov11n.pt" ]; then
    echo -e "${YELLOW}Downloading YOLOv11 Nano model...${NC}"
    wget -O "$MODELS_DIR/yolov11n.pt" \
        https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov11n.pt
    echo -e "${GREEN}YOLOv11 model downloaded${NC}"
else
    echo -e "${GREEN}YOLOv11 model already exists${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Build the package:"
echo "   cd ~/ros2_ws  # or your workspace"
echo "   colcon build --packages-select wheelchair_sensor_fusion"
echo "   source install/setup.bash"
echo ""
echo "2. Test the installation:"
echo "   ros2 launch wheelchair_sensor_fusion sensor_fusion.launch.py"
echo ""
echo -e "${GREEN}Happy coding!${NC}"

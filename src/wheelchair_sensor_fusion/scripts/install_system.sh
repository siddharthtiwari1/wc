#!/bin/bash
###############################################################################
# Wheelchair Sensor Fusion - Complete System Installation Script
###############################################################################
# This script performs a COMPLETE installation of all dependencies and
# components required for the wheelchair sensor fusion system.
#
# Target: Ubuntu 24.04 + ROS2 Jazzy
# Hardware: RPLidar S3, Intel RealSense D455
# Compute: Jetson Orin Nano OR x86 + NVIDIA GPU
#
# Author: Siddharth Tiwari (IIT Mandi)
# Usage: ./install_system.sh [--skip-ros2] [--skip-gpu]
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Parse arguments
SKIP_ROS2=false
SKIP_GPU=false
for arg in "$@"; do
    case $arg in
        --skip-ros2)
            SKIP_ROS2=true
            shift
            ;;
        --skip-gpu)
            SKIP_GPU=true
            shift
            ;;
    esac
done

echo "###############################################################################"
echo "#  WHEELCHAIR SENSOR FUSION - INSTALLATION SCRIPT"
echo "###############################################################################"
echo ""

# Check OS
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$NAME
    VER=$VERSION_ID
else
    log_error "Cannot detect OS"
    exit 1
fi

log_info "Detected OS: $OS $VER"

if [[ ! "$OS" =~ "Ubuntu" ]] || [[ "$VER" != "24.04" ]]; then
    log_warn "This script is designed for Ubuntu 24.04"
    log_warn "Current OS: $OS $VER"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Detect hardware
if [ -d "/sys/module/tegra_fuse" ]; then
    PLATFORM="jetson"
    log_info "Detected platform: NVIDIA Jetson"
else
    PLATFORM="x86"
    log_info "Detected platform: x86/x64"
fi

# Detect GPU
if command -v nvidia-smi &> /dev/null; then
    GPU_AVAILABLE=true
    GPU_INFO=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n1)
    log_success "GPU detected: $GPU_INFO"
else
    GPU_AVAILABLE=false
    log_warn "No NVIDIA GPU detected - will use CPU mode (slower)"
fi

###############################################################################
# STEP 1: System Updates
###############################################################################
log_info "Step 1/10: Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y
log_success "System updated"

###############################################################################
# STEP 2: ROS2 Jazzy Installation
###############################################################################
if [ "$SKIP_ROS2" = false ]; then
    log_info "Step 2/10: Installing ROS2 Jazzy..."

    # Check if ROS2 already installed
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        log_success "ROS2 Jazzy already installed"
    else
        log_info "Adding ROS2 repository..."
        sudo apt install -y software-properties-common
        sudo add-apt-repository universe -y
        sudo apt update && sudo apt install -y curl

        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg

        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
            sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

        sudo apt update
        sudo apt install -y ros-jazzy-desktop
        sudo apt install -y ros-dev-tools

        # Setup environment
        echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
        source /opt/ros/jazzy/setup.bash

        log_success "ROS2 Jazzy installed"
    fi
else
    log_info "Step 2/10: Skipping ROS2 installation (--skip-ros2 flag)"
fi

# Source ROS2 if available
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

###############################################################################
# STEP 3: ROS2 Dependencies
###############################################################################
log_info "Step 3/10: Installing ROS2 dependencies..."
sudo apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-message-filters \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization

log_success "ROS2 dependencies installed"

###############################################################################
# STEP 4: Python Dependencies
###############################################################################
log_info "Step 4/10: Installing Python dependencies..."
sudo apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-opencv \
    python3-sklearn \
    python3-yaml \
    python3-rosdep

# Install pip packages
pip3 install --upgrade pip
pip3 install numpy>=1.20.0
pip3 install opencv-python>=4.5.0
pip3 install scikit-learn>=1.0.0
pip3 install scipy>=1.7.0

log_success "Python dependencies installed"

###############################################################################
# STEP 5: PyTorch and CUDA (if GPU available)
###############################################################################
if [ "$SKIP_GPU" = false ] && [ "$GPU_AVAILABLE" = true ]; then
    log_info "Step 5/10: Installing PyTorch with CUDA support..."

    if [ "$PLATFORM" = "jetson" ]; then
        log_info "Installing PyTorch for Jetson..."
        # Jetson-specific PyTorch installation
        pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
    else
        log_info "Installing PyTorch with CUDA 12.1..."
        pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
    fi

    # Verify CUDA
    python3 -c "import torch; print(f'PyTorch CUDA available: {torch.cuda.is_available()}')"
    log_success "PyTorch with CUDA installed"
else
    log_info "Step 5/10: Installing PyTorch (CPU-only)..."
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
    log_success "PyTorch (CPU) installed"
fi

###############################################################################
# STEP 6: Ultralytics YOLOv11
###############################################################################
log_info "Step 6/10: Installing Ultralytics YOLOv11..."
pip3 install ultralytics>=8.0.0

# Download YOLOv11 models
log_info "Downloading YOLOv11 models..."
mkdir -p ~/yolo_models
cd ~/yolo_models

python3 << EOF
from ultralytics import YOLO
# Download nano model (fastest for embedded)
model_n = YOLO('yolov11n.pt')
print("YOLOv11n downloaded")

# Download small model (better accuracy)
model_s = YOLO('yolov11s.pt')
print("YOLOv11s downloaded")
EOF

log_success "YOLOv11 installed and models downloaded"

###############################################################################
# STEP 7: RealSense SDK
###############################################################################
log_info "Step 7/10: Installing Intel RealSense SDK..."

# Check if already installed
if dpkg -l | grep -q librealsense2; then
    log_success "RealSense SDK already installed"
else
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
        sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list

    sudo apt-get update
    sudo apt-get install -y \
        librealsense2-dkms \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg

    # Install ROS2 wrapper
    sudo apt-get install -y ros-jazzy-realsense2-camera

    log_success "RealSense SDK installed"
fi

###############################################################################
# STEP 8: RPLidar Driver
###############################################################################
log_info "Step 8/10: Installing RPLidar driver..."

# Install ROS2 RPLidar package
sudo apt-get install -y ros-jazzy-rplidar-ros || {
    log_warn "RPLidar ROS2 package not in apt, building from source..."

    # Create workspace if doesn't exist
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

    if [ ! -d "rplidar_ros" ]; then
        git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
    fi

    cd ~/ros2_ws
    colcon build --packages-select rplidar_ros
    source install/setup.bash
}

log_success "RPLidar driver installed"

###############################################################################
# STEP 9: Build Wheelchair Sensor Fusion Package
###############################################################################
log_info "Step 9/10: Building wheelchair_sensor_fusion package..."

# Determine workspace root (go up from scripts dir)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR="$( dirname "$SCRIPT_DIR" )"
WS_DIR="$( dirname "$( dirname "$PKG_DIR" )" )"

log_info "Workspace: $WS_DIR"
log_info "Package: $PKG_DIR"

cd "$WS_DIR"

# Install rosdep dependencies
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --packages-select wheelchair_sensor_fusion --symlink-install

if [ $? -eq 0 ]; then
    log_success "Package built successfully"
    echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
else
    log_error "Package build failed"
    exit 1
fi

###############################################################################
# STEP 10: Validation and Testing
###############################################################################
log_info "Step 10/10: Validating installation..."

source "$WS_DIR/install/setup.bash"

# Check if nodes are executable
log_info "Checking node executables..."
ros2 pkg executables wheelchair_sensor_fusion

# Verify Python imports
log_info "Validating Python dependencies..."
python3 << EOF
import sys
try:
    import numpy
    import cv2
    import sklearn
    import scipy
    import ultralytics
    print("✓ All Python dependencies OK")
except ImportError as e:
    print(f"✗ Missing dependency: {e}")
    sys.exit(1)

try:
    import torch
    print(f"✓ PyTorch {torch.__version__} installed")
    print(f"  CUDA available: {torch.cuda.is_available()}")
except ImportError:
    print("✗ PyTorch not installed")
    sys.exit(1)
EOF

if [ $? -eq 0 ]; then
    log_success "All validations passed"
else
    log_error "Validation failed"
    exit 1
fi

###############################################################################
# INSTALLATION COMPLETE
###############################################################################
echo ""
echo "###############################################################################"
echo "#  INSTALLATION COMPLETE!"
echo "###############################################################################"
echo ""
log_success "Wheelchair sensor fusion system installed successfully"
echo ""
echo "Platform: $PLATFORM"
echo "GPU: $([ "$GPU_AVAILABLE" = true ] && echo "Available ($GPU_INFO)" || echo "Not available (CPU mode)")"
echo ""
echo "Next steps:"
echo "1. Source the workspace: source ~/.bashrc"
echo "2. Connect sensors (RPLidar S3, RealSense D455)"
echo "3. Verify sensors:"
echo "   - Camera: ros2 topic echo /camera/color/image_raw --once"
echo "   - LiDAR:  ros2 topic echo /scan --once"
echo "4. Launch fusion system:"
echo "   ros2 launch wheelchair_sensor_fusion wheelchair_fusion.launch.py"
echo ""
echo "Documentation: src/wheelchair_sensor_fusion/WHEELCHAIR_INTEGRATION.md"
echo "###############################################################################"

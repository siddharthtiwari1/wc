#!/bin/bash

################################################################################
# Dense Crowd Navigation - Complete Setup Script
#
# This script sets up the entire development and deployment environment
# for wheelchair dense crowd navigation using CrowdSurfer and Diffusion models.
#
# Author: Siddharth Tiwari
# System: RTX 5050 | CUDA 13.0 | ROS2 Jazzy
################################################################################

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

# Header
echo "========================================================================="
echo "  Dense Crowd Navigation Setup"
echo "  ROS2 Jazzy | CUDA 13.0 | RTX 5050"
echo "========================================================================="
echo ""

# Check if running from workspace root
if [ ! -f "IMPLEMENTATION_GUIDE.md" ]; then
    log_error "Please run this script from the workspace root (~/wc)"
    exit 1
fi

WORKSPACE_ROOT=$(pwd)
log_info "Workspace: $WORKSPACE_ROOT"

################################################################################
# Step 1: Check System Requirements
################################################################################

log_info "Step 1/8: Checking system requirements..."

# Check ROS2 Jazzy
if ! command -v ros2 &> /dev/null; then
    log_error "ROS2 not found. Please install ROS2 Jazzy first."
    log_info "Installation guide: https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

ROS_DISTRO=$(printenv ROS_DISTRO)
if [ "$ROS_DISTRO" != "jazzy" ]; then
    log_warn "ROS_DISTRO is $ROS_DISTRO, expected 'jazzy'"
    log_info "Sourcing ROS2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
fi

log_success "ROS2 Jazzy detected"

# Check Python version
PYTHON_VERSION=$(python3 --version | grep -oP '\d+\.\d+')
if [ $(echo "$PYTHON_VERSION >= 3.10" | bc) -eq 0 ]; then
    log_error "Python 3.10+ required, found Python $PYTHON_VERSION"
    exit 1
fi
log_success "Python $PYTHON_VERSION detected"

# Check CUDA
if command -v nvidia-smi &> /dev/null; then
    CUDA_VERSION=$(nvidia-smi | grep "CUDA Version" | awk '{print $9}')
    log_success "CUDA $CUDA_VERSION detected"

    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
    GPU_MEMORY=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader | head -1)
    log_info "GPU: $GPU_NAME | Memory: $GPU_MEMORY"
else
    log_warn "NVIDIA GPU not detected. CPU-only mode will be slow."
fi

################################################################################
# Step 2: Create Python Virtual Environment
################################################################################

log_info "Step 2/8: Setting up Python virtual environment..."

VENV_DIR="$HOME/crowdnav_venv"

if [ -d "$VENV_DIR" ]; then
    log_warn "Virtual environment already exists at $VENV_DIR"
    read -p "Recreate it? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$VENV_DIR"
        python3 -m venv "$VENV_DIR"
    fi
else
    python3 -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"
log_success "Virtual environment activated"

# Upgrade pip
pip install --upgrade pip setuptools wheel
log_success "pip upgraded"

################################################################################
# Step 3: Install PyTorch with CUDA 13.0
################################################################################

log_info "Step 3/8: Installing PyTorch with CUDA 13.0 support..."

if python3 -c "import torch; print(torch.cuda.is_available())" 2>/dev/null | grep -q "True"; then
    log_warn "PyTorch with CUDA already installed"
    TORCH_VERSION=$(python3 -c "import torch; print(torch.__version__)")
    log_info "Current PyTorch version: $TORCH_VERSION"
    read -p "Reinstall? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "Skipping PyTorch installation"
    else
        pip install --upgrade torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130
    fi
else
    log_info "Installing PyTorch with CUDA 13.0..."
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130
fi

# Verify PyTorch installation
if python3 -c "import torch; assert torch.cuda.is_available(), 'CUDA not available'; print(f'PyTorch {torch.__version__} with CUDA {torch.version.cuda}')"; then
    log_success "PyTorch with CUDA installed successfully"
else
    log_error "PyTorch CUDA installation failed"
    exit 1
fi

################################################################################
# Step 4: Install JAX with CUDA support
################################################################################

log_info "Step 4/8: Installing JAX with CUDA 13.0 support..."

pip install "jax[cuda13_pip]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html

# Verify JAX installation
if python3 -c "import jax; print(f'JAX version: {jax.__version__}')"; then
    log_success "JAX installed successfully"
else
    log_warn "JAX installation may have issues"
fi

################################################################################
# Step 5: Install Other Python Dependencies
################################################################################

log_info "Step 5/8: Installing remaining Python packages..."

pip install -r requirements.txt

log_success "All Python dependencies installed"

################################################################################
# Step 6: Install ROS2 System Dependencies
################################################################################

log_info "Step 6/8: Installing ROS2 system dependencies..."

sudo apt update

# Install Nav2 and required packages
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-gazebo-ros2-control \
    ros-jazzy-robot-localization \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf-transformations \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

log_success "ROS2 dependencies installed"

################################################################################
# Step 7: Clone and Setup Simulation Dependencies
################################################################################

log_info "Step 7/8: Setting up simulation environment..."

cd "$WORKSPACE_ROOT/src"

# Clone HuNavSim if not exists
if [ ! -d "hunav_sim" ]; then
    log_info "Cloning HuNavSim..."
    git clone https://github.com/robotics-upo/hunav_sim.git -b jazzy-devel 2>/dev/null || \
    git clone https://github.com/robotics-upo/hunav_sim.git
    log_success "HuNavSim cloned"
else
    log_info "HuNavSim already exists"
fi

# Clone Gazebo wrapper if not exists
if [ ! -d "hunav_gazebo_wrapper" ]; then
    log_info "Cloning HuNavSim Gazebo wrapper..."
    git clone https://github.com/robotics-upo/hunav_gazebo_wrapper.git
    log_success "Gazebo wrapper cloned"
else
    log_info "Gazebo wrapper already exists"
fi

cd "$WORKSPACE_ROOT"

################################################################################
# Step 8: Build ROS2 Workspace
################################################################################

log_info "Step 8/8: Building ROS2 workspace..."

source /opt/ros/jazzy/setup.bash

# Update rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
log_info "Running colcon build (this may take several minutes)..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4

if [ $? -eq 0 ]; then
    log_success "Workspace built successfully"
else
    log_error "Build failed"
    exit 1
fi

# Source the workspace
source install/setup.bash

################################################################################
# Finalization
################################################################################

echo ""
echo "========================================================================="
echo "  ✅ Setup Complete!"
echo "========================================================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Activate environment:"
echo "   source ~/crowdnav_venv/bin/activate"
echo "   source ~/wc/install/setup.bash"
echo ""
echo "2. Download pre-trained models (optional):"
echo "   ./scripts/download_models.sh"
echo ""
echo "3. Launch simulation:"
echo "   ros2 launch crowdsurfer_nav full_simulation.launch.py"
echo ""
echo "For training from scratch, see IMPLEMENTATION_GUIDE.md"
echo ""
echo "========================================================================="

# Create activation helper script
cat > "$WORKSPACE_ROOT/activate.sh" <<'EOF'
#!/bin/bash
# Quick activation script
source ~/crowdnav_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/wc/install/setup.bash
export WC_MODEL_PATH=~/models/deployed
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/wc/src/hunav_gazebo_wrapper/models
echo "✅ CrowdNav environment activated"
EOF

chmod +x "$WORKSPACE_ROOT/activate.sh"

log_success "Created activation script: ./activate.sh"
echo ""
log_info "Run './activate.sh' in future terminals to activate the environment"

exit 0

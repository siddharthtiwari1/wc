#!/bin/bash
###############################################################################
# Wheelchair Sensor Fusion - System Validation Script
###############################################################################
# This script validates that the entire sensor fusion system is correctly
# installed and ready for operation.
#
# Usage: ./validate_system.sh
# Returns: Exit code 0 if all checks pass, 1 if any check fails
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

FAILURES=0
WARNINGS=0
PASSED=0

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
    ((PASSED++))
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    ((FAILURES++))
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
    ((WARNINGS++))
}

echo "###############################################################################"
echo "#  WHEELCHAIR SENSOR FUSION - SYSTEM VALIDATION"
echo "###############################################################################"
echo ""

###############################################################################
# 1. ROS2 INSTALLATION
###############################################################################
echo "==> Checking ROS2 Installation..."

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    check_pass "ROS2 Jazzy installed"
    source /opt/ros/jazzy/setup.bash
else
    check_fail "ROS2 Jazzy not found at /opt/ros/jazzy"
    echo "  Run: sudo apt install ros-jazzy-desktop"
    exit 1
fi

if command -v ros2 &> /dev/null; then
    check_pass "ros2 command available"
else
    check_fail "ros2 command not found"
    echo "  Source ROS2: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

###############################################################################
# 2. WORKSPACE AND PACKAGE
###############################################################################
echo ""
echo "==> Checking Workspace and Package..."

# Find workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR="$( dirname "$SCRIPT_DIR" )"
WS_DIR="$( dirname "$( dirname "$PKG_DIR" )" )"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    check_pass "Workspace found at $WS_DIR"
    source "$WS_DIR/install/setup.bash"
else
    check_fail "Workspace not built at $WS_DIR"
    echo "  Run: cd $WS_DIR && colcon build"
    exit 1
fi

# Check if package is built
if ros2 pkg prefix wheelchair_sensor_fusion &> /dev/null; then
    check_pass "wheelchair_sensor_fusion package found"
else
    check_fail "wheelchair_sensor_fusion package not found"
    echo "  Run: cd $WS_DIR && colcon build --packages-select wheelchair_sensor_fusion"
    exit 1
fi

# Check executables
EXECUTABLES=("lidar_processor_node" "yolo_detector_node" "sensor_fusion_node_robust" "obstacle_publisher_node")
for exe in "${EXECUTABLES[@]}"; do
    if ros2 pkg executables wheelchair_sensor_fusion | grep -q "$exe"; then
        check_pass "Executable found: $exe"
    else
        check_fail "Executable missing: $exe"
    fi
done

###############################################################################
# 3. PYTHON DEPENDENCIES
###############################################################################
echo ""
echo "==> Checking Python Dependencies..."

python3 << 'EOF'
import sys

deps = {
    'numpy': 'numpy',
    'cv2': 'opencv-python',
    'sklearn': 'scikit-learn',
    'scipy': 'scipy',
    'ultralytics': 'ultralytics (YOLOv11)',
    'torch': 'PyTorch'
}

failures = []
for module, name in deps.items():
    try:
        __import__(module)
        print(f"\033[0;32m✓\033[0m {name}")
    except ImportError:
        print(f"\033[0;31m✗\033[0m {name} - MISSING")
        failures.append(name)

if failures:
    print(f"\n\033[0;31mInstall missing dependencies:\033[0m")
    print(f"pip3 install numpy opencv-python scikit-learn scipy ultralytics torch")
    sys.exit(1)
EOF

if [ $? -eq 0 ]; then
    check_pass "All Python dependencies installed"
else
    check_fail "Missing Python dependencies"
    ((FAILURES++))
fi

###############################################################################
# 4. GPU/CUDA CHECK
###############################################################################
echo ""
echo "==> Checking GPU/CUDA..."

if command -v nvidia-smi &> /dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n1)
    check_pass "NVIDIA GPU detected: $GPU_NAME"

    python3 << 'EOF'
try:
    import torch
    if torch.cuda.is_available():
        print(f"\033[0;32m✓\033[0m PyTorch CUDA available: {torch.cuda.get_device_name(0)}")
    else:
        print(f"\033[1;33m⚠\033[0m PyTorch installed but CUDA not available")
        print("  System will use CPU mode (slower)")
except:
    print(f"\033[0;31m✗\033[0m PyTorch not properly installed")
EOF
else
    check_warn "No NVIDIA GPU detected - will use CPU mode"
    echo "  Performance will be reduced (10-15 Hz instead of 30 Hz)"
fi

###############################################################################
# 5. SENSOR DRIVERS
###############################################################################
echo ""
echo "==> Checking Sensor Drivers..."

# RealSense
if command -v rs-enumerate-devices &> /dev/null; then
    check_pass "RealSense SDK installed"

    if ros2 pkg list | grep -q realsense2_camera; then
        check_pass "RealSense ROS2 wrapper installed"
    else
        check_fail "RealSense ROS2 wrapper not installed"
        echo "  Run: sudo apt install ros-jazzy-realsense2-camera"
    fi
else
    check_fail "RealSense SDK not installed"
    echo "  Install from: https://github.com/IntelRealSense/librealsense"
fi

# RPLidar
if ros2 pkg list | grep -q rplidar_ros; then
    check_pass "RPLidar ROS2 package installed"
else
    check_warn "RPLidar ROS2 package not found"
    echo "  Install: sudo apt install ros-jazzy-rplidar-ros"
    echo "  OR build from: https://github.com/Slamtec/rplidar_ros"
fi

###############################################################################
# 6. YOLO MODELS
###############################################################################
echo ""
echo "==> Checking YOLO Models..."

python3 << 'EOF'
import os
from pathlib import Path

model_paths = [
    Path.home() / 'yolo_models' / 'yolov11n.pt',
    Path.home() / 'yolo_models' / 'yolov11s.pt',
]

found = False
for path in model_paths:
    if path.exists():
        print(f"\033[0;32m✓\033[0m Model found: {path}")
        found = True

if not found:
    print(f"\033[1;33m⚠\033[0m No YOLO models found")
    print("  Download models:")
    print("  python3 -c 'from ultralytics import YOLO; YOLO(\"yolov11n.pt\")'")
EOF

###############################################################################
# 7. CONFIGURATION FILES
###############################################################################
echo ""
echo "==> Checking Configuration Files..."

CONFIG_FILES=(
    "$PKG_DIR/config/wheelchair_integration.yaml"
    "$PKG_DIR/config/sensor_fusion.yaml"
)

for config in "${CONFIG_FILES[@]}"; do
    if [ -f "$config" ]; then
        check_pass "Config found: $(basename $config)"
    else
        check_warn "Config missing: $(basename $config)"
    fi
done

###############################################################################
# 8. LAUNCH FILES
###############################################################################
echo ""
echo "==> Checking Launch Files..."

LAUNCH_FILES=(
    "$PKG_DIR/launch/wheelchair_fusion.launch.py"
    "$PKG_DIR/launch/sensor_fusion.launch.py"
)

for launch in "${LAUNCH_FILES[@]}"; do
    if [ -f "$launch" ]; then
        check_pass "Launch found: $(basename $launch)"
    else
        check_fail "Launch missing: $(basename $launch)"
    fi
done

###############################################################################
# 9. ROS2 DEPENDENCIES
###############################################################################
echo ""
echo "==> Checking ROS2 Dependencies..."

ROS_PACKAGES=(
    "vision_msgs"
    "cv_bridge"
    "tf2_ros"
    "message_filters"
    "nav2_bringup"
)

for pkg in "${ROS_PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^$pkg\$"; then
        check_pass "ROS2 package: $pkg"
    else
        check_fail "ROS2 package missing: $pkg"
        echo "  Install: sudo apt install ros-jazzy-$pkg"
    fi
done

###############################################################################
# 10. HARDWARE SENSORS (IF CONNECTED)
###############################################################################
echo ""
echo "==> Checking Hardware Sensors (if connected)..."

# Check for RealSense
if command -v rs-enumerate-devices &> /dev/null; then
    RS_OUTPUT=$(rs-enumerate-devices 2>&1)
    if echo "$RS_OUTPUT" | grep -q "Device info"; then
        SERIAL=$(echo "$RS_OUTPUT" | grep "Serial Number" | head -n1 | awk '{print $NF}')
        check_pass "RealSense D455 detected (Serial: $SERIAL)"
    else
        check_warn "RealSense not connected (or not detected)"
        echo "  Connect camera to USB 3.0 port and run: rs-enumerate-devices"
    fi
fi

# Check for LiDAR (USB serial)
if ls /dev/ttyUSB* &> /dev/null; then
    USB_DEVICES=$(ls /dev/ttyUSB*)
    check_warn "USB serial devices found: $USB_DEVICES"
    echo "  RPLidar likely at /dev/ttyUSB0"
    echo "  Grant permissions: sudo chmod 666 /dev/ttyUSB0"
else
    check_warn "No USB serial devices found"
    echo "  Connect RPLidar and check with: ls /dev/ttyUSB*"
fi

###############################################################################
# SUMMARY
###############################################################################
echo ""
echo "###############################################################################"
echo "#  VALIDATION SUMMARY"
echo "###############################################################################"
echo ""
echo -e "${GREEN}Passed:${NC}   $PASSED"
echo -e "${YELLOW}Warnings:${NC} $WARNINGS"
echo -e "${RED}Failed:${NC}   $FAILURES"
echo ""

if [ $FAILURES -eq 0 ]; then
    echo -e "${GREEN}✓ SYSTEM VALIDATION PASSED${NC}"
    echo ""
    echo "System is ready to launch!"
    echo ""
    echo "Next steps:"
    echo "1. Connect hardware (RPLidar + RealSense)"
    echo "2. Launch fusion: ros2 launch wheelchair_sensor_fusion wheelchair_fusion.launch.py"
    echo "3. Check status: ros2 topic echo /fusion/status"
    echo ""
    echo "See QUICKSTART.md for detailed launch instructions"
    echo ""
    exit 0
else
    echo -e "${RED}✗ SYSTEM VALIDATION FAILED${NC}"
    echo ""
    echo "Fix the failed checks above before launching."
    echo ""
    echo "For help:"
    echo "- Check QUICKSTART.md troubleshooting section"
    echo "- Run installation script: ./install_system.sh"
    echo "- Check logs and documentation"
    echo ""
    exit 1
fi

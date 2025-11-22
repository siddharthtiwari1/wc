#!/bin/bash
###############################################################################
# Wheelchair Sensor Fusion - Sensor Calibration Helper
###############################################################################
# Helps calibrate and verify sensor setup:
# - Checks sensor connectivity
# - Verifies coordinate frame transforms
# - Tests data quality
# - Guides manual calibration if needed
#
# Author: Siddharth Tiwari (IIT Mandi)
# Usage: ./calibrate_sensors.sh
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

echo "###############################################################################"
echo "#  WHEELCHAIR SENSOR FUSION - SENSOR CALIBRATION"
echo "###############################################################################"
echo ""

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    log_error "ROS2 not found. Source your workspace first:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source ~/ros2_ws/install/setup.bash"
    exit 1
fi

echo "This script will help you calibrate and verify your sensor setup."
echo "Make sure both sensors (RPLidar + RealSense) are connected."
echo ""
read -p "Press Enter to continue..."
echo ""

###############################################################################
# STEP 1: Check Sensor Connectivity
###############################################################################
log_info "Step 1/5: Checking sensor connectivity..."
echo ""

# Check RPLidar
if ls /dev/ttyUSB* &> /dev/null; then
    USB_DEVICE=$(ls /dev/ttyUSB* | head -n1)
    log_success "RPLidar USB device found: $USB_DEVICE"

    # Check permissions
    if [ -r "$USB_DEVICE" ] && [ -w "$USB_DEVICE" ]; then
        log_success "Permissions OK for $USB_DEVICE"
    else
        log_warn "Insufficient permissions for $USB_DEVICE"
        log_info "Fix with: sudo chmod 666 $USB_DEVICE"
        log_info "Or permanently: sudo usermod -a -G dialout $USER (then logout/login)"
    fi
else
    log_error "RPLidar not found. Check USB connection."
    exit 1
fi

echo ""

# Check RealSense
if command -v rs-enumerate-devices &> /dev/null; then
    RS_OUTPUT=$(rs-enumerate-devices 2>&1)

    if echo "$RS_OUTPUT" | grep -q "Device info"; then
        SERIAL=$(echo "$RS_OUTPUT" | grep "Serial Number" | head -n1 | awk '{print $NF}')
        FIRMWARE=$(echo "$RS_OUTPUT" | grep "Firmware Version" | head -n1 | awk '{print $NF}')
        log_success "RealSense D455 detected"
        log_info "  Serial: $SERIAL"
        log_info "  Firmware: $FIRMWARE"
    else
        log_error "RealSense not detected. Check USB 3.0 connection."
        exit 1
    fi
else
    log_error "RealSense SDK not installed. Run install_system.sh first."
    exit 1
fi

echo ""
read -p "Press Enter to continue to Step 2..."
echo ""

###############################################################################
# STEP 2: Launch Sensors
###############################################################################
log_info "Step 2/5: Launching sensor nodes..."
echo ""

log_info "Launching RealSense camera..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch realsense2_camera rs_launch.py; exec bash" &
CAMERA_PID=$!
sleep 3

log_info "Launching RPLidar..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch rplidar_ros rplidar.launch.py; exec bash" &
LIDAR_PID=$!
sleep 3

log_success "Sensor nodes launched in separate terminals"
echo ""
read -p "Press Enter to continue to Step 3..."
echo ""

###############################################################################
# STEP 3: Verify Data Streams
###############################################################################
log_info "Step 3/5: Verifying sensor data streams..."
echo ""

# Check LiDAR scan
log_info "Checking LiDAR scan..."
if timeout 5 ros2 topic echo /scan --once > /tmp/lidar_check.txt 2>&1; then
    RANGES=$(grep -o "ranges:" /tmp/lidar_check.txt | wc -l)
    if [ "$RANGES" -gt 0 ]; then
        log_success "LiDAR publishing scan data (/scan)"

        # Check range values
        ros2 topic echo /scan --once 2>&1 | head -n 30 | tee /tmp/lidar_sample.txt
        echo ""
        log_info "Sample LiDAR readings shown above. Check for:"
        log_info "  - No excessive 'inf' values (bad data)"
        log_info "  - Range values in reasonable range (0.1m - 6m indoor)"
    else
        log_error "LiDAR not publishing valid data"
        exit 1
    fi
else
    log_error "LiDAR topic /scan not available"
    exit 1
fi

echo ""

# Check camera color image
log_info "Checking camera color image..."
if timeout 5 ros2 topic echo /camera/color/image_raw --once > /tmp/camera_check.txt 2>&1; then
    WIDTH=$(grep "width:" /tmp/camera_check.txt | awk '{print $2}')
    HEIGHT=$(grep "height:" /tmp/camera_check.txt | awk '{print $2}')
    log_success "Camera publishing color image ($WIDTH x $HEIGHT)"
else
    log_error "Camera topic /camera/color/image_raw not available"
    exit 1
fi

echo ""

# Check depth image
log_info "Checking camera depth image..."
if timeout 5 ros2 topic echo /camera/aligned_depth_to_color/image_raw --once > /tmp/depth_check.txt 2>&1; then
    log_success "Camera publishing aligned depth image"
else
    log_warn "Aligned depth topic not available. Check camera configuration."
fi

echo ""
read -p "Press Enter to continue to Step 4..."
echo ""

###############################################################################
# STEP 4: Check TF Transforms
###############################################################################
log_info "Step 4/5: Checking coordinate frame transforms..."
echo ""

log_info "Generating TF tree... (this takes 5 seconds)"
timeout 5 ros2 run tf2_tools view_frames &
FRAMES_PID=$!
sleep 6

if [ -f "frames.pdf" ]; then
    log_success "TF tree generated: frames.pdf"
    log_info "Opening TF tree..."
    xdg-open frames.pdf &
    echo ""
    log_info "Verify the following frames exist:"
    echo "  - base_link (wheelchair base)"
    echo "  - laser_frame (RPLidar)"
    echo "  - camera_link (RealSense)"
    echo "  - camera_color_optical_frame (RGB camera)"
    echo "  - camera_depth_optical_frame (Depth camera)"
    echo ""
    log_info "If frames are missing, you need to publish static transforms."
    log_info "See config/wheelchair_integration.yaml for transform setup."
else
    log_warn "Could not generate TF tree. Check if transforms are being published."
fi

echo ""
read -p "Press Enter to continue to Step 5..."
echo ""

###############################################################################
# STEP 5: Calibration Checklist
###############################################################################
log_info "Step 5/5: Calibration checklist"
echo ""

cat << 'EOF'
Complete the following calibration steps:

1. PHYSICAL MOUNTING
   [ ] RealSense D455 mounted rigidly on wheelchair
   [ ] RPLidar S3 mounted horizontally at chest height
   [ ] Both sensors have clear field of view
   [ ] No vibration or movement of mounts

2. COORDINATE FRAMES
   [ ] base_link at wheelchair center
   [ ] laser_frame correctly positioned relative to base_link
   [ ] camera_link correctly positioned relative to base_link
   [ ] All transforms are STATIC (no drift over time)

3. SENSOR ALIGNMENT
   [ ] LiDAR and camera roughly point in same direction
   [ ] LiDAR scan plane is horizontal
   [ ] Camera is level (not tilted up/down)

4. DATA QUALITY
   [ ] LiDAR detects obstacles at 1-6m range
   [ ] Camera image is clear and properly exposed
   [ ] Depth image shows valid depth values
   [ ] No significant lag or delay

5. FUSION PARAMETERS
   [ ] Review config/wheelchair_integration.yaml
   [ ] Adjust max_obstacle_distance for your environment
   [ ] Tune obstacle_inflation for safety margin
   [ ] Test fusion with: ros2 launch wheelchair_sensor_fusion wheelchair_fusion.launch.py

EOF

echo ""
log_info "Calibration values to measure and update in wheelchair_integration.yaml:"
echo ""
echo "  static_transform_publisher:"
echo "    - Measure LiDAR position relative to base_link (x, y, z in meters)"
echo "    - Measure camera position relative to base_link (x, y, z in meters)"
echo "    - Use a tape measure or ruler for accuracy"
echo ""
echo "  Example measurements for typical wheelchair:"
echo "    LiDAR:  x=0.3, y=0.0, z=0.8  (30cm forward, center, 80cm height)"
echo "    Camera: x=0.3, y=0.0, z=1.0  (30cm forward, center, 100cm height)"
echo ""

###############################################################################
# CLEANUP
###############################################################################
echo ""
log_info "Calibration guide complete!"
echo ""
log_warn "Sensor nodes are still running in separate terminals."
log_info "Close them manually when done testing."
echo ""

log_info "Next steps:"
echo "  1. Update transforms in config/wheelchair_integration.yaml"
echo "  2. Test fusion: ros2 launch wheelchair_sensor_fusion wheelchair_fusion.launch.py"
echo "  3. Verify in RViz: rviz2 -d src/wheelchair_sensor_fusion/rviz/sensor_fusion.rviz"
echo "  4. Run validation: ./scripts/validate_system.sh"
echo ""

###############################################################################
# End of calibration
###############################################################################

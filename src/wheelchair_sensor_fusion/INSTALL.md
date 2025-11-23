# Installation Guide

## Automated Installation (Recommended)

Run the automated installation script:

```bash
cd src/wheelchair_sensor_fusion/scripts
./install_dependencies.sh
```

This will install:
- All ROS2 dependencies
- RealSense SDK
- RPLidar drivers
- Python packages (PyTorch, Ultralytics, OpenCV, etc.)
- YOLOv11 model

## Manual Installation

### 1. System Dependencies

```bash
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-nav2-bringup \
    python3-pip \
    python3-opencv
```

### 2. Python Packages

```bash
# PyTorch (with CUDA)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Ultralytics YOLO
pip3 install ultralytics

# Other dependencies
pip3 install numpy scikit-learn opencv-python
```

### 3. USB Permissions

```bash
./scripts/setup_permissions.sh
# Log out and log back in
```

### 4. Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select wheelchair_sensor_fusion
source install/setup.bash
```

## Verification

Test sensors:

```bash
./scripts/test_sensors.sh
```

Expected output:
- ✓ RealSense camera detected
- ✓ USB serial devices found
- ✓ Permissions OK

## Troubleshooting

### Camera not detected
```bash
rs-enumerate-devices
realsense-viewer
```

### LiDAR permission denied
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Python module not found
```bash
pip3 install --user ultralytics
```

For more issues, see README.md Troubleshooting section.

# Wheelchair Hardware Interface

A robust ROS2 hardware interface for differential drive wheelchairs using Arduino control boards. This interface integrates your final_control Arduino firmware with the ROS2 control framework, enabling seamless operation with diff_drive_controller and navigation systems.

## Features

- **Arduino Integration**: Direct communication with final_control.ino firmware
- **Multiple Control Modes**: Supports wheel velocity, cmd_vel, and PPM input modes
- **Safety Features**: Emergency stop, velocity limiting, command timeout
- **ROS2 Control Integration**: Works with diff_drive_controller and navigation stack
- **Real-time Communication**: 50Hz control loop with robust serial parsing
- **PPM Support**: Priority handling for RC transmitter control
- **Caster Swivel Prevention**: Built-in pivot mode for direction changes

## Hardware Requirements

- Arduino with your final_control.ino firmware
- Wheel encoders (10,000 CPR)
- Motor driver (Cytron SmartDriveDuo)
- USB/Serial connection to ROS2 computer

## Arduino Communication Protocol

The interface communicates with your Arduino using the established protocol:

**Commands to Arduino (Write):**
```
rp5.0,lp3.0,    # Wheel mode: right positive 5.0 rad/s, left positive 3.0 rad/s
x:0.5,t:0.1,    # CMD_VEL mode: 0.5 m/s linear, 0.1 rad/s angular
```

**Feedback from Arduino (Read):**
```
[WHEEL] rp5.23,ln2.14,          # Wheel velocities with mode indicator
[PPM] [PIVOT] rp0.00,lp0.00,    # PPM mode with pivot indication
[CMD_VEL] rp3.45,lp3.45,        # CMD_VEL mode feedback
```

## Setup Instructions

### 1. Build the Package

```bash
cd ~/wc_ws
colcon build --packages-select wheelchair_firmware wc_control wheelchair_description
source install/setup.bash
```

### 2. Arduino Setup

Upload the final_control.ino firmware to your Arduino. Ensure:
- Correct encoder pins (2,3,18,19)
- Proper motor driver connections
- Serial communication at 115200 baud

### 3. Hardware Configuration

Update parameters in `config/wheelchair_hardware.yaml`:

```yaml
# Wheelchair specifications (match your Arduino)
wheel_separation: 0.57    # 57 cm wheelbase
wheel_radius: 0.1524      # 6 inch wheels

# Safety limits (from Arduino MAX values)
max_velocity: 2.68        # MAX_LINEAR_VEL
max_acceleration: 5.0     # MAX_ACCEL
```

### 4. URDF Integration

Include the hardware interface in your robot URDF:

```xml
<xacro:include filename=\"$(find wheelchair_description)/urdf/wheelchair_ros2_control.xacro\" />
```

## Usage

### 1. Start Hardware Interface

```bash
ros2 launch wheelchair_firmware wheelchair_hardware_interface.launch.py
```

This launches:
- Hardware interface node
- Controller manager
- Wheelchair controller (diff_drive_controller)
- Joint state broadcaster

### 2. Test the Interface

```bash
# Run the test script
ros2 run wheelchair_firmware test_wheelchair_interface.py

# Or manually send commands
ros2 topic pub /wheelchair_controller/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

### 3. Monitor Status

```bash
# Check joint states
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers

# Monitor serial communication (debug)
ros2 topic echo /wheelchair_controller/odom
```

## Configuration Files

### wheelchair_hardware.yaml
Main controller configuration with limits and parameters.

### wheelchair_ros2_control.xacro
Hardware interface definition for URDF integration.

### wheelchair_hardware_interface.launch.py
Complete launch file for hardware interface and controllers.

## Safety Features

1. **Command Timeout**: Automatically stops if no commands received (0.5s default)
2. **Velocity Limiting**: Enforces maximum linear (2.68 m/s) and angular (10.0 rad/s) velocities
3. **Emergency Stop**: Immediate stop capability with proper Arduino communication
4. **PPM Priority**: RC transmitter input overrides serial commands
5. **Serial Recovery**: Automatic reconnection on communication failures

## Troubleshooting

### Arduino Not Responding
```bash
# Check USB connection
ls /dev/ttyACM*

# Test direct communication
screen /dev/ttyACM0 115200

# Check permissions
sudo usermod -a -G dialout $USER
# (logout and login again)
```

### Controller Not Starting
```bash
# Check hardware interface status
ros2 control list_hardware_interfaces

# Verify controller configuration
ros2 control list_controllers

# Check for errors
ros2 topic echo /diagnostics
```

### Communication Issues
```bash
# Enable debug logging
ros2 launch wheelchair_firmware wheelchair_hardware_interface.launch.py \
  --ros-args --log-level debug

# Monitor raw serial data
# Add debug prints in parseArduinoData() method
```

## Integration with Navigation

Once the hardware interface is running, you can use it with:

- **Navigation2**: Full autonomous navigation
- **Teleop**: Keyboard or joystick control
- **MoveIt2**: Motion planning (if applicable)
- **SLAM**: Mapping and localization

Example navigation launch:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  map:=/path/to/your/map.yaml
```

## Performance Notes

- **Control Loop**: 50Hz update rate (Arduino: 20Hz, Interface: 50Hz)
- **Latency**: ~20ms typical serial communication delay
- **Accuracy**: Encoder resolution: 10,000 CPR for precise odometry
- **Safety**: Multiple failsafe mechanisms ensure safe operation

## Differences from Python Script

This hardware interface replaces the `arduino_receiver_updated.py` script with:

1. **Direct Integration**: Works with ros2_control framework
2. **Better Performance**: C++ implementation with lower latency
3. **Standard Interface**: Compatible with all ROS2 controllers
4. **Professional Architecture**: Follows ROS2 hardware interface patterns
5. **Enhanced Safety**: Built-in safety features and monitoring

The Arduino firmware remains unchanged - only the ROS2 side is improved for better integration and performance.
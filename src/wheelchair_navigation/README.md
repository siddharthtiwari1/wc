# Wheelchair Navigation Package

This package provides the complete Nav2 navigation stack configuration for the autonomous wheelchair, mirroring the robust functionality from the bumperbot packages.

## Package Structure

```
wheelchair_navigation/
├── behavior_tree/           # Custom behavior tree XML files
│   ├── simple_navigation.xml
│   ├── simple_navigation_w_replanning.xml
│   └── wheelchair_navigation_w_replanning_and_recovery.xml
├── config/                  # Navigation configuration
│   └── nav2_params_wheelchair.yaml
├── launch/                  # Launch files
│   └── navigation.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Behavior Trees

### 1. simple_navigation.xml
- **Use case**: Basic navigation in well-known, static environments
- **Features**: Single path computation followed by path execution
- **Replanning**: No
- **Recovery**: No

### 2. simple_navigation_w_replanning.xml
- **Use case**: Dynamic environments where obstacles may appear
- **Features**: Continuous path replanning at 1 Hz while executing
- **Replanning**: Yes (1 Hz)
- **Recovery**: No

### 3. wheelchair_navigation_w_replanning_and_recovery.xml (Recommended)
- **Use case**: Real-world deployment with unpredictable environments
- **Features**:
  - Continuous path replanning at 1 Hz
  - Recovery behaviors (costmap clearing, spinning, backing up)
  - 2 retry attempts before giving up
- **Replanning**: Yes (1 Hz)
- **Recovery**: Yes
- **Safety**: Wheelchair-optimized parameters (slower backup speed, shorter backup distance)

## Configuration

The `nav2_params_wheelchair.yaml` file contains comprehensive configuration for all Nav2 servers:

- **Controller Server**: DWB local planner with safety parameters
- **Planner Server**: Supports multiple global planners (NavFn, SmacPlanner, custom planners)
- **Behavior Server**: Recovery behaviors (spin, backup, wait, clear costmap)
- **BT Navigator**: Behavior tree execution
- **Smoother Server**: Path smoothing
- **Velocity Smoother**: Real-time velocity smoothing for safety
- **Collision Monitor**: Safety layer for obstacle detection
- **Costmap Layers**: Global and local costmaps with proper configuration

### Safety Features for Wheelchair

The configuration includes wheelchair-specific safety parameters:

- Maximum linear velocity: 0.5 m/s
- Maximum angular velocity: 1.0 rad/s
- Velocity smoother for gradual acceleration/deceleration
- Collision monitor with safety zones
- Conservative costmap inflation for obstacle avoidance

## Usage

### Launch Navigation Stack

```bash
ros2 launch wheelchair_navigation navigation.launch.py
```

### Select Behavior Tree

By default, the navigation stack uses the default Nav2 behavior tree. To use a custom behavior tree:

```bash
ros2 launch wheelchair_navigation navigation.launch.py \
    default_nav_to_pose_bt_xml:=/path/to/behavior_tree/wheelchair_navigation_w_replanning_and_recovery.xml
```

Or modify the `nav2_params_wheelchair.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "$(find-pkg-share wheelchair_navigation)/behavior_tree/wheelchair_navigation_w_replanning_and_recovery.xml"
```

## Integration with Custom Planners

This package integrates with:

- **wheelchair_motion**: Custom local planners (PD, Pure Pursuit)
- **wheelchair_planning**: Custom global planners (Dijkstra, A*)

To use custom planners, update the behavior tree XML files to reference the appropriate planner/controller IDs defined in `nav2_params_wheelchair.yaml`.

## Dependencies

- nav2_bringup
- nav2_bt_navigator
- nav2_controller
- nav2_planner
- nav2_behaviors
- nav2_smoother
- nav2_velocity_smoother
- nav2_collision_monitor
- wheelchair_planning (custom global planners)

## Building

```bash
cd /home/user/autonomous_wheelchair
colcon build --packages-select wheelchair_navigation
source install/setup.bash
```

## Related Packages

- **wheelchair_motion**: Local trajectory controllers
- **wheelchair_planning**: Global path planners
- **wheelchair_localization**: SLAM and localization
- **wheelchair_bringup**: System integration and launch files
- **wheelchair_description**: Robot URDF and visualization

## Comparison with Bumperbot

This package successfully replicates and enhances the bumperbot navigation functionality:

✅ Complete Nav2 integration
✅ Custom behavior trees (3 variants)
✅ Comprehensive navigation parameters
✅ Plugin support for custom planners/controllers
✅ Wheelchair-specific safety features
✅ Enhanced documentation

## Notes

- For simulation, set `use_sim_time: true` in `nav2_params_wheelchair.yaml`
- For hardware deployment, ensure all sensor topics are properly configured
- The recovery behavior tree includes costmap clearing for better performance in challenging scenarios
- Backup speeds and distances are conservative for wheelchair safety (0.10 m/s, 0.20 m)

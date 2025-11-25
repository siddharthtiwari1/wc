# Bumperbot WS - Nav2 Autonomous Navigation Learnings

**Source**: `/home/sidd/Downloads/lidartest-20251121T154203Z-1-001/lidartest/src/bumperbot_ws`
**Last Updated**: 2025-11-25

This document contains comprehensive learnings from the bumperbot_ws workspace for implementing autonomous navigation on the wheelchair.

---

## 1. Architecture Overview

### Package Structure
```
bumperbot_ws/src/
├── bumperbot_bringup/        # Main launch files
├── bumperbot_controller/     # Teleop, joystick, twist_mux
├── bumperbot_description/    # URDF, meshes
├── bumperbot_firmware/       # Hardware interface (Arduino, IMU)
├── bumperbot_localization/   # EKF, AMCL configs
├── bumperbot_mapping/        # SLAM configs, saved maps
├── bumperbot_motion/         # Custom controllers (Pure Pursuit, PD)
├── bumperbot_navigation/     # Nav2 configs, behavior trees
├── bumperbot_planning/       # Custom planners (A*, Dijkstra)
├── bumperbot_utils/          # Safety utilities
└── bumperbot_msgs/           # Custom messages
```

### Key Launch Files
1. **real_robot.launch.py**: Main entry point for real hardware
   - Launches hardware interface, lidar, controller, joystick, IMU
   - Conditionally launches localization OR SLAM
   - Always launches navigation stack

---

## 2. Nav2 Navigation Stack Configuration

### Navigation Launch Pattern
File: `bumperbot_navigation/launch/navigation.launch.py`

**Lifecycle Nodes** (in correct startup order):
```python
lifecycle_nodes = [
    "controller_server",   # Local trajectory planning
    "planner_server",      # Global path planning
    "smoother_server",     # Path smoothing
    "behavior_server",     # Recovery behaviors
    "bt_navigator"         # Behavior tree executive
]
```

**Key Pattern**: Each node loads its own config file:
```python
nav2_controller_server = Node(
    package="nav2_controller",
    executable="controller_server",
    output="screen",
    parameters=[
        os.path.join(pkg, "config", "controller_server.yaml"),
        {"use_sim_time": use_sim_time}
    ],
)
```

### Lifecycle Manager Configuration
```python
nav2_lifecycle_manager = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="lifecycle_manager_navigation",
    output="screen",
    parameters=[
        {"node_names": lifecycle_nodes},
        {"use_sim_time": use_sim_time},
        {"autostart": True}  # Auto-activate nodes
    ],
)
```

---

## 3. Controller Server Configuration

File: `bumperbot_navigation/config/controller_server.yaml`

### Regulated Pure Pursuit Controller (RECOMMENDED)
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    odom_topic: /bumperbot_controller/odom  # CRITICAL: Match your odom topic

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.3
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.2
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 1.0
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
```

### Custom Controllers Available
1. **PD Motion Planner**: `bumperbot_motion::PDMotionPlanner`
   - kp: 2.0, kd: 0.1
   - max_linear_velocity: 0.3, max_angular_velocity: 1.0
   - step_size: 0.1

2. **Pure Pursuit**: `bumperbot_motion::PurePursuit`
   - look_ahead_distance: 0.5
   - max_linear_velocity: 0.3, max_angular_velocity: 1.0

---

## 4. Costmap Configuration

### Local Costmap (Rolling Window)
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.1  # Adjust for wheelchair!
      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          obstacle_max_range: 2.5

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

### Global Costmap (Static Map + Obstacles)
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.1
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /map
        map_subscribe_transient_local: True

      # Same obstacle/inflation as local
```

---

## 5. Planner Server Configuration

File: `bumperbot_navigation/config/planner_server.yaml`

### SMAC Planner 2D (RECOMMENDED)
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.125
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0
      cost_travel_multiplier: 2.0
      use_final_approach_orientation: false
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
```

### Custom Planners Available
1. **Dijkstra Planner**: `bumperbot_planning::DijkstraPlanner`
2. **A* Planner**: `bumperbot_planning::AStarPlanner`

---

## 6. Behavior Tree Configuration

### Simple Navigation with Replanning and Recovery
File: `bumperbot_navigation/behavior_tree/simple_navigation_w_replanning_and_recovery.xml`

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <RecoveryNode number_of_retries="1" name="NavigateRecovery">
            <PipelineSequence name="NavigateWithReplanning">
                <RateController hz="1.0">
                    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                </RateController>
                <FollowPath path="{path}" controller_id="FollowPath"/>
            </PipelineSequence>
            <Sequence>
                <Spin spin_dist="1.57"/>
                <Wait wait_duration="5.0"/>
                <BackUp backup_dist="0.30" backup_speed="0.15"/>
            </Sequence>
        </RecoveryNode>
    </BehaviorTree>
</root>
```

**Key Patterns**:
- `RateController hz="1.0"`: Replan every 1 second
- `RecoveryNode`: Try main action, if fails execute recovery sequence
- Recovery sequence: Spin → Wait → BackUp

### BT Navigator Config
```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /bumperbot_controller/odom
    transform_tolerance: 0.3
    default_server_timeout: 1000
    bt_loop_duration: 10
    default_nav_to_pose_bt_xml: "/path/to/behavior_tree.xml"
```

---

## 7. Behavior Server (Recovery Behaviors)

File: `bumperbot_navigation/config/behavior_server.yaml`

```yaml
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]

    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    wait:
      plugin: "nav2_behaviors::Wait"

    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

---

## 8. Localization Configuration

### AMCL Configuration
File: `bumperbot_localization/config/amcl.yaml`

```yaml
amcl:
  ros__parameters:
    alpha1: 0.05        # Rotation noise from rotation
    alpha2: 0.2         # Rotation noise from translation
    alpha3: 0.2         # Translation noise from translation
    alpha4: 0.05        # Translation noise from rotation
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    transform_tolerance: 1.0
    update_min_a: 0.2   # Rotation to trigger update
    update_min_d: 0.25  # Distance to trigger update
    scan_topic: scan
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

### EKF Configuration (robot_localization)
File: `bumperbot_localization/config/ekf.yaml`

**Key Settings**:
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint_ekf
    world_frame: odom

    # IMU config - use yaw, yaw velocity, x acceleration
    imu0: imu_ekf
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]
    imu0_differential: true

    # Odom config - use vx, vy only (not pose)
    odom0: bumperbot_controller/odom_noisy
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, false,
                   false, false, false]
    odom0_differential: true
```

---

## 9. Safety Features

### Safety Stop Node
File: `bumperbot_utils/bumperbot_utils/safety_stop.py`

```python
class SafetyStop(Node):
    def __init__(self):
        # Parameters
        self.warning_distance = 0.6  # Slow down
        self.danger_distance = 0.2   # Full stop

    def laser_callback(self, msg: LaserScan):
        for range_value in msg.ranges:
            if range_value <= self.warning_distance:
                self.state = State.WARNING
                self.decrease_speed_client.send_goal_async(...)
            if range_value <= self.danger_distance:
                self.state = State.DANGER
                self.safety_stop_pub.publish(True)
                break
```

---

## 10. Custom Planner Implementation (A*)

File: `bumperbot_planning/bumperbot_planning/a_star_planner.py`

### Key Functions:
```python
class AStarPlanner(Node):
    def plan(self, start: Pose, goal: Pose):
        # A* with manhattan distance heuristic
        start_node.heuristic = self.manhattan_distance(start_node, goal_node)

        while not pending_nodes.empty():
            active_node = pending_nodes.get()

            if active_node == goal_node:
                break

            for direction in [(-1,0), (1,0), (0,-1), (0,1)]:
                new_node = active_node + direction
                if self.pose_on_map(new_node) and cost < 99:
                    new_node.cost = active_node.cost + 1 + costmap_value
                    new_node.heuristic = manhattan_distance(new_node, goal)
                    pending_nodes.put(new_node)
```

### Registering Custom Plugins
File: `bumperbot_planning/global_planner_plugins.xml`
```xml
<library path="bumperbot_planning">
  <class name="bumperbot_planning::DijkstraPlanner"
         type="bumperbot_planning::DijkstraPlanner"
         base_class_type="nav2_core::GlobalPlanner">
    <description>Dijkstra global planner plugin</description>
  </class>
</library>
```

---

## 11. Custom Controller Implementation (Pure Pursuit)

File: `bumperbot_motion/bumperbot_motion/pure_pursuit.py`

### Key Algorithm:
```python
def control_loop(self):
    # 1. Get carrot pose (look-ahead point on path)
    carrot_pose = self.get_carrot_pose(robot_pose)

    # 2. Transform to robot frame
    carrot_pose_robot = transform_to_robot_frame(carrot_pose)

    # 3. Calculate curvature
    curvature = 2.0 * carrot_pose_robot.y / (x^2 + y^2)

    # 4. Apply velocity
    cmd_vel.linear.x = max_linear_velocity
    cmd_vel.angular.z = curvature * max_angular_velocity

def get_carrot_pose(self, robot_pose):
    # Find point on path at look_ahead_distance
    for pose in reversed(path.poses):
        if distance(pose, robot_pose) > look_ahead_distance:
            return pose
    return path.poses[-1]  # Goal if no point found
```

---

## 12. Critical Topics/TF for Navigation

### Required Topics:
| Topic | Type | Source |
|-------|------|--------|
| `/scan` | LaserScan | RPLidar |
| `/odom` or `/odometry/filtered` | Odometry | EKF |
| `/map` | OccupancyGrid | map_server |
| `/tf` | TF2 | robot_state_publisher + EKF + AMCL |
| `/cmd_vel` | Twist | Nav2 controller |

### Required TF Frames:
```
map → odom → base_link → [sensors]
         ↑
       (AMCL publishes this transform)
```

---

## 13. Wheelchair-Specific Recommendations

### Speed Limits (Passenger Comfort)
```yaml
# Controller
desired_linear_vel: 0.3-0.5 m/s  # Comfortable walking speed
max_angular_vel: 0.8 rad/s       # Gentle turns

# Velocity Smoother
max_velocity: [0.4, 0.0, 0.8]
max_accel: [0.3, 0.0, 1.5]
max_decel: [-0.4, 0.0, -2.0]
```

### Robot Radius
```yaml
robot_radius: 0.4-0.5  # Include wheelchair footprint
inflation_radius: 0.6-0.7  # Safe clearance
```

### Recovery Behaviors (Gentle)
```xml
<Spin spin_dist="1.0"/>  <!-- Less than full rotation -->
<BackUp backup_dist="0.15" backup_speed="0.10"/>  <!-- Slow, short -->
<Wait wait_duration="3.0"/>  <!-- Allow environment changes -->
```

---

## 14. Quick Debugging Commands

```bash
# Check Nav2 status
ros2 topic echo /bt_navigator/transition_event

# Check if costmaps are updating
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap

# Check TF tree
ros2 run tf2_tools view_frames

# Check lifecycle states
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

# Manually activate nodes
ros2 lifecycle set /controller_server activate

# Send test goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
```

---

## 15. Common Issues & Solutions

### Issue: "No valid path found"
- Check global costmap has static_layer with map
- Verify map → odom → base_link TF chain
- Increase `max_planning_time`

### Issue: "Transform timeout"
- Increase `transform_tolerance` in all configs
- Check TF publishers are running

### Issue: Robot oscillates
- Reduce `controller_frequency`
- Increase `lookahead_dist`
- Tune PID/Pure Pursuit gains

### Issue: Recovery behaviors not working
- Check behavior_server lifecycle is active
- Verify `costmap_topic` and `footprint_topic` match actual topics

---

## Summary: Key Takeaways

1. **Modular Config**: Separate YAML files per node for clarity
2. **Lifecycle Manager**: Handles startup order automatically
3. **Odom Topic**: MUST match your actual odometry source
4. **Robot Radius**: Critical for costmap inflation
5. **Behavior Tree**: Simple BT with RateController + RecoveryNode is robust
6. **Transform Tolerance**: Increase if seeing TF errors
7. **Wheelchair Safety**: Lower speeds, gentle recoveries, larger inflation

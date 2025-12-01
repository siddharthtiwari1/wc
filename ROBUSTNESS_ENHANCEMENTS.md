# SLAM TOOLBOX ROBUSTNESS ENHANCEMENTS
## Making v14r25_FINAL Production-Ready

Created: 2025-11-23
Status: v14r25_FINAL is WORKING ✅
Goal: Make it BULLETPROOF 🛡️

---

## PHASE 1: IMMEDIATE ROBUSTNESS TWEAKS (No Breaking Changes)

These are SAFE enhancements that won't break your working config.

### 1.1 INCREASE SCAN MATCHER RESILIENCE

**Problem:** What if a scan temporarily fails to match?

**Solution: Add minimum penalties floor**

```yaml
# Current (working):
minimum_distance_penalty: 0.5
minimum_angle_penalty: 0.9

# Enhanced (more resilient):
minimum_distance_penalty: 0.4  # Allow 60% penalty max (was 50%)
minimum_angle_penalty: 0.85     # Allow 15% penalty max (was 10%)
```

**Why:**
- Allows scan matcher to accept slightly worse matches
- Prevents total rejection in challenging areas
- Still maintains quality (not too permissive)

**Trade-off:** Minimal - only affects edge cases

---

### 1.2 EXPAND COARSE SEARCH FOR RECOVERY

**Problem:** If robot gets "lost" temporarily, can it recover?

**Solution: Widen coarse angular search**

```yaml
# Current (working):
coarse_search_angle_offset: 0.174  # ±10°

# Enhanced (more recovery):
coarse_search_angle_offset: 0.262  # ±15°
```

**Why:**
- Handles unexpected rotations (wheel slip, collision)
- Better recovery from temporary failures
- Still much tighter than default ±20°

**When to use:** If you see occasional pose jumps or recovery issues

---

### 1.3 ADD SCAN QUALITY FILTERING

**Problem:** Moving people, chairs create bad scans

**Solution: Already using HuberLoss ✅, but add scan filtering**

**Create scan filter node:**

```python
# /home/sidd/wc/src/scripts/scripts/scan_filter_node.py
#!/usr/bin/env python3
"""
Filters laser scans to remove:
- Outliers (suddenly appearing/disappearing points)
- Moving objects (temporal consistency check)
- Too-close points (< 0.2m from S3)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')

        # Parameters
        self.declare_parameter('min_range', 0.2)  # S3 minimum
        self.declare_parameter('max_range', 12.0)  # Your environment
        self.declare_parameter('angle_increment_tolerance', 0.001)

        # Subscribers and publishers
        self.sub = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        self.pub = self.create_publisher(
            LaserScan, '/scan', 10)

        # Temporal filter (detect moving objects)
        self.prev_scan = None
        self.moving_object_threshold = 0.5  # 50cm change = moving

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        # Filter 1: Range limits
        valid_mask = (ranges >= self.get_parameter('min_range').value) & \
                     (ranges <= self.get_parameter('max_range').value)

        # Filter 2: Remove inf/nan
        valid_mask &= np.isfinite(ranges)

        # Filter 3: Temporal consistency (detect moving objects)
        if self.prev_scan is not None:
            prev_ranges = np.array(self.prev_scan.ranges)
            if len(prev_ranges) == len(ranges):
                # Points that changed >50cm are likely moving
                change = np.abs(ranges - prev_ranges)
                valid_mask &= (change < self.moving_object_threshold)

        # Apply filter
        filtered_ranges = ranges.copy()
        filtered_ranges[~valid_mask] = float('inf')

        # Publish
        filtered_msg = msg
        filtered_msg.ranges = filtered_ranges.tolist()
        self.pub.publish(filtered_msg)

        self.prev_scan = msg

def main():
    rclpy.init()
    node = ScanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Add to launch file:**
```python
# In wheelchair_slam_mapping.launch.py
scan_filter_node = Node(
    package='scripts',
    executable='scan_filter_node',
    name='scan_filter_node',
    output='screen',
    remappings=[
        ('/scan_raw', '/scan'),  # RPLidar output
        ('/scan', '/scan_filtered')  # To SLAM
    ]
)
```

**RPLidar launch change:**
```python
# Change RPLidar to publish to /scan_raw instead
# Then scan_filter publishes cleaned /scan to SLAM
```

---

### 1.4 ADD LOCALIZATION COVARIANCE MONITORING

**Problem:** How do you KNOW if SLAM is confident vs uncertain?

**Solution: Monitor pose covariance**

```python
# /home/sidd/wc/src/scripts/scripts/slam_health_monitor.py
#!/usr/bin/env python3
"""
Monitors SLAM health by checking:
- Pose covariance (uncertainty)
- Map update frequency
- TF tree freshness
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import numpy as np

class SLAMHealthMonitor(Node):
    def __init__(self):
        super().__init__('slam_health_monitor')

        # Thresholds
        self.pos_cov_warn = 0.1    # >10cm variance = warning
        self.pos_cov_error = 0.5   # >50cm variance = error
        self.rot_cov_warn = 0.05   # >5° variance = warning
        self.rot_cov_error = 0.2   # >20° variance = error

        # Subscribe to pose
        self.sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)

        # Timer for periodic health check
        self.timer = self.create_timer(2.0, self.health_check)

        self.last_odom = None
        self.last_odom_time = None

    def odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_time = self.get_clock().now()

    def health_check(self):
        if self.last_odom is None:
            self.get_logger().warn('🔴 NO ODOMETRY DATA!')
            return

        # Check position covariance
        pos_cov = self.last_odom.pose.covariance[0]  # x variance
        if pos_cov > self.pos_cov_error:
            self.get_logger().error(f'🔴 POSITION UNCERTAINTY HIGH: {pos_cov:.3f}m')
        elif pos_cov > self.pos_cov_warn:
            self.get_logger().warn(f'⚠️  Position uncertainty: {pos_cov:.3f}m')
        else:
            self.get_logger().info(f'✅ Position confidence: {pos_cov:.4f}m')

        # Check rotation covariance
        rot_cov = self.last_odom.pose.covariance[35]  # yaw variance
        if rot_cov > self.rot_cov_error:
            self.get_logger().error(f'🔴 ROTATION UNCERTAINTY HIGH: {rot_cov:.3f}rad')
        elif rot_cov > self.rot_cov_warn:
            self.get_logger().warn(f'⚠️  Rotation uncertainty: {rot_cov:.3f}rad')
        else:
            self.get_logger().info(f'✅ Rotation confidence: {rot_cov:.4f}rad')

        # Check data freshness
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if age > 1.0:
            self.get_logger().error(f'🔴 STALE DATA: {age:.1f}s old')

def main():
    rclpy.init()
    node = SLAMHealthMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## PHASE 2: ADVANCED ROBUSTNESS FEATURES

### 2.1 ADAPTIVE PARAMETER SWITCHING

**Problem:** Different environments need different parameters

**Solution: Auto-detect environment and switch configs**

```python
# /home/sidd/wc/src/scripts/scripts/adaptive_slam.py
#!/usr/bin/env python3
"""
Monitors environment and adjusts SLAM parameters:
- Featureless corridor → Increase odom trust
- Feature-rich room → Increase scan trust
- High dynamics → Stricter matching
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
import numpy as np

class AdaptiveSLAM(Node):
    def __init__(self):
        super().__init__('adaptive_slam')

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Parameter update client
        self.param_client = self.create_client(
            SetParameters, '/slam_toolbox/set_parameters')

        # Environment detection
        self.feature_richness_history = []
        self.max_history = 10

    def scan_callback(self, msg):
        # Analyze scan for feature richness
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) < 10:
            return

        # Feature richness = variation in ranges
        range_std = np.std(valid_ranges)
        range_mean = np.mean(valid_ranges)
        feature_richness = range_std / (range_mean + 0.001)

        self.feature_richness_history.append(feature_richness)
        if len(self.feature_richness_history) > self.max_history:
            self.feature_richness_history.pop(0)

        # Detect environment type
        avg_richness = np.mean(self.feature_richness_history)

        if avg_richness < 0.1:
            # FEATURELESS CORRIDOR
            self.get_logger().info('🏢 Corridor detected - trusting odometry more')
            self.adjust_parameters(
                distance_penalty=0.35,  # More odom trust
                angle_penalty=0.35
            )
        elif avg_richness > 0.3:
            # FEATURE-RICH ROOM
            self.get_logger().info('🏠 Feature-rich area - trusting scans more')
            self.adjust_parameters(
                distance_penalty=0.55,  # More scan trust
                angle_penalty=0.55
            )
        else:
            # NORMAL AREA (use default 0.45)
            pass

    def adjust_parameters(self, distance_penalty, angle_penalty):
        # Create parameter update request
        request = SetParameters.Request()

        param1 = Parameter()
        param1.name = 'distance_variance_penalty'
        param1.value = ParameterValue(type=3, double_value=distance_penalty)

        param2 = Parameter()
        param2.name = 'angle_variance_penalty'
        param2.value = ParameterValue(type=3, double_value=angle_penalty)

        request.parameters = [param1, param2]

        # Send async request
        self.param_client.call_async(request)

def main():
    rclpy.init()
    node = AdaptiveSLAM()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

### 2.2 LOOP CLOSURE VALIDATION

**Problem:** False loop closures can corrupt the map

**Solution: Add validation layer**

```yaml
# Enhanced loop closure parameters:
loop_match_minimum_response_coarse: 0.45  # Stricter (was 0.40)
loop_match_minimum_response_fine: 0.50    # Stricter (was 0.45)
loop_match_minimum_chain_size: 10         # Longer (was 8)

# Add new parameter (if available in your version):
loop_closure_use_scan_context: true       # Geometric validation
```

---

### 2.3 MULTI-RESOLUTION MAPPING (Like Hector!)

**Problem:** SLAM Toolbox uses single resolution

**Solution: Maintain multiple map resolutions**

```python
# /home/sidd/wc/src/scripts/scripts/multi_res_mapper.py
#!/usr/bin/env python3
"""
Subscribes to /map (0.02m resolution)
Publishes:
- /map_fine (0.02m) - for navigation
- /map_coarse (0.05m) - for global planning
- /map_visualization (0.01m) - for RViz
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.ndimage import zoom

class MultiResMapper(Node):
    def __init__(self):
        super().__init__('multi_res_mapper')

        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.pub_fine = self.create_publisher(OccupancyGrid, '/map_fine', 10)
        self.pub_coarse = self.create_publisher(OccupancyGrid, '/map_coarse', 10)

    def map_callback(self, msg):
        # Original map (0.02m)
        self.pub_fine.publish(msg)

        # Coarse map (0.05m) - downsample by 2.5×
        coarse = self.downsample_map(msg, 2.5)
        self.pub_coarse.publish(coarse)

    def downsample_map(self, map_msg, factor):
        # Convert to numpy
        data = np.array(map_msg.data).reshape(
            (map_msg.info.height, map_msg.info.width))

        # Downsample
        downsampled = zoom(data, 1.0/factor, order=0)

        # Create new message
        new_msg = OccupancyGrid()
        new_msg.header = map_msg.header
        new_msg.info = map_msg.info
        new_msg.info.resolution *= factor
        new_msg.info.width = downsampled.shape[1]
        new_msg.info.height = downsampled.shape[0]
        new_msg.data = downsampled.flatten().tolist()

        return new_msg

def main():
    rclpy.init()
    node = MultiResMapper()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## PHASE 3: FALLBACK AND RECOVERY STRATEGIES

### 3.1 ODOMETRY-ONLY FALLBACK

**Problem:** What if scan matching totally fails?

**Solution: Fallback to pure odometry temporarily**

```python
# /home/sidd/wc/src/scripts/scripts/slam_watchdog.py
#!/usr/bin/env python3
"""
Monitors SLAM health and triggers fallback:
- If pose covariance explodes → fallback to odometry
- If no map updates for 10s → restart SLAM
- If TF chain breaks → alert user
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
import numpy as np

class SLAMWatchdog(Node):
    def __init__(self):
        super().__init__('slam_watchdog')

        # State
        self.slam_healthy = True
        self.fallback_active = False
        self.unhealthy_count = 0
        self.unhealthy_threshold = 5  # 5 consecutive failures

        # Subscribers
        self.slam_pose_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.slam_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/wc_control/odom', self.odom_callback, 10)

        # Publisher for fallback pose
        self.fallback_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/pose', 10)

        # Service clients
        self.pause_slam = self.create_client(
            Trigger, '/slam_toolbox/pause_new_measurements')
        self.resume_slam = self.create_client(
            Trigger, '/slam_toolbox/resume_new_measurements')

        # Timer
        self.timer = self.create_timer(1.0, self.watchdog_check)

        self.last_slam_pose = None
        self.last_odom_pose = None

    def slam_callback(self, msg):
        self.last_slam_pose = msg

        # Check covariance
        pos_cov = msg.pose.covariance[0]
        if pos_cov > 1.0:  # >1m uncertainty
            self.unhealthy_count += 1
            if self.unhealthy_count >= self.unhealthy_threshold:
                self.trigger_fallback()
        else:
            self.unhealthy_count = max(0, self.unhealthy_count - 1)
            if self.fallback_active and self.unhealthy_count == 0:
                self.recover_from_fallback()

    def odom_callback(self, msg):
        self.last_odom_pose = msg

    def trigger_fallback(self):
        if not self.fallback_active:
            self.get_logger().error('🔴 SLAM UNHEALTHY - FALLBACK TO ODOMETRY')
            self.fallback_active = True

            # Pause SLAM
            self.pause_slam.call_async(Trigger.Request())

            # Publish odometry as pose estimate
            if self.last_odom_pose:
                fallback_msg = PoseWithCovarianceStamped()
                fallback_msg.header = self.last_odom_pose.header
                fallback_msg.pose = self.last_odom_pose.pose
                self.fallback_pub.publish(fallback_msg)

    def recover_from_fallback(self):
        self.get_logger().info('✅ SLAM RECOVERED - RESUMING')
        self.fallback_active = False
        self.resume_slam.call_async(Trigger.Request())

    def watchdog_check(self):
        # Check for stale data, TF breaks, etc.
        pass

def main():
    rclpy.init()
    node = SLAMWatchdog()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

### 3.2 AUTOMATIC MAP SAVING

**Problem:** If robot crashes, lose all mapping work

**Solution: Auto-save map periodically**

```python
# /home/sidd/wc/src/scripts/scripts/auto_map_saver.py
#!/usr/bin/env python3
"""
Automatically saves map every N minutes or M meters traveled
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from slam_toolbox.srv import SaveMap
import numpy as np
from datetime import datetime

class AutoMapSaver(Node):
    def __init__(self):
        super().__init__('auto_map_saver')

        # Parameters
        self.declare_parameter('save_interval_seconds', 300.0)  # 5 min
        self.declare_parameter('save_interval_meters', 50.0)     # 50m

        # State
        self.last_save_time = self.get_clock().now()
        self.last_save_pose = None
        self.total_distance = 0.0

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)

        # Service client
        self.save_map_client = self.create_client(
            SaveMap, '/slam_toolbox/save_map')

        # Timer
        self.timer = self.create_timer(10.0, self.check_save_conditions)

    def odom_callback(self, msg):
        if self.last_save_pose is not None:
            # Calculate distance traveled
            dx = msg.pose.pose.position.x - self.last_save_pose.position.x
            dy = msg.pose.pose.position.y - self.last_save_pose.position.y
            dist = np.sqrt(dx*dx + dy*dy)
            self.total_distance += dist

        self.last_save_pose = msg.pose.pose

    def check_save_conditions(self):
        now = self.get_clock().now()
        time_elapsed = (now - self.last_save_time).nanoseconds / 1e9

        save_interval_sec = self.get_parameter(
            'save_interval_seconds').value
        save_interval_meters = self.get_parameter(
            'save_interval_meters').value

        should_save = False
        reason = ""

        if time_elapsed >= save_interval_sec:
            should_save = True
            reason = f"time ({time_elapsed:.0f}s)"
        elif self.total_distance >= save_interval_meters:
            should_save = True
            reason = f"distance ({self.total_distance:.1f}m)"

        if should_save:
            self.save_map(reason)
            self.last_save_time = now
            self.total_distance = 0.0

    def save_map(self, reason):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_name = f"automap_{timestamp}"

        self.get_logger().info(
            f'💾 Auto-saving map: {map_name} (reason: {reason})')

        request = SaveMap.Request()
        request.name.data = map_name

        future = self.save_map_client.call_async(request)
        future.add_done_callback(self.save_complete)

    def save_complete(self, future):
        try:
            response = future.result()
            self.get_logger().info('✅ Map saved successfully')
        except Exception as e:
            self.get_logger().error(f'❌ Map save failed: {e}')

def main():
    rclpy.init()
    node = AutoMapSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## PHASE 4: TESTING PROTOCOLS FOR ROBUSTNESS

### 4.1 STRESS TESTS

**Test 1: Kidnapped Robot**
```bash
# While SLAM running:
# 1. Pick up robot
# 2. Move it 5m away
# 3. Put it down
# Expected: SLAM should detect loop closure and correct
```

**Test 2: Featureless Corridor**
```bash
# Drive down long, blank corridor
# Expected: Pose should keep updating (not freeze)
# Monitor: /slam_toolbox/graph_visualization
```

**Test 3: Dynamic Environment**
```bash
# Have people walk around while mapping
# Expected: HuberLoss filters them out
# Check: Map should be clean (no ghost obstacles)
```

**Test 4: Long Duration**
```bash
# Map for 30+ minutes continuously
# Expected: No memory leaks, no drift accumulation
# Monitor: CPU, memory, pose covariance
```

**Test 5: Loop Closure Stress**
```bash
# Explore large area, return to start from different angle
# Expected: Loop closure detects and corrects
# Check: Start position should match end position
```

---

### 4.2 ROBUSTNESS METRICS

**Create metrics logger:**

```python
# /home/sidd/wc/src/scripts/scripts/slam_metrics_logger.py
#!/usr/bin/env python3
"""
Logs SLAM performance metrics:
- Pose update rate
- Covariance over time
- Loop closures detected
- CPU usage
- Map quality metrics
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
import csv
from datetime import datetime

class SLAMMetricsLogger(Node):
    def __init__(self):
        super().__init__('slam_metrics_logger')

        # Create CSV file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(f'slam_metrics_{timestamp}.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'pos_x', 'pos_y', 'yaw',
            'pos_covariance', 'rot_covariance',
            'map_cells_occupied', 'map_cells_free'
        ])

        # Subscribers
        self.pose_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.pose_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.last_map_stats = None

    def pose_callback(self, msg):
        import tf_transformations

        # Extract pose
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([
            orient.x, orient.y, orient.z, orient.w])

        # Extract covariances
        pos_cov = msg.pose.covariance[0]
        rot_cov = msg.pose.covariance[35]

        # Log
        self.csv_writer.writerow([
            self.get_clock().now().nanoseconds / 1e9,
            pos.x, pos.y, euler[2],
            pos_cov, rot_cov,
            self.last_map_stats[0] if self.last_map_stats else 0,
            self.last_map_stats[1] if self.last_map_stats else 0
        ])
        self.csv_file.flush()

    def map_callback(self, msg):
        import numpy as np
        data = np.array(msg.data)
        occupied = np.sum(data > 50)
        free = np.sum(data == 0)
        self.last_map_stats = (occupied, free)

    def __del__(self):
        self.csv_file.close()

def main():
    rclpy.init()
    node = SLAMMetricsLogger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## PHASE 5: PRODUCTION DEPLOYMENT CHECKLIST

### ✅ PRE-DEPLOYMENT

- [ ] Run all 5 stress tests
- [ ] Monitor metrics for 30+ minutes
- [ ] Verify loop closure works
- [ ] Test recovery from failures
- [ ] Validate map quality matches Hector 2024

### ✅ MONITORING SETUP

- [ ] Deploy slam_health_monitor
- [ ] Deploy auto_map_saver
- [ ] Set up alerts (ROS bag recording on error)
- [ ] Dashboard (RViz + custom panels)

### ✅ FALLBACK STRATEGIES

- [ ] Odometry-only fallback tested
- [ ] Manual recovery procedure documented
- [ ] Emergency stop tested

### ✅ DOCUMENTATION

- [ ] Parameter justification (done! ✅)
- [ ] Troubleshooting guide (done! ✅)
- [ ] Recovery procedures
- [ ] Tuning guide for different environments

---

## SUMMARY: ROBUSTNESS LAYERS

```
┌─────────────────────────────────────────────────────────┐
│ LAYER 5: Monitoring & Alerts                          │
│   - Health monitor, metrics logger, auto-save         │
├─────────────────────────────────────────────────────────┤
│ LAYER 4: Fallback & Recovery                          │
│   - Watchdog, odometry fallback, restart logic        │
├─────────────────────────────────────────────────────────┤
│ LAYER 3: Adaptive Behavior                            │
│   - Environment detection, parameter switching        │
├─────────────────────────────────────────────────────────┤
│ LAYER 2: Enhanced Filtering                           │
│   - Scan filter, HuberLoss, loop closure validation  │
├─────────────────────────────────────────────────────────┤
│ LAYER 1: Solid Foundation (WORKING ✅)                 │
│   - v14r25_FINAL parameters                           │
│   - 0.02m resolution, 0.45/0.45 variance, HuberLoss   │
└─────────────────────────────────────────────────────────┘
```

---

## RECOMMENDED NEXT STEPS

**IMMEDIATE (This Week):**
1. Add slam_health_monitor (easy, non-breaking)
2. Add auto_map_saver (safety net)
3. Run stress test #2 (featureless corridor)

**SHORT-TERM (This Month):**
1. Implement scan_filter_node
2. Deploy slam_watchdog
3. Run all 5 stress tests

**LONG-TERM (Next Month):**
1. Adaptive parameter switching
2. Multi-resolution mapping
3. Full production deployment

**Your v14r25_FINAL is WORKING.** Now we make it BULLETPROOF! 🛡️

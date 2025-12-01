# Deep Workspace Analysis: Visual-LiDAR Fusion for Wheelchair Navigation

**Author**: Siddharth Tiwari (s24035@students.iitmandi.ac.in)
**Analysis Date**: 2025-12-01
**Target**: ICRA/IROS Publication-Worthy System

---

## Executive Summary

This analysis evaluates the current wheelchair navigation workspace for developing a **publication-worthy visual-LiDAR fusion system** targeting ICRA/IROS. The system combines:
- **RPLidar S3** (2D 360° LiDAR, 40m range, 32kHz)
- **Intel RealSense D455** (Depth camera + RGB + IMU)

### Current State Assessment

| Component | Status | Publication Readiness |
|-----------|--------|----------------------|
| EKF Localization | **Excellent** | Ready |
| AMCL Global Localization | **Good** | Needs tuning |
| SLAM (slam_toolbox) | **Good** | Ready |
| Laser Filtering | **Basic** | Needs enhancement |
| Sensor Fusion Node | **Exists** | Major gaps |
| Nav2 Navigation | **Configured** | Not functional |
| Depth Camera in Costmaps | **Missing** | Critical gap |
| Pointcloud Processing | **Missing** | Critical gap |

---

## Part 1: Current Architecture Analysis

### 1.1 Launch File Architecture

```
wheelchair_slam_mapping.launch.py
├── unified_wheelchair.launch.py (hardware + ros2_control)
├── wheelchair_sensors.launch.py (RealSense + IMU filter)
├── rplidar_s3_launch.py
├── laser_filter_node (/scan → /scan_filtered)
├── ekf_local_node (wheel odom + IMU → /odometry/filtered)
├── ekf_global_node (local + SLAM → /odometry/global)
├── slam_toolbox_node
└── map_saver_server

wheelchair_global_localization.launch.py
├── unified_wheelchair.launch.py
├── wheelchair_sensors.launch.py
├── rplidar_s3_launch.py
├── ekf_local_node
├── map_server (loads pre-built map)
├── AMCL (global particle filter)
└── twist_to_stamped_converter

wheelchair_autonomous_nav.launch.py
├── wheelchair_global_localization.launch.py (includes above)
└── Nav2 Stack:
    ├── controller_server (Regulated Pure Pursuit)
    ├── planner_server (NavFn)
    ├── behavior_server (spin, backup, wait)
    ├── bt_navigator
    ├── smoother_server
    ├── velocity_smoother
    └── waypoint_follower
```

### 1.2 EKF Configuration (EXCELLENT)

**File**: `src/wheelchair_localization/config/ekf.yaml`

**Strengths**:
- Clean separation: Wheel odom provides X, Y position + X velocity
- IMU provides ONLY yaw + yaw velocity (no noisy acceleration)
- 30Hz frequency, 2D mode enabled
- Well-tuned process noise covariance

**Key Config**:
```yaml
odom0_config: [true, true, false,    # x, y from wheels
               false, false, false,   # NO yaw from wheels
               true, false, false,    # vx from wheels
               ...]

imu0_config: [false, false, false,   # no position from IMU
              false, false, true,    # ONLY yaw
              false, false, false,
              false, false, true,    # ONLY yaw_vel
              false, false, false]   # NO acceleration
```

### 1.3 AMCL Configuration (GOOD, needs tuning)

**File**: `src/wheelchair_localization/config/amcl_v2.yaml`

**Strengths**:
- Up to 10,000 particles (leveraging i5 13th Gen)
- Tight update thresholds (1cm, 2°)
- High S3 utilization (240 beams from 3200 points)
- Aggressive recovery (alpha_fast: 0.15)

**Identified Issues**:
- `laser_sigma_hit: 0.08` may be too tight causing jitter
- No integration with depth camera for better localization

### 1.4 Laser Filter (BASIC)

**File**: `src/wheelchair_localization/config/laser_filter.yaml`

**Current Filters**:
1. Range Filter (0.15m - 15m)
2. Speckle Filter (3 neighbors, 15cm threshold)

**Missing**:
- Temporal median filter
- Angular filter (remove low-angle readings)
- Intensity-based filtering
- Shadow filter (robot body occlusion)

### 1.5 Sensor Fusion Package (EXISTS but INCOMPLETE)

**Location**: `src/wheelchair_sensor_fusion/`

**Existing Nodes**:
1. `lidar_processor_node.py` - DBSCAN clustering on 2D scan
2. `sensor_fusion_node.py` - LiDAR-YOLO fusion
3. `yolo_detector_node.py` - Object detection
4. `obstacle_publisher_node.py`

**Critical Issues**:
- Fusion is semantic only (YOLO + LiDAR clusters)
- **NO integration with Nav2 costmaps**
- **Depth camera NOT used for obstacle detection**
- Association uses simple greedy matching, not Hungarian algorithm
- No temporal tracking/persistence

### 1.6 Nav2 Configuration (CONFIGURED but NOT FUNCTIONAL)

**File**: `src/wheelchair_navigation/config/nav2_params_wheelchair.yaml`

**Costmap Configuration**:
```yaml
local_costmap:
  plugins: ["obstacle_layer", "inflation_layer"]
  obstacle_layer:
    observation_sources: scan  # ONLY LiDAR!
    scan:
      topic: /scan
      data_type: "LaserScan"
```

**CRITICAL GAP**: Depth camera NOT in costmaps!

---

## Part 2: State-of-the-Art Research (ICRA/IROS 2024-2025)

### 2.1 Relevant Papers

| Paper | Conference | Key Contribution |
|-------|------------|------------------|
| Fuzzy Logic Based Fusion of 2D LIDAR and Depth Camera | ASME IMECE 2022 | Fuzzy adaptive weighting |
| DRL-Based UAV Navigation with LiDAR-Depth Fusion | MDPI 2025 | Deep RL for fusion |
| Multi-Sensor Fusion Perception | IJITSR 2024 | Adaptive fusion algorithms |
| STVL: Spatio-Temporal Voxel Layer | IJRR 2020 | 3D temporal voxel grids |
| LIO-SAM | IROS 2020 | Factor graph LiDAR-IMU |

### 2.2 Key Insights from Research

1. **Adaptive Weighting** (Multi-Sensor Fusion 2024):
   - Measurement accuracy improved by 0.61% over single camera
   - Safe stopping distance maintained at ±0.7m

2. **Fuzzy Logic Fusion** (ASME 2022):
   - Higher depth confidence than individual sensors
   - Smart gap correction between sensors

3. **STVL** (Steve Macenski):
   - 2x reduction in resource utilization
   - Temporal decay for dynamic environments
   - Direct Nav2 integration

### 2.3 Reference Implementations

| Repository | Description | Stars |
|------------|-------------|-------|
| [spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) | Nav2 3D voxel plugin | 500+ |
| [ros2_navigation_stvl](https://github.com/mich-pest/ros2_navigation_stvl) | STVL example | - |
| [awesome-LiDAR-Visual-SLAM](https://github.com/sjtuyinjie/awesome-LiDAR-Visual-SLAM) | Curated resources | 1000+ |

---

## Part 3: Gap Analysis & Publication Opportunities

### 3.1 Critical Gaps

| Gap | Impact | Priority |
|-----|--------|----------|
| Depth camera not in costmaps | Navigation blind to frontal obstacles | **P0** |
| No STVL integration | No 3D perception | **P0** |
| Fusion not connected to Nav2 | Fusion results unused | **P0** |
| No pointcloud filtering | Noisy depth data | **P1** |
| No temporal obstacle tracking | Phantom obstacles | **P1** |
| Basic laser filter | Clutter in map | **P2** |

### 3.2 Publication-Worthy Contributions

**Proposed Title**: *"Adaptive Visual-LiDAR Fusion for Robust Wheelchair Navigation in Dynamic Indoor Environments"*

**Novel Contributions**:

1. **Adaptive Confidence-Weighted Fusion (ACWF)**
   - Distance-based sensor reliability weighting
   - Lighting-adaptive camera confidence
   - Motion-based LiDAR confidence

2. **Dual-Layer Costmap Architecture**
   - 2D LiDAR for global planning (efficient)
   - 3D STVL for local obstacle avoidance (accurate)
   - Cross-validation between layers

3. **Temporal Obstacle Persistence Model**
   - Track obstacle lifetime
   - Differentiate static vs dynamic
   - Predict obstacle motion

4. **Wheelchair-Specific Safety Layer**
   - Passenger comfort constraints
   - Stability-aware planning
   - Emergency stop with depth verification

---

## Part 4: Recommended Architecture

### 4.1 Enhanced System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ENHANCED WHEELCHAIR NAVIGATION                       │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
┌───────▼───────┐         ┌─────────▼─────────┐       ┌─────────▼─────────┐
│    SENSORS    │         │   PREPROCESSING   │       │     FUSION        │
└───────┬───────┘         └─────────┬─────────┘       └─────────┬─────────┘
        │                           │                           │
┌───────┴───────┐         ┌─────────┴─────────┐       ┌─────────┴─────────┐
│ RPLidar S3    │────────▶│ Enhanced Laser    │       │ Adaptive Sensor   │
│ /scan         │         │ Filter Chain      │       │ Fusion Node       │
├───────────────┤         │ • Range           │       │ • ACWF algorithm  │
│ RealSense D455│         │ • Speckle         │       │ • Temporal track  │
│ /camera/depth │────────▶│ • Shadow          │       │ • Confidence      │
│ /camera/color │         │ • Temporal median │       └─────────┬─────────┘
├───────────────┤         └─────────┬─────────┘                 │
│ IMU           │                   │                           │
│ /imu          │                   │                           │
└───────────────┘                   │                           │
        │                           │                           │
        │                  ┌────────▼────────┐                  │
        │                  │   LOCALIZATION  │                  │
        │                  └────────┬────────┘                  │
        │                           │                           │
        │                  ┌────────┴────────┐                  │
        └─────────────────▶│ EKF (30Hz)      │                  │
                           │ wheel+IMU       │                  │
                           ├─────────────────┤                  │
                           │ AMCL (10kP)     │                  │
                           │ map→odom TF     │                  │
                           └────────┬────────┘                  │
                                    │                           │
                           ┌────────▼────────────────────────────▼────────┐
                           │              NAV2 COSTMAPS                    │
                           └────────┬────────────────────────────┬────────┘
                                    │                            │
                           ┌────────▼────────┐          ┌────────▼────────┐
                           │ GLOBAL COSTMAP  │          │ LOCAL COSTMAP   │
                           │ • static_layer  │          │ • STVL layer    │◀── NEW!
                           │ • obstacle_layer│          │ • fusion_layer  │◀── NEW!
                           │ • inflation     │          │ • inflation     │
                           └────────┬────────┘          └────────┬────────┘
                                    │                            │
                           ┌────────▼────────────────────────────▼────────┐
                           │              NAV2 NAVIGATION                  │
                           └────────┬────────────────────────────┬────────┘
                                    │                            │
                           ┌────────▼────────┐          ┌────────▼────────┐
                           │ SMAC Planner    │          │ Regulated PP    │
                           │ Global Path     │          │ Local Control   │
                           └────────┬────────┘          └────────┬────────┘
                                    │                            │
                           ┌────────▼────────────────────────────▼────────┐
                           │            SAFETY LAYER (NEW!)                │
                           │ • Depth verification before motion            │
                           │ • Emergency stop with sensor cross-check      │
                           │ • Passenger comfort constraints               │
                           └──────────────────────────────────────────────┘
```

### 4.2 New Components Required

#### 4.2.1 Enhanced Laser Filter Chain
```yaml
# laser_filter_enhanced.yaml
scan_to_scan_filter_chain:
  ros__parameters:
    filter1:  # Range
      type: laser_filters/LaserScanRangeFilter
      params:
        lower_threshold: 0.15
        upper_threshold: 15.0

    filter2:  # Shadow (remove robot body reflections)
      type: laser_filters/ScanShadowsFilter
      params:
        min_angle: 10
        max_angle: 170
        neighbors: 2
        window: 1

    filter3:  # Angular (remove floor/ceiling)
      type: laser_filters/LaserScanAngularBoundsFilter
      params:
        lower_angle: -2.8  # -160°
        upper_angle: 2.8   # +160°

    filter4:  # Temporal Median (noise reduction)
      type: laser_filters/LaserScanMedianFilter
      params:
        window_size: 3

    filter5:  # Speckle (isolated points)
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 0
        max_range_difference: 0.10
        filter_window: 5
```

#### 4.2.2 STVL Integration for Depth Camera
```yaml
# nav2_params_fusion.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["stvl_layer", "obstacle_layer", "inflation_layer"]

      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        voxel_decay: 5.0          # seconds
        decay_model: 0            # linear decay
        voxel_size: 0.05          # 5cm voxels
        track_unknown_space: true
        mark_threshold: 0
        update_footprint_enabled: true
        combination_method: 1     # max
        origin_z: 0.0
        publish_voxel_map: true
        transform_tolerance: 0.2
        mapping_mode: false
        map_save_duration: 60.0
        observation_sources: depth_camera

        depth_camera:
          topic: /camera/depth/color/points  # PointCloud2
          data_type: PointCloud2
          sensor_frame: camera_depth_optical_frame
          clearing: true
          marking: true
          max_z: 1.8              # Above wheelchair user height
          min_z: 0.1              # Floor filter
          vertical_fov_angle: 0.8 # D455 FOV
          horizontal_fov_angle: 1.5
          decay_acceleration: 1.0
          model_type: 0           # depth camera model
```

#### 4.2.3 Adaptive Confidence-Weighted Fusion Node

```python
# adaptive_fusion_node.py (conceptual)
class AdaptiveFusionNode(Node):
    """
    Novel Adaptive Confidence-Weighted Fusion (ACWF) Algorithm

    Key innovations:
    1. Distance-based reliability: LiDAR more reliable far, depth camera near
    2. Lighting-adaptive: Reduce camera weight in low light
    3. Motion-adaptive: Increase LiDAR weight during fast motion
    4. Temporal persistence: Track obstacles over time
    """

    def compute_adaptive_weights(self, distance, light_level, velocity):
        # Sigmoid-based distance weighting
        dist_factor = 1.0 / (1.0 + exp(-k * (distance - threshold)))

        # Lighting adaptive (from camera brightness)
        light_factor = min(1.0, light_level / 128.0)

        # Motion adaptive (reduce camera weight at high speed)
        motion_factor = 1.0 - min(1.0, velocity / max_velocity)

        w_lidar = 0.3 + 0.7 * dist_factor
        w_camera = 0.3 + 0.7 * (1.0 - dist_factor) * light_factor * motion_factor

        return normalize(w_lidar, w_camera)
```

#### 4.2.4 Safety Verification Layer

```python
# safety_verification_node.py (conceptual)
class SafetyVerificationNode(Node):
    """
    Cross-modal safety verification before motion execution

    Key features:
    1. Verify depth camera agrees with LiDAR before forward motion
    2. Emergency stop if sensors disagree on obstacle presence
    3. Passenger comfort: limit jerk, prevent sudden stops
    """

    def verify_motion_safety(self, cmd_vel):
        if cmd_vel.linear.x > 0:  # Forward motion
            lidar_clear = self.check_lidar_clear(self.frontal_arc)
            depth_clear = self.check_depth_clear(self.frontal_region)

            if lidar_clear and not depth_clear:
                # Potential low obstacle (table, knee-height)
                return self.slow_and_verify()

            if not lidar_clear and depth_clear:
                # Potential glass/transparent obstacle
                return self.emergency_stop()

        return self.apply_comfort_limits(cmd_vel)
```

---

## Part 5: Implementation Roadmap

### Phase 1: Foundation (Week 1-2)
**Goal**: Get depth camera into costmaps

1. Install STVL: `sudo apt install ros-jazzy-spatio-temporal-voxel-layer`
2. Configure depth-to-pointcloud conversion
3. Add STVL to local costmap
4. Test obstacle detection with depth camera
5. Verify costmap updates in RViz

### Phase 2: Enhanced Filtering (Week 2-3)
**Goal**: Clean sensor data

1. Implement enhanced laser filter chain
2. Add pointcloud voxel filter for depth data
3. Implement temporal median filter
4. Test filter chain with different environments

### Phase 3: Adaptive Fusion (Week 3-5)
**Goal**: Publication-worthy fusion algorithm

1. Implement ACWF algorithm
2. Create temporal obstacle tracking
3. Implement cross-modal verification
4. Collect dataset for validation

### Phase 4: Navigation Integration (Week 5-6)
**Goal**: Functional autonomous navigation

1. Test navigation with fused costmaps
2. Tune Regulated Pure Pursuit for wheelchair
3. Test behavior tree recovery
4. End-to-end validation

### Phase 5: Publication Preparation (Week 6-8)
**Goal**: ICRA/IROS submission

1. Quantitative evaluation (localization error, obstacle detection rate)
2. Comparison with baseline (LiDAR-only, depth-only)
3. Real-world experiments
4. Paper writing

---

## Part 6: Immediate Action Items

### P0 - Critical (Do First)

1. **Add depth camera pointcloud to costmap**
   ```bash
   # Install STVL
   sudo apt install ros-jazzy-spatio-temporal-voxel-layer
   ```

   Modify `nav2_params_wheelchair.yaml` to include STVL layer

2. **Create depth-to-pointcloud launch**
   - RealSense already publishes `/camera/depth/color/points`
   - Configure STVL to subscribe

3. **Test basic navigation**
   - Fix any TF issues
   - Verify costmaps update
   - Send test goal

### P1 - Important (Do Second)

4. **Enhance laser filter chain**
   - Add shadow filter
   - Add temporal median

5. **Connect fusion node to Nav2**
   - Publish fused obstacles to costmap layer
   - Or create custom costmap plugin

### P2 - Enhancement (Do Third)

6. **Implement ACWF algorithm**
7. **Add safety verification layer**
8. **Create evaluation framework**

---

## Part 7: Key Configuration Files to Create/Modify

### Files to Create

| File | Purpose |
|------|---------|
| `laser_filter_enhanced.yaml` | Multi-stage laser filtering |
| `nav2_params_fusion.yaml` | Nav2 with STVL + depth camera |
| `adaptive_fusion.launch.py` | Launch fusion system |
| `pointcloud_filter.yaml` | Depth pointcloud preprocessing |
| `safety_verification.yaml` | Safety layer config |

### Files to Modify

| File | Modification |
|------|--------------|
| `wheelchair_global_localization.launch.py` | Add depth pointcloud |
| `wheelchair_autonomous_nav.launch.py` | Use new nav2 params |
| `nav2_params_wheelchair.yaml` | Add STVL layer |

---

## Appendix A: Reference Resources

### Papers
- [Fuzzy Logic Based Fusion (ASME 2022)](https://asmedigitalcollection.asme.org/IMECE/proceedings-abstract/IMECE2022/86670/V005T07A069/1157190)
- [Multi-Sensor Fusion (IJITSR 2024)](https://link.springer.com/article/10.1007/s13177-024-00460-x)
- [STVL Paper (IJRR 2020)](https://journals.sagepub.com/doi/full/10.1177/1729881420910530)

### GitHub Repositories
- [STVL](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)
- [ros2_navigation_stvl](https://github.com/mich-pest/ros2_navigation_stvl)
- [awesome-LiDAR-Visual-SLAM](https://github.com/sjtuyinjie/awesome-LiDAR-Visual-SLAM)

### Documentation
- [Nav2 STVL Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_stvl.html)
- [Nav2 Costmap Configuration](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)

---

## Appendix B: Evaluation Metrics for Publication

### Localization Accuracy
- Absolute Trajectory Error (ATE)
- Relative Pose Error (RPE)
- Map consistency score

### Obstacle Detection
- True Positive Rate (TPR)
- False Positive Rate (FPR)
- Detection latency
- Miss rate for different obstacle types

### Navigation Performance
- Success rate (% goals reached)
- Path length efficiency
- Computation time
- Recovery behavior frequency

### Safety Metrics
- Near-miss rate
- Emergency stop frequency
- Passenger comfort (jerk, acceleration)

---

*Analysis generated for publication-worthy visual-LiDAR fusion wheelchair navigation system*

# Safe Sensor Fusion & Fault Detection

**Last Updated**: 2025-12-06
**Purpose**: Keep the wheelchair safe when sensors degrade, fail, or disagree

---

## Overview

This document covers intelligent sensor fusion that can:
1. Detect faulty sensor data
2. Continue operating with degraded sensors
3. Stop safely when localization is not possible
4. Decide when sensor combination is insufficient for navigation

---

## Part 1: What EKF Already Does (Your Senior is Right!)

The Extended Kalman Filter (EKF) in `robot_localization` already provides significant fault tolerance:

### Built-in EKF Capabilities

| Feature | How It Works |
|---------|--------------|
| **Covariance weighting** | Sensors with high uncertainty contribute less |
| **Innovation gating** | Rejects measurements that deviate too much from prediction |
| **Missing data handling** | Continues prediction if a sensor temporarily drops |
| **Multi-sensor fusion** | Combines wheel odom + IMU optimally |

### EKF Parameters for Fault Tolerance

```yaml
# In ekf.yaml
# Mahalanobis distance threshold for rejecting outliers
mahalanobis_threshold: 5.0

# Process noise - how much to trust prediction vs measurement
process_noise_covariance: [...]

# Sensor noise - uncertainty of each sensor
# Higher values = less trust in that sensor
```

### What EKF Does NOT Do

- Does not detect **complete sensor failure** (just handles missing data briefly)
- Does not make **stop decisions** (that's your job)
- Does not track **sensor health over time**
- Does not handle **systematic errors** (like wrong frame transforms)

---

## Part 2: EKF vs AI/ML - Honest Comparison

### Your Senior's Point: "EKF does this already"

**TRUE for:**
- Statistical outlier rejection
- Optimal sensor weighting
- Handling temporary dropouts
- Fusing multiple sensors

**FALSE for:**
- Complex pattern recognition (e.g., "IMU is on a vibrating surface")
- Long-term health monitoring
- Detecting subtle degradation before failure
- Learning environment-specific failure modes

### When AI/ML Adds Value

| Scenario | EKF | AI/ML |
|----------|-----|-------|
| Single outlier measurement | Handles well | Overkill |
| Sensor gradually drifting | Misses it | Can detect trend |
| IMU on vibrating surface | Can't distinguish | Can learn pattern |
| Wheel slipping on carpet | Treats as valid | Can learn signature |
| Lidar seeing glass walls | No understanding | Can learn failure mode |

### The Dataset Problem (Your Question!)

**"From where will you bring dataset?"** - This is the RIGHT question!

#### Option 1: Collect Your Own (Recommended)

```bash
# Create fault injection datasets
# 1. Normal operation - label as "healthy"
rosbag record /imu /wc_control/odom /scan -O healthy_indoor.bag

# 2. Induced faults - label by fault type
# - Cover IMU with hand (vibration/temperature change)
# - Lift one wheel (encoder fault)
# - Block lidar partially (occlusion)
# - Drive on different surfaces (carpet, tile, outdoor)
rosbag record /imu /wc_control/odom /scan -O fault_imu_covered.bag
```

#### Option 2: Public Datasets (Limited Use)

| Dataset | Sensors | Use Case |
|---------|---------|----------|
| EuRoC MAV | Stereo + IMU | IMU fault patterns |
| TUM VI | Camera + IMU | Visual-inertial failures |
| KITTI | Lidar + Camera + GPS + IMU | Multi-sensor baseline |
| nuScenes | Full AV stack | Failure case studies |

**Problem**: These are for drones/cars, NOT wheelchairs. Different:
- Motion dynamics
- Environment (indoor vs road)
- Failure modes

#### Option 3: Simulation + Domain Adaptation

```python
# Simulate faults in Gazebo
# - Add noise to IMU
# - Drop wheel encoder packets
# - Add lidar occlusions
# Then fine-tune on real wheelchair data
```

### Honest Recommendation

**Start with EKF + Simple Rules** (no ML needed):
1. Rate monitoring (is sensor publishing?)
2. Range checks (is value physically possible?)
3. Consistency checks (do sensors agree?)

**Add ML later IF**:
- You have 50+ hours of labeled fault data
- Simple rules aren't catching failures
- You have compute budget for inference

---

## Part 3: Practical Fault Detection (No ML Required)

### Sensor Health Monitor Node

```python
class SensorHealthMonitor:
    """
    Monitor sensor health using simple, interpretable rules.
    No ML required - just physics and statistics.
    """

    def __init__(self):
        # Expected rates
        self.expected_rates = {
            'imu': 200.0,      # Hz
            'odom': 50.0,      # Hz
            'scan': 10.0,      # Hz
        }

        # Timeout thresholds
        self.timeout_warn = 0.5   # seconds
        self.timeout_fail = 2.0   # seconds

        # Health status
        self.health = {
            'imu': 'OK',
            'odom': 'OK',
            'scan': 'OK',
            'fusion': 'OK'
        }

    def check_rate(self, sensor, actual_rate):
        """Check if sensor is publishing at expected rate."""
        expected = self.expected_rates[sensor]
        if actual_rate < expected * 0.5:
            return 'DEGRADED'
        elif actual_rate < expected * 0.1:
            return 'FAILED'
        return 'OK'

    def check_imu_sanity(self, msg):
        """Check IMU values are physically plausible."""
        # Angular velocity limits (rad/s)
        if abs(msg.angular_velocity.z) > 10.0:  # ~570 deg/s
            return 'SUSPECT'

        # Linear acceleration limits (m/s^2)
        accel_mag = sqrt(msg.linear_acceleration.x**2 +
                        msg.linear_acceleration.y**2 +
                        msg.linear_acceleration.z**2)
        if accel_mag < 5.0 or accel_mag > 15.0:  # Expect ~9.8
            return 'SUSPECT'

        return 'OK'

    def check_odom_sanity(self, msg):
        """Check odometry is physically plausible."""
        # Wheelchair max speed ~2 m/s
        if abs(msg.twist.twist.linear.x) > 3.0:
            return 'SUSPECT'

        # Max turn rate ~2 rad/s
        if abs(msg.twist.twist.angular.z) > 3.0:
            return 'SUSPECT'

        return 'OK'

    def check_consistency(self, imu_yaw_rate, odom_yaw_rate):
        """Check IMU and odom agree on yaw rate."""
        diff = abs(imu_yaw_rate - odom_yaw_rate)
        if diff > 0.5:  # rad/s disagreement
            return 'INCONSISTENT'
        return 'OK'
```

### Degraded Operation Modes

```
NORMAL MODE (all sensors healthy)
├── IMU: yaw + yaw_rate
├── Odom: x, y, vx
├── Lidar: localization
└── Full navigation enabled

DEGRADED MODE 1 (IMU failed)
├── Odom: x, y, vx, yaw (from wheel diff)
├── Lidar: localization
├── Limit: Reduce max turn rate, warn user
└── Navigation: Continue with caution

DEGRADED MODE 2 (Odom failed)
├── IMU: yaw, yaw_rate
├── Lidar: localization (if AMCL)
├── Limit: No forward velocity estimate, crawl speed only
└── Navigation: Minimal, prepare to stop

DEGRADED MODE 3 (Lidar failed)
├── IMU: yaw + yaw_rate
├── Odom: x, y, vx
├── Limit: Dead reckoning only, time-limited
└── Navigation: Return to last known safe position

STOP MODE (critical failure)
├── Triggers:
│   - IMU + Odom both failed
│   - Localization confidence too low
│   - Sensor disagreement too large
└── Action: Safe stop, alert user
```

---

## Part 4: Decision Matrix

### Can We Navigate?

| IMU | Odom | Lidar | Can Navigate? | Mode |
|-----|------|-------|---------------|------|
| OK  | OK   | OK    | YES | NORMAL |
| OK  | OK   | FAIL  | YES (limited) | Dead reckoning, time limit |
| OK  | FAIL | OK    | YES (limited) | No velocity, slow mode |
| FAIL| OK   | OK    | YES (limited) | Wheel-only yaw, no rate |
| OK  | FAIL | FAIL  | NO | STOP - no position |
| FAIL| OK   | FAIL  | NO | STOP - no yaw source |
| FAIL| FAIL | OK    | NO | STOP - no odometry |
| FAIL| FAIL | FAIL  | NO | EMERGENCY STOP |

### Minimum Viable Sensor Sets

For **2D indoor navigation**, you need:
1. **Position source**: Wheel odom OR Lidar AMCL
2. **Orientation source**: IMU yaw OR Wheel differential
3. **Obstacle detection**: Lidar OR depth camera

If ANY category has zero healthy sensors → STOP

---

## Part 5: Implementation Plan

### Phase 1: Basic Monitoring (Week 1)

```yaml
# Add to launch file
sensor_monitor:
  ros__parameters:
    imu_topic: /imu
    odom_topic: /wc_control/odom
    scan_topic: /scan
    rate_tolerance: 0.5
    timeout_warn: 0.5
    timeout_fail: 2.0
```

### Phase 2: Consistency Checks (Week 2)

- Compare IMU yaw rate vs wheel-derived yaw rate
- Flag when disagreement > threshold
- Log disagreements for analysis

### Phase 3: Degraded Mode Logic (Week 3)

- Implement mode transitions
- Add velocity limits per mode
- User alerts/warnings

### Phase 4: Optional ML Enhancement (Future)

Only if needed:
- Collect fault dataset (50+ hours)
- Train simple anomaly detector
- Deploy as sidecar, not replacing EKF

---

## Part 6: Why NOT to Over-Engineer with AI/ML

### The 80/20 Rule

- **80% of failures** are caught by simple rules:
  - Sensor stopped publishing
  - Value out of physical range
  - Sensors disagree significantly

- **20% of failures** might need ML:
  - Subtle drift before failure
  - Environment-specific issues
  - Complex multi-sensor correlations

### Cost-Benefit Analysis

| Approach | Development Time | Maintenance | Reliability |
|----------|------------------|-------------|-------------|
| Simple rules | 1-2 weeks | Low | High (interpretable) |
| ML-based | 2-3 months | High | Medium (black box) |

### Your Senior's Wisdom

> "EKF does this only"

This is mostly correct. The EKF handles:
- Normal sensor noise
- Occasional outliers
- Brief dropouts

What you ADD on top:
- Stop/go decisions
- Mode switching
- User alerts
- Long-term health tracking

These don't require ML - just good engineering.

---

## Part 7: Datasets for Future ML (If Needed)

### Self-Collected (Most Valuable)

```bash
# Recording script
#!/bin/bash
FAULT_TYPE=$1  # healthy, imu_covered, wheel_lifted, etc.
rosbag record \
    /imu /wc_control/odom /scan \
    /camera/imu /camera/depth/image_rect_raw \
    -O "${FAULT_TYPE}_$(date +%Y%m%d_%H%M%S).bag"
```

### Fault Injection Scenarios

| Fault | How to Induce | Label |
|-------|---------------|-------|
| IMU vibration | Place on vibrating surface | `imu_vibration` |
| IMU temperature | Cover/heat camera | `imu_thermal` |
| Wheel slip | Drive on plastic sheet | `odom_slip` |
| Wheel lift | Lift one side slightly | `odom_lift` |
| Lidar occlusion | Block partially with cardboard | `lidar_occluded` |
| Lidar reflection | Point at mirror/glass | `lidar_reflection` |

### Public Datasets (Reference Only)

| Dataset | URL | Useful For |
|---------|-----|------------|
| EuRoC | projects.asl.ethz.ch/datasets | IMU characteristics |
| TUM VI | vision.in.tum.de/data/datasets | VI failure modes |
| Rawseeds | rawseeds.ira.disco.unimib.it | Indoor robot data |

**Remember**: Public datasets won't have YOUR wheelchair's failure modes. Always validate/fine-tune on your own data.

---

## Summary

### What to Implement NOW

1. **Rate monitoring** - Is sensor publishing?
2. **Range checks** - Is value physically possible?
3. **Consistency checks** - Do sensors agree?
4. **Mode switching** - Degrade gracefully
5. **Stop logic** - Know when to stop

### What to Defer (Maybe Never Needed)

1. ML anomaly detection
2. Complex pattern recognition
3. Predictive failure models

### The Right Mindset

> "Make it work with simple rules first. Add ML only when you have data proving simple rules aren't enough."

Your senior is right that EKF handles a lot. Your job is to add the **decision layer** on top - when to trust fusion, when to degrade, when to stop. That's engineering, not ML.

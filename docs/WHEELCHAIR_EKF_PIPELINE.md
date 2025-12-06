# Wheelchair EKF/IMU Pipeline (Jazzy)

**Last Updated**: 2025-12-06
**Status**: Production-tested, 99% yaw drift reduction achieved

This captures the **current, tested** IMU → EKF pipeline and the learnings from extensive calibration testing.

---

## Pipeline Overview

```
/camera/imu (raw RealSense)
       │
       ▼
┌──────────────────────┐
│  imu_bias_corrector  │  Subtracts gyro bias in CAMERA frame
│  (camera frame)      │  BEFORE Madgwick filter
└──────────────────────┘
       │
       ▼
/camera/imu_corrected
       │
       ▼
┌──────────────────────┐
│  imu_filter_madgwick │  Fuses accel+gyro → orientation
│  (orientation est.)  │  Outputs quaternion in camera frame
└──────────────────────┘
       │
       ▼
/imu/data
       │
       ▼
┌──────────────────────────────┐
│  imu_wheelchair_republisher  │  Transforms to base_link frame
│  (frame transform)           │  Using TF or hardcoded quaternion
└──────────────────────────────┘
       │
       ▼
/imu (base_link frame)
       │
       ▼
┌──────────────────────┐
│  robot_localization  │  Fuses IMU yaw + wheel odom
│  (EKF)               │  Outputs /odometry/filtered
└──────────────────────┘
```

---

## Frame Transformation (CRITICAL)

### Camera IMU Optical Frame → Base Link

The RealSense D455 camera IMU uses `camera_imu_optical_frame` which is rotated relative to `base_link`.

**Transform quaternion**: `[0.5, -0.5, 0.5, 0.5]` (x, y, z, w)

**Rotation matrix** (camera → base_link):
```
[ 0  -1   0 ]
[ 0   0  -1 ]
[ 1   0   0 ]
```

**Axis mapping**:
| Camera Axis | → | Base Link Axis |
|-------------|---|----------------|
| camera_x    | → | base_z (YAW)   |
| camera_y    | → | -base_x (ROLL) |
| camera_z    | → | -base_y (PITCH)|

**This means**:
- Camera gyro_x bias causes **base_link yaw drift**
- Camera gyro_y bias causes **base_link roll drift**
- Camera gyro_z bias causes **base_link pitch drift**

---

## Gyro Bias Calibration

### Current Calibrated Values (2025-12-06)

From static test `full_system_20251206_173829.csv` (1044 seconds):

```yaml
# In imu_bias_corrector.py and wheelchair_sensors.launch.py
gyro_x_bias: -0.004302  # rad/s → affects base_link YAW
gyro_y_bias:  0.000787  # rad/s → affects base_link ROLL
gyro_z_bias:  0.000948  # rad/s → affects base_link PITCH
```

### Results After Calibration

| Metric | Before Fix | After Fix | Improvement |
|--------|-----------|-----------|-------------|
| Yaw drift rate | 60.33 °/min | 0.62 °/min | **99%** |
| Rectangle yaw error | N/A | 1.3° | Excellent |
| Static test (3.4 min) | 60° drift | 2° drift | 97% |

### How to Recalibrate

1. **Log raw camera IMU data** during static test:
   ```bash
   ros2 run scripts topic_data_logger.py
   # Keep wheelchair completely still for 2+ minutes
   ```

2. **Analyze the log**:
   ```python
   import pandas as pd
   df = pd.read_csv('your_log.csv')

   # These are the bias values to use:
   gyro_x_bias = df['raw_cam_imu_ang_vel_x'].mean()
   gyro_y_bias = df['raw_cam_imu_ang_vel_y'].mean()
   gyro_z_bias = df['raw_cam_imu_ang_vel_z'].mean()
   ```

3. **Update bias corrector** in `wheelchair_sensors.launch.py`:
   ```python
   'gyro_x_bias': <measured_value>,
   'gyro_y_bias': <measured_value>,
   'gyro_z_bias': <measured_value>,
   ```

4. **Rebuild and test**:
   ```bash
   colcon build --packages-select wc_control --symlink-install
   ```

---

## Bias Instability (Important!)

### Key Finding

The gyro bias is **NOT perfectly stable**. It drifts over time due to:
- Temperature changes (camera heats up during operation)
- Time-dependent random walk
- Power cycle variations

### Measured Variation

From multiple static tests on 2025-12-06:

| Axis | Variation Range | Impact |
|------|-----------------|--------|
| gyro_x | 0.008 °/s | ~0.5 °/min yaw uncertainty |
| gyro_y | 0.002 °/s | Minimal |
| gyro_z | 0.013 °/s | Minimal |

**Within a single 17-minute test**, gyro_x varied by 0.019 °/s between segments.

### Industry Solutions

1. **Startup Calibration** (Most Common)
   - System stays static for 1-5 seconds at startup
   - Measures current bias and subtracts it
   - Simple, effective, used by smartphones/drones

2. **Online EKF Bias Estimation** (Professional)
   - Add gyro bias as state variable in EKF
   - EKF tracks and corrects bias in real-time
   - Used by autonomous vehicles, industrial robots

3. **Zero Velocity Updates (ZUPT)**
   - Detect stationary periods during operation
   - Recalibrate bias when stopped
   - Perfect for wheelchair (stops frequently)

4. **Temperature Compensation**
   - Factory calibrate bias vs temperature curve
   - Apply correction based on temperature sensor

### Recommendation for Wheelchair

Implement **Startup Calibration + ZUPT**:
- Calibrate on boot when wheelchair is stationary
- Recalibrate when detecting zero velocity during operation

---

## EKF Configuration

### What EKF Fuses

| Source | Data Used | Purpose |
|--------|-----------|---------|
| `/imu` | Yaw orientation, Yaw rate | Orientation, turn detection |
| `/wc_control/odom` | X/Y position, X velocity | Translation, forward motion |

### What EKF Does NOT Fuse
- IMU linear acceleration (too noisy)
- IMU roll/pitch (not needed for 2D navigation)

### robot_localization Config Key Points

```yaml
imu0: /imu
imu0_config: [false, false, false,    # x, y, z position
              false, false, true,      # roll, pitch, YAW
              false, false, false,     # x, y, z velocity
              false, false, true,      # roll_rate, pitch_rate, YAW_RATE
              false, false, false]     # x, y, z acceleration

odom0: /wc_control/odom
odom0_config: [true, true, false,      # X, Y, z position
               false, false, false,    # roll, pitch, yaw
               true, false, false,     # X velocity, y, z
               false, false, false,    # roll_rate, pitch_rate, yaw_rate
               false, false, false]    # accelerations
```

---

## Validation Checklist

1. **TF sanity**:
   ```bash
   ros2 run tf2_ros tf2_echo base_link camera_imu_optical_frame
   # Should show quat ≈ [-0.5, 0.5, -0.5, 0.5]
   ```

2. **Topic frame check**:
   ```bash
   ros2 topic echo -n 1 /imu header
   # frame_id should be: base_link
   ```

3. **Rate check**:
   ```bash
   ros2 topic hz /imu           # Should be ~200 Hz
   ros2 topic hz /wc_control/odom  # Should be ~50 Hz
   ```

4. **Static bias test**:
   - Log 2+ minutes static
   - Analyze: gyro_z mean should be < 0.005 rad/s
   - Yaw drift should be < 1 °/min

5. **Rectangle closure test**:
   - Drive a rectangle, return to start
   - Yaw error should be < 5° for one loop
   - Position closure < 0.5m

---

## Common Pitfalls (Solved)

| Problem | Cause | Solution |
|---------|-------|----------|
| 60°/min yaw drift | Wrong/no bias correction | Calibrate from RAW camera IMU |
| Gravity on wrong axis | Different quaternion for vectors vs orientation | Use same transform for all |
| Double bias correction | Bias in corrector AND republisher | Apply bias ONLY in corrector |
| Frame mismatch | Publishing in wrong frame | Always publish as `base_link` |
| TF inversion error | Using base→sensor instead of sensor→base | Invert TF output correctly |

---

## Files Reference

| File | Purpose |
|------|---------|
| `wc_control/imu_bias_corrector.py` | Applies gyro bias in camera frame |
| `wc_control/imu_wheelchair_republisher.py` | Transforms to base_link frame |
| `wc_control/wheelchair_sensors.launch.py` | Launches IMU pipeline with params |
| `scripts/topic_data_logger.py` | Logs raw IMU + odom for analysis |
| `wheelchair_localization/config/ekf.yaml` | EKF configuration |

---

## Test Results Summary (2025-12-06)

### Static Tests
- 17-minute test: 2.09° total drift (0.62 °/min)
- Previous (before fix): 1050° drift (60 °/min)

### Rectangle Trajectories
| Direction | Total Rotation | Error | Closure |
|-----------|---------------|-------|---------|
| Counter-clockwise | -361.3° | **+1.3°** | 0.40m |
| Clockwise | +329.5° | -30.5° | 0.25m |

Note: Clockwise had higher error due to bias drift between tests (demonstrates bias instability).

---

## Next Steps / Future Improvements

1. [ ] Implement startup auto-calibration (5-second static period)
2. [ ] Add ZUPT detection for bias recalibration during stops
3. [ ] Configure EKF online bias estimation
4. [ ] Temperature compensation (if temp sensor available)

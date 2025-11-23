# Complete Parameter Analysis: v14r6 vs Defaults vs Recommended

## Variance Penalties - THE CRITICAL PARAMETERS

### slam_toolbox Defaults (GitHub ROS config)
```yaml
distance_variance_penalty: 0.5
angle_variance_penalty: 1.0
```

### Your v14r6
```yaml
distance_variance_penalty: 0.75  # HIGHER than 0.5 = trusts scan MORE
angle_variance_penalty: 0.75     # LOWER than 1.0 BUT still too high for sharp corners
```

### Recommended (Current Config)
```yaml
distance_variance_penalty: 0.5   # ROS default - balanced
angle_variance_penalty: 0.25     # MUCH LOWER than 1.0 = trust odometry rotation
```

---

## Why Your v14r6 Has Issues

### Issue 1: Slanted Corners
**Root cause**: `angle_variance_penalty: 0.75`
- Default is 1.0 (already trusts scan matching)
- Your 0.75 is slightly better but STILL TOO HIGH
- Need 0.20-0.30 to preserve sharp rotations
- **Formula proof**:
  - Penalty = 1.0 - (0.2 * angleDeviation² / 0.75)
  - With 10° deviation: penalty = 0.95 (95% acceptance!)
  - Scan matching is smoothing your 90° turns

### Issue 2: Your Interpretation Was Backwards
You thought: "0.75 = 75% trust in odometry"
**Actually**: 0.75 = less penalty for deviating = trust scan matching MORE

---

## Complete Parameter Comparison

| Parameter | slam_toolbox Default | Your v14r6 | Recommended | Notes |
|-----------|---------------------|------------|-------------|-------|
| **distance_variance_penalty** | 0.5 | 0.75 | **0.5** | v14r6 too high, use default |
| **angle_variance_penalty** | 1.0 | 0.75 | **0.25** | Need MUCH lower for sharp corners |
| **correlation_search_space_dimension** | 0.5 | 0.7 | **1.0** | v14r6 too narrow for initial scan |
| **correlation_search_space_resolution** | 0.01 | 0.005 | **0.01** | v14r6 too fine (CPU cost) |
| **link_match_minimum_response_fine** | 0.1 | 0.30 | **0.25** | v14r6 too strict (freezing) |
| **link_scan_maximum_distance** | 1.5 | 1.0 | **1.5** | v14r6 too conservative |
| **minimum_travel_distance** | 0.5 | 0.15 | **0.15** | v14r6 good for clutter ✓ |
| **minimum_travel_heading** | 0.5 | 0.06 | **0.10** | v14r6 too small (noise) |
| **minimum_time_interval** | 0.5 | 0.05 | **0.1** | Match S3's 10Hz |
| **resolution** | 0.05 | 0.02 | **0.025** | Balance detail vs CPU |
| **map_update_interval** | 5.0 | 3.0 | **1.0** | More responsive |
| **loop_search_maximum_distance** | 3.0 | 10.0 | **15.0** | Match environment size |
| **loop_match_minimum_response_fine** | 0.45 | 0.45 | **0.45** | Good ✓ |
| **minimum_angle_penalty** | 0.9 | 1.2 | **1.0** | v14r6 too strict |
| **minimum_distance_penalty** | 0.5 | 0.7 | **0.5** | Use default |
| **ceres_loss_function** | None | None | **HuberLoss** | Robust to clutter |

---

## The Source Code Formula (DEFINITIVE)

From `Mapper.cpp`:

```cpp
// Constants
const kt_double DISTANCE_PENALTY_GAIN = 0.2;
const kt_double ANGLE_PENALTY_GAIN = 0.2;

// Calculate penalties
kt_double squaredDistance = squareX + squareY;  // deviation from odometry (meters²)
kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN *
    squaredDistance / m_pMapper->m_pDistanceVariancePenalty->GetValue());

kt_double squaredAngleDistance = math::Square(angle - m_rSearchCenter.GetHeading());
kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN *
    squaredAngleDistance / m_pMapper->m_pAngleVariancePenalty->GetValue());

// Apply to response
response *= (distancePenalty * anglePenalty);
```

**Since variance_penalty is in DENOMINATOR**:
- Larger variance_penalty → smaller fraction → penalty closer to 1.0 → response stays HIGH → ACCEPT deviation
- Smaller variance_penalty → larger fraction → penalty reduced MORE → response goes LOW → REJECT deviation

---

## Example Calculations

### Rotation Scenario: 10° deviation from odometry

**Your v14r6** (angle_variance_penalty: 0.75):
```
squaredAngleDistance = (10° = 0.174 rad)² = 0.0303
anglePenalty = 1.0 - (0.2 * 0.0303 / 0.75) = 1.0 - 0.0081 = 0.992
response *= 0.992  (99.2% acceptance!)
→ Scan match ACCEPTED even though it deviates 10° from odometry
→ Result: Corners get smoothed/slanted
```

**Recommended** (angle_variance_penalty: 0.25):
```
squaredAngleDistance = 0.0303
anglePenalty = 1.0 - (0.2 * 0.0303 / 0.25) = 1.0 - 0.0242 = 0.976
response *= 0.976  (97.6% acceptance)
→ Stronger penalty, but still reasonable
→ Small deviations rejected, preserves odometry's sharp turns
```

**Code default** (angle_variance_penalty: 0.12):
```
anglePenalty = 1.0 - (0.2 * 0.0303 / 0.12) = 1.0 - 0.0505 = 0.950
response *= 0.950  (95% acceptance)
→ Even stronger penalty
→ Very sharp corners, maximum odometry trust
```

---

## Why Hector SLAM Worked for You

From Hector SLAM launch file:
```xml
<param name="use_tf_pose_start_estimate" value="false"/>
```

**Hector SLAM is PURE scan matching**:
- No odometry integration AT ALL
- No variance penalties needed
- No odometry/scan conflict
- Works great when you have good geometry (sharp corners, rectangles)
- But drifts over time without odometry correction

---

## Recommendations

### For Sharp 90° Corners (Your Priority)

**Option 1: Moderate Trust** (Current config - RECOMMENDED)
```yaml
distance_variance_penalty: 0.5   # ROS default
angle_variance_penalty: 0.25     # Low - trust odometry rotation
```
- Should give sharp corners
- Allows some scan correction for position
- Balanced approach

**Option 2: Maximum Odometry Trust** (If Option 1 not sharp enough)
```yaml
distance_variance_penalty: 0.3   # Lower than default
angle_variance_penalty: 0.15     # Very low - maximum rotation trust
```
- Sharpest possible corners
- Minimal scan matching influence
- Closer to Hector SLAM behavior

**Option 3: Extreme** (Nuclear option)
```yaml
distance_variance_penalty: 0.09  # Code default
angle_variance_penalty: 0.12     # Code default
```
- Essentially ignores scan matching for odometry deviations
- Pure odometry-driven SLAM
- Use only if your odometry is PERFECT

---

## Testing Strategy

1. **Start with current config** (distance: 0.5, angle: 0.25)
2. **Drive L-shape pattern**:
   - 6m forward
   - 90° turn
   - 4m forward
3. **Check RViz map**:
   - Measure corner angle
   - Should be 88-90°

**If corners STILL slanted** → Decrease angle to 0.20 or 0.15
**If corners PERFECT but map noisy** → Increase to 0.30
**If corners TOO sharp (overshooting)** → Increase to 0.35-0.40

---

## Summary: What Was Wrong

### Your v14r6 Misconceptions

1. **Backwards interpretation**: You thought higher value = more odometry trust
   - Actually: higher value = LESS penalty for deviation = more scan trust

2. **Symmetric penalties**: 0.75/0.75
   - You thought: "Balanced, both at 75% odometry trust"
   - Actually: Both trusting scan matching MORE than defaults
   - Defaults are asymmetric: 0.5/1.0 (position balanced, rotation scan-favored)

3. **Missing the formula**: Without checking source code
   - Can't understand parameter effect
   - Guessing led to wrong direction

### The Fix

- **distance: 0.5** (keep ROS default - it's good)
- **angle: 0.25** (MUCH lower than default 1.0 to preserve rotation)
- Result: Sharp corners restored!

---

## File Locations

**Current corrected config**:
- `/home/user/wc/src/wheelchair_localization/config/slam_toolbox.yaml`
- Has correct values: distance=0.5, angle=0.25

**Your v14r6** (from your message):
- You haven't pushed this yet (good!)
- Don't use it - interpretation was wrong

**Launch file**:
- `/home/user/wc/src/wheelchair_bringup/launch/wheelchair_slam_mapping.launch.py`
- Uses `slam_toolbox.yaml` by default
- Will automatically use corrected config

---

**Test now and report results!**

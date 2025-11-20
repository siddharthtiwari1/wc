# SLAM Map Quality Assessment Guide

## Quick Quality Checks (5 Minutes)

### ✅ Method 1: Visual Inspection in RViz

**What to Look For:**

| Feature | Good Quality ✅ | Poor Quality ❌ |
|---------|----------------|-----------------|
| **Wall Thickness** | 1-2 cells (5-10cm) | >3 cells (15cm+) thick or fuzzy |
| **Wall Straightness** | Clean, straight lines | Wavy, bumpy, irregular |
| **Corners** | Sharp 90° angles | Rounded, blurry, multiple lines |
| **Ghosting** | Single walls only | Double/triple wall images |
| **Artifacts** | Clean map | Random dots, floating obstacles |
| **Symmetry** | Symmetric rooms look symmetric | Distorted, skewed shapes |
| **Closure** | Loops close perfectly | Gap when returning to start |

**How to Check:**
1. Launch SLAM and drive around
2. In RViz, look at the `/map` topic
3. Zoom in on walls (use mouse wheel)
4. Check each criterion above

---

### ✅ Method 2: Loop Closure Test (BEST QUANTITATIVE TEST)

**This measures how accurately SLAM closes a loop.**

**Test Procedure:**

1. **Mark your starting position** (use tape on floor)
2. **Launch SLAM:**
   ```bash
   ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py
   ```
3. **In another terminal, monitor position:**
   ```bash
   ros2 topic echo /slam_toolbox/pose
   ```
4. **Record starting pose** (write down x, y, theta from first message)
5. **Drive a square pattern:**
   - Forward 3 meters → Stop
   - Rotate 90° left → Stop
   - Forward 3 meters → Stop
   - Rotate 90° left → Stop
   - Forward 3 meters → Stop
   - Rotate 90° left → Stop
   - Forward 3 meters → Stop
   - Rotate 90° left → Stop (back to start)
6. **Return to tape mark** (visually align wheelchair)
7. **Record ending pose** (x, y, theta)
8. **Calculate error:**
   - X error: |end_x - start_x|
   - Y error: |end_y - start_y|
   - Total error: sqrt(X_error² + Y_error²)

**Quality Rating:**

| Total Error | Map Quality | Rating |
|-------------|-------------|--------|
| **< 5 cm** | Excellent | ⭐⭐⭐⭐⭐ |
| **5-10 cm** | Very Good | ⭐⭐⭐⭐ |
| **10-20 cm** | Good | ⭐⭐⭐ |
| **20-50 cm** | Fair | ⭐⭐ |
| **> 50 cm** | Poor | ⭐ Need tuning |

---

### ✅ Method 3: SLAM Metrics Monitoring

**Check SLAM Toolbox's internal quality scores**

**Real-time monitoring:**
```bash
# Terminal 1: Launch SLAM
ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py

# Terminal 2: Monitor scan matching quality
ros2 topic echo /slam_toolbox/scan_visualization

# Terminal 3: Monitor loop closures
ros2 topic echo /slam_toolbox/graph_visualization

# Terminal 4: Check TF quality
ros2 run tf2_ros tf2_echo map odom
```

**What to watch:**
- `/slam_toolbox/pose` - Should update smoothly, no jumps
- TF `map→odom` - Should be stable, small corrections only
- Console output - Look for "Loop closure found" messages

---

### ✅ Method 4: Wall Thickness Measurement

**Measure actual vs map thickness**

**Physical measurement:**
1. Measure a real wall with measuring tape: e.g., 15cm thick
2. In RViz, use "Measure" tool or count grid cells
3. Calculate:
   ```
   Map thickness = (number of cells) × resolution
   Example: 2 cells × 0.05m = 0.10m = 10cm

   Error = |Map thickness - Real thickness|
   Quality = (1 - Error/Real thickness) × 100%
   ```

**Good quality:** Error < 5cm
**Poor quality:** Error > 10cm

---

### ✅ Method 5: Distance Accuracy Test

**Compare real-world distances to map distances**

**Test procedure:**
1. **Measure real distance** with measuring tape:
   - Example: Door to opposite wall = 5.00 meters
2. **Drive SLAM** to create map
3. **Measure in RViz:**
   - Use "Measure" tool or ruler
   - Or use this command:
   ```bash
   ros2 topic echo /slam_toolbox/pose
   # Drive from point A to point B, record poses
   # Distance = sqrt((x2-x1)² + (y2-y1)²)
   ```
4. **Calculate accuracy:**
   ```
   Error = |Map distance - Real distance|
   Accuracy = (1 - Error/Real distance) × 100%
   ```

**Quality rating:**

| Accuracy | Map Quality |
|----------|-------------|
| **> 99%** (< 1% error) | Excellent ⭐⭐⭐⭐⭐ |
| **98-99%** (1-2% error) | Very Good ⭐⭐⭐⭐ |
| **95-98%** (2-5% error) | Good ⭐⭐⭐ |
| **90-95%** (5-10% error) | Fair ⭐⭐ |
| **< 90%** (> 10% error) | Poor ⭐ |

---

### ✅ Method 6: Scan Matching Score

**Monitor SLAM's confidence in scan matching**

**Enable debug logging:**
Edit `/home/sidd/wc_ws/src/wheelchair_localization/config/slam_toolbox.yaml`:
```yaml
debug_logging: true  # Change from false to true
```

Rebuild and launch:
```bash
cd ~/wc_ws
colcon build --packages-select wheelchair_localization
source install/setup.bash
ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py
```

**Watch console output for:**
```
[slam_toolbox]: Scan matching score: 0.85
```

**Score interpretation:**

| Score | Quality | Meaning |
|-------|---------|---------|
| **> 0.8** | Excellent | High confidence match ⭐⭐⭐⭐⭐ |
| **0.6-0.8** | Good | Acceptable match ⭐⭐⭐⭐ |
| **0.4-0.6** | Fair | Weak match ⭐⭐⭐ |
| **< 0.4** | Poor | Very uncertain ⭐⭐ |

**If scores are low:** Environment has poor features, increase movement thresholds

---

### ✅ Method 7: Repeatability Test

**Map the same area twice and compare**

**Test procedure:**
1. **Map area once:**
   ```bash
   ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py
   # Drive around, then save map
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'test_map1'}}"
   ```

2. **Restart and map again (same path):**
   ```bash
   # Kill and restart
   ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py
   # Drive EXACT same path
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'test_map2'}}"
   ```

3. **Compare maps visually:**
   - Load both in RViz
   - Overlay them
   - Check if they're nearly identical

**Good quality:** Maps are 95%+ identical
**Poor quality:** Significant differences

---

## Automated Quality Checker Script

I'll create a script to automate some checks:

**File: `/home/sidd/wc_ws/src/scripts/scripts/map_quality_checker.py`**

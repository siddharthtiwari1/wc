# RPLidar S3 SLAM Tuning Guide for Wheelchair

## Table of Contents
1. [Configuration Files Overview](#configuration-files-overview)
2. [RPLidar S3 Capabilities](#rplidar-s3-capabilities)
3. [Parameter Tuning Strategy](#parameter-tuning-strategy)
4. [Field Survey Checklist](#field-survey-checklist)
5. [Progressive Tuning Process](#progressive-tuning-process)
6. [Troubleshooting](#troubleshooting)

---

## Configuration Files Overview

You now have **THREE** SLAM configurations:

| File | Use Case | Performance | Stability |
|------|----------|-------------|-----------|
| `slam_toolbox.yaml` | **BASELINE** - Start here | Conservative (A1-like) | ⭐⭐⭐⭐⭐ Max |
| `slam_toolbox_s3_optimized.yaml` | **RECOMMENDED** - Production use | Balanced S3 optimization | ⭐⭐⭐⭐ High |
| `slam_toolbox_s3_aggressive.yaml` | **EXPERIMENTAL** - Feature-rich areas | Maximum S3 utilization | ⭐⭐⭐ Medium |

### When to Use Each Config:

**Use `slam_toolbox.yaml` (Baseline) when:**
- First time mapping a new environment
- Environment has poor features (blank walls, long corridors)
- You need guaranteed stability

**Use `slam_toolbox_s3_optimized.yaml` (Recommended) when:**
- Environment has moderate-to-good features
- Indoor spaces with furniture, doorways, corners
- You want better performance than A1 config

**Use `slam_toolbox_s3_aggressive.yaml` (Experimental) when:**
- Environment is feature-rich (warehouses, offices, outdoor with structures)
- You tested optimized config and it works perfectly
- You want maximum responsiveness and range

---

## RPLidar S3 Capabilities

### What Makes S3 Superior:

| Feature | RPLidar A1 | **RPLidar S3** | Your Advantage |
|---------|------------|----------------|----------------|
| Range | 12m | **40m** | 3.3x longer detection |
| Scan Rate | 5.5Hz | **20Hz** | 3.6x more frequent scans |
| Sample Rate | 8k/s | **32k/s** | 4x denser point clouds |
| Angular Resolution | ~1° | **0.225°** | 4.4x finer detail |
| Points per scan | ~360 | **~1,600** | 4.4x more data points |

**Key Insight:** S3 gives you ~1,600 laser points every 50ms, vs A1's ~360 points every 180ms.

### S3 Range Quality Profile:

| Distance | Quality | Recommended Use |
|----------|---------|-----------------|
| 0-12m | ⭐⭐⭐⭐⭐ Excellent | Safe for all configs |
| 12-20m | ⭐⭐⭐⭐ Very Good | Optimized config |
| 20-30m | ⭐⭐⭐ Good | Aggressive config (outdoor) |
| 30-40m | ⭐⭐ Fair | Noisy, avoid for SLAM |

**Recommendation:** Use 12-20m range for best quality/performance balance.

---

## Parameter Tuning Strategy

### Key Parameters to Tune for S3:

#### 1. **Movement Thresholds** (MOST CRITICAL)

These control when SLAM processes a new scan:

```yaml
minimum_travel_distance: 0.4  # meters - distance threshold
minimum_travel_heading: 0.4   # radians (~23°) - rotation threshold
```

**How to tune based on environment:**

| Environment Type | Distance (m) | Heading (rad) | Reasoning |
|------------------|--------------|---------------|-----------|
| **Large warehouse/outdoor** | 0.25-0.35 | 0.25-0.35 | Open space, good features at distance |
| **Office/indoor with furniture** | 0.35-0.45 | 0.35-0.45 | Moderate features, balanced |
| **Narrow hallways/corridors** | 0.45-0.60 | 0.45-0.60 | Poor features, need more movement |
| **Blank walls/featureless** | 0.50-0.70 | 0.50-0.70 | Very poor features, conservative |

**Rule of Thumb:**
- **Good features** (corners, objects, varied geometry) → **SMALLER** thresholds (0.25-0.35)
- **Poor features** (blank walls, uniform corridors) → **LARGER** thresholds (0.50+)

#### 2. **Laser Range** (LEVERAGE S3'S STRENGTH)

```yaml
max_laser_range: 20.0  # meters
```

**Progressive tuning:**
1. Start: `12.0` (A1 baseline - guaranteed to work)
2. Test: `15.0` (25% increase)
3. Test: `20.0` (S3 optimized - recommended)
4. Test: `25.0` (aggressive, outdoor)
5. Max: `30.0` (only for very open outdoor spaces)

**Signs to increase:** Map looks "short-sighted", not detecting distant walls
**Signs to decrease:** Map has noise/artifacts at edges, unstable

#### 3. **Map Update Rate** (RESPONSIVENESS)

```yaml
map_update_interval: 2.0  # seconds
```

**Progressive tuning:**
1. Conservative: `5.0` (bumperbot baseline)
2. Moderate: `2.0` (S3 optimized - 2.5x faster)
3. Aggressive: `1.0` (5x faster, feature-rich only)
4. Max: `0.5` (10x faster, may lag on slow computers)

**Warning:** Faster updates need more CPU. Monitor with `htop`.

#### 4. **Scan Buffer Size** (USE S3'S DENSITY)

```yaml
scan_buffer_size: 20  # number of scans
```

**Tuning guide:**
- A1 setting: `10`
- S3 moderate: `15-20` (handles denser scans)
- S3 aggressive: `25-30` (maximum history)

**Trade-off:** Larger buffer = more memory, better scan matching in large spaces

#### 5. **Loop Closure Range** (LONG-RANGE ACCURACY)

```yaml
loop_search_maximum_distance: 8.0  # meters
```

**Progressive tuning based on space size:**
| Space Size | Loop Distance | Reasoning |
|------------|---------------|-----------|
| Small rooms (<5m) | 3.0-5.0 | Conservative |
| Medium spaces (5-15m) | 5.0-8.0 | S3 optimized |
| Large spaces (15-30m) | 8.0-12.0 | Use S3's range |
| Warehouses (30m+) | 12.0-20.0 | Maximum S3 |

#### 6. **Scan Processing Timing**

```yaml
minimum_time_interval: 0.3  # seconds between scans
```

**Tuning based on S3's 20Hz (50ms) scan rate:**
- Conservative: `0.5` (every 10th scan)
- Moderate: `0.3` (S3 optimized, every 6th scan)
- Aggressive: `0.2` (every 4th scan)
- Maximum: `0.1` (every 2nd scan, only for very fast motion)

---

## Field Survey Checklist

Before tuning, survey your environment:

### Environment Analysis:

**1. Feature Density** (MOST IMPORTANT)

Walk through your mapping area and score:

```
Feature Type                           Score (0-5)
─────────────────────────────────────────────────
Corners (walls meeting)                 _____
Doorways and openings                   _____
Furniture/obstacles                      _____
Varied wall distances                    _____
Geometric structures (pillars, etc)     _____
─────────────────────────────────────────────────
TOTAL SCORE:                            _____/25
```

**Scoring Guide:**
- **20-25 points:** Use `aggressive` config
- **15-19 points:** Use `optimized` config
- **10-14 points:** Use `baseline` config
- **<10 points:** Environment is very challenging, use conservative baseline

**2. Space Characteristics:**

- [ ] Maximum room dimension: _______ meters
- [ ] Longest corridor: _______ meters
- [ ] Narrowest passage: _______ meters
- [ ] Ceiling height: _______ meters
- [ ] Indoor / Outdoor / Mixed

**3. Surface Materials:**

Check materials that reflect laser poorly:
- [ ] Glass windows/walls
- [ ] Mirrors
- [ ] Shiny metal
- [ ] Very dark surfaces (absorb laser)
- [ ] Transparent materials

**More checkboxes = more challenging environment**

**4. Dynamic Obstacles:**

- [ ] People moving through area
- [ ] Moving equipment
- [ ] Doors that open/close
- [ ] Lighting changes (if camera-based)

**More dynamic = use conservative settings**

**5. Wheelchair Operating Speed:**

Typical speed during mapping: _______ m/s

**Speed-based recommendations:**
- **Slow (<0.3 m/s):** Can use aggressive thresholds (0.25m)
- **Moderate (0.3-0.7 m/s):** Use moderate thresholds (0.35-0.45m)
- **Fast (>0.7 m/s):** Use conservative thresholds (0.50m+)

---

## Progressive Tuning Process

### Step-by-Step Tuning:

#### Phase 1: Baseline Test (30 minutes)

**Config:** `slam_toolbox.yaml` (bumperbot-like)

1. Launch SLAM:
```bash
cd ~/wc_ws
source install/setup.bash
ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py \
    slam_config:=/home/sidd/wc_ws/src/wheelchair_localization/config/slam_toolbox.yaml
```

2. **Drive test pattern:**
   - Forward 3m → Stop
   - Rotate 90° → Stop
   - Forward 3m → Stop
   - Rotate 90° → Stop
   - Complete a square

3. **Observe in RViz:**
   - [ ] Map updates smoothly after moving >0.5m
   - [ ] Walls align well
   - [ ] Corners are sharp
   - [ ] No drift after completing square

4. **Baseline metrics to record:**
   - CPU usage: _____%
   - Map updates feel: Too slow / Good / Too fast
   - Scan matching quality: Poor / OK / Good / Excellent

✅ **If baseline fails:** Your environment is very challenging OR hardware issue (check `/scan` topic)

#### Phase 2: S3 Optimized Test (30 minutes)

**Config:** `slam_toolbox_s3_optimized.yaml`

1. Launch with optimized config:
```bash
ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py \
    slam_config:=/home/sidd/wc_ws/src/wheelchair_localization/config/slam_toolbox_s3_optimized.yaml
```

2. **Drive same test pattern**

3. **Compare to baseline:**
   - Map updates: Slower / Same / **Faster** ✓
   - Map detail: Less / Same / **More detail** ✓
   - Wall detection range: Shorter / Same / **Longer** ✓
   - CPU usage: Lower / Same / Higher (acceptable if <80%)

4. **Advanced test - Long range:**
   - Stand 15-20m from a wall
   - Check if wall appears in scan (blue points in RViz)
   - Drive toward wall slowly
   - Map should update continuously

✅ **If optimized works well:** You can try aggressive config
⚠️ **If map degrades:** Reduce `max_laser_range` to 15.0, reduce `minimum_travel_distance` to 0.45

#### Phase 3: S3 Aggressive Test (OPTIONAL - 30 minutes)

**Config:** `slam_toolbox_s3_aggressive.yaml`

⚠️ **Only proceed if:**
- Optimized config worked perfectly
- Your environment scored >15 on feature density
- You want maximum S3 performance

1. Launch aggressive config:
```bash
ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py \
    slam_config:=/home/sidd/wc_ws/src/wheelchair_localization/config/slam_toolbox_s3_aggressive.yaml
```

2. **Critical test - Small movements:**
   - Move forward ONLY 0.3m → Stop
   - Does map update? (Should update at 0.25m threshold)
   - Rotate ONLY 15° → Stop
   - Does map update? (Should update at 0.25 rad = 14°)

3. **Stress test - Large space:**
   - Drive to opposite end of largest room
   - Check if loop closure works (map corrects drift)

✅ **If map quality is excellent:** You've maximized S3!
⚠️ **If map becomes noisy/unstable:** Stick with optimized config

---

## What to Survey During Mapping

### Real-Time Monitoring:

**In RViz, watch these:**

1. **LaserScan topic (`/scan`):**
   - Points should be dense and smooth
   - Range: Check max distance of blue points
   - Good: Points extend 15-20m
   - Issue: Points only reach 5-10m (increase `max_laser_range`)

2. **Map topic (`/map`):**
   - Walls should be 1-2 cells thick (with 0.05 resolution)
   - Thick walls (>3 cells) → Reduce `minimum_travel_distance`
   - Walls have gaps → Increase scan buffer

3. **TF tree (`map` → `odom` → `base_link`):**
   - `map` → `odom` transform should update smoothly
   - Jerky updates → Increase movement thresholds

**In terminal, monitor CPU:**

```bash
# In separate terminal:
htop
# Watch for slam_toolbox CPU usage
# <60%: Excellent, can be more aggressive
# 60-80%: Good
# >80%: Too aggressive, reduce update rates
```

### Data to Collect:

Create a test log:

```
Date: __________
Config: baseline / optimized / aggressive
Environment: ____________________

Test 1: Square Pattern (4m × 4m)
─────────────────────────────────
Closure error (measure in RViz): _____ cm
CPU usage: _____%
Map update lag: None / Slight / Noticeable

Test 2: Long Corridor (15m+)
─────────────────────────────
Straight line drift: _____ cm
Wall visibility at 15m: Yes / Partial / No

Test 3: 360° Rotation in Place
────────────────────────────────
Return to start accuracy: _____ cm
Map artifacts: None / Minor / Major

Test 4: Large Loop (largest space)
───────────────────────────────────
Loop closure triggered: Yes / No
Post-closure map quality: Excellent / Good / Poor
```

---

## Troubleshooting

### Problem: Map not updating

**Symptoms:** Robot moves, but map stays frozen

**Diagnosis:**
1. Check `/scan` topic:
```bash
ros2 topic hz /scan
# Should show ~20 Hz for S3
```

2. Check movement thresholds:
```bash
ros2 param get /slam_toolbox minimum_travel_distance
ros2 param get /slam_toolbox minimum_travel_heading
```

**Solutions:**
- If thresholds are too large (>0.5m), robot isn't moving enough
- **Fix:** Reduce `minimum_travel_distance` to 0.3-0.4
- Or drive further before expecting update

---

### Problem: Map is noisy/has artifacts

**Symptoms:** Walls are thick, ghosting, extra features that don't exist

**Diagnosis:**
- Laser range too high (picking up noise at distance)
- Movement thresholds too small (processing similar scans)

**Solutions:**
1. Reduce `max_laser_range` from 20.0 → 15.0 → 12.0
2. Increase `minimum_travel_distance` from 0.4 → 0.5
3. Increase `correlation_search_space_smear_deviation` from 0.08 → 0.10

---

### Problem: Map drifts (doesn't close loops)

**Symptoms:** After driving a square/loop, start and end don't align

**Diagnosis:**
- Loop closure search distance too small
- Not enough distinctive features
- Odometry drift too large

**Solutions:**
1. Increase `loop_search_maximum_distance` from 8.0 → 12.0
2. Decrease `loop_match_minimum_response_fine` from 0.45 → 0.35 (more permissive)
3. Check odometry quality:
```bash
ros2 topic echo /wc_control/odom
# Watch for reasonable velocity values
```

---

### Problem: CPU usage too high (>80%)

**Symptoms:** System lags, map updates are jerky

**Diagnosis:**
- Update rates too aggressive for your computer
- Scan buffer too large
- Search spaces too large

**Solutions:**
1. Increase `map_update_interval` from 2.0 → 5.0
2. Reduce `scan_buffer_size` from 20 → 15
3. Reduce `loop_search_space_dimension` from 12.0 → 8.0
4. Increase `minimum_time_interval` from 0.3 → 0.5

---

### Problem: Short detection range despite S3

**Symptoms:** Walls beyond 10m not visible, using only half of S3's capability

**Diagnosis:**
- `max_laser_range` set too conservatively
- Physical obstacles blocking laser
- Laser quality degraded

**Solutions:**
1. Increase `max_laser_range`: 12.0 → 15.0 → 20.0
2. Check S3 is clean (no dust on lens)
3. Verify `/scan` ranges:
```bash
ros2 topic echo /scan --field ranges
# Look for values >12.0
```

---

## Recommended Workflow

### For Production Mapping:

```
1. Survey environment (15 min)
   └→ Fill out checklist above

2. Start with baseline config (30 min)
   └→ Verify system works, collect baseline metrics

3. Switch to optimized config (30 min)
   └→ Test in same environment, compare metrics

4. IF optimized works well AND environment is feature-rich:
   └→ Try aggressive config (30 min)

5. Select best config based on:
   - Map quality (most important)
   - CPU usage (<80%)
   - Update responsiveness

6. Fine-tune selected config:
   - Adjust movement thresholds based on results
   - Adjust laser range based on space size
   - Document final parameters
```

### Quick Reference: Parameter Adjustment

| Symptom | Increase ↑ | Decrease ↓ |
|---------|------------|------------|
| Map not updating | | `minimum_travel_distance`, `minimum_travel_heading` |
| Map too noisy | `minimum_travel_distance`, `minimum_travel_heading` | `max_laser_range` |
| Drift/poor loop closure | `loop_search_maximum_distance` | `loop_match_minimum_response_*` |
| CPU too high | `map_update_interval`, `minimum_time_interval` | `scan_buffer_size`, search dimensions |
| Map updates too slow | | `map_update_interval`, `minimum_time_interval` |
| Short range detection | `max_laser_range`, `scan_buffer_maximum_scan_distance` | |

---

## Launch File Modification

To use the new configs, modify your launch file:

**Option 1: Command-line override (easiest for testing)**

```bash
# Test optimized:
ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py \
    slam_config:=/home/sidd/wc_ws/src/wheelchair_localization/config/slam_toolbox_s3_optimized.yaml

# Test aggressive:
ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py \
    slam_config:=/home/sidd/wc_ws/src/wheelchair_localization/config/slam_toolbox_s3_aggressive.yaml
```

**Option 2: Change default in launch file**

Edit `/home/sidd/wc_ws/src/wheelchair_bringup/launch/wheelchair_slam_mapping.launch.py`:

Line 42-46, change:
```python
default_slam_config = os.path.join(
    wheelchair_localization_dir,
    'config',
    'slam_toolbox_s3_optimized.yaml',  # Changed from slam_toolbox.yaml
)
```

Then rebuild:
```bash
cd ~/wc_ws
colcon build --packages-select wheelchair_bringup
source install/setup.bash
```

---

## Summary: Leveraging S3's Superiority

**What makes S3 better than A1:**
1. ✅ **4x more laser points** → Enables denser scan buffers, better matching
2. ✅ **3.6x faster scans** → Can process scans more frequently
3. ✅ **3.3x longer range** → Enables distant loop closures, larger spaces
4. ✅ **4x finer resolution** → More accurate angular measurements

**How the configs use these advantages:**

| Feature | Baseline (A1-like) | Optimized (S3) | Aggressive (Max S3) |
|---------|-------------------|----------------|---------------------|
| Laser Range | 12m | **20m** | **25m** |
| Scan Buffer | 10 scans | **20 scans** | **25 scans** |
| Update Interval | 5.0s | **2.0s** | **1.0s** |
| Movement Threshold | 0.5m | **0.4m** | **0.25m** |
| Loop Search Distance | 3.0m | **8.0m** | **12.0m** |
| Scan Processing Time | 0.5s | **0.3s** | **0.2s** |

**Bottom line:** S3 optimized gives you ~2x better SLAM performance than A1, while aggressive can give 4x in ideal conditions.

Start with optimized, tune based on your environment survey! 🚀

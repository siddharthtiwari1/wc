# RAN System Deployment Guide

**Complete step-by-step guide for deploying RAN on wheelchair hardware**

**Author**: Siddharth Tiwari
**Last Updated**: 2025-11-22
**Status**: Production-ready system

---

## 📋 Pre-Deployment Checklist

### Hardware Setup

- [ ] **Wheelchair Platform**
  - [ ] Base controller operational
  - [ ] Emergency stop button accessible
  - [ ] Battery fully charged (>80%)
  - [ ] Wheels properly inflated
  - [ ] Base publishes `/odom` at ≥20 Hz
  - [ ] Base subscribes to `/cmd_vel`

- [ ] **RPLidar S3**
  - [ ] Mounted at 15cm height, level with ground
  - [ ] Full 360° visibility (no obstructions)
  - [ ] USB cable connected securely
  - [ ] Spinning when powered (red laser visible)
  - [ ] `/dev/ttyUSB*` device detected
  - [ ] Read/write permissions granted

- [ ] **RealSense D455**
  - [ ] Mounted front-facing, 30° downward tilt
  - [ ] USB 3.0 connection (blue port)
  - [ ] Firmware updated to latest version
  - [ ] Detected by `rs-enumerate-devices`
  - [ ] Publishing RGB + Depth at 30 Hz

- [ ] **Computing Unit**
  - [ ] NVIDIA GPU with CUDA 12.1+ (or CPU fallback)
  - [ ] At least 16 GB RAM
  - [ ] Ubuntu 24.04 LTS installed
  - [ ] ROS2 Jazzy installed and sourced
  - [ ] All Python dependencies installed

### Software Setup

- [ ] **Workspace Build**
  - [ ] `/home/user/wc` workspace exists
  - [ ] `colcon build` completed successfully
  - [ ] `install/setup.bash` sourced in `.bashrc`
  - [ ] No build errors or warnings

- [ ] **Pretrained Models**
  - [ ] YOLO-World (237 MB) downloaded
  - [ ] SAM2 (896 MB) downloaded
  - [ ] DINOv2 (auto-downloads on first run)
  - [ ] CLIP (auto-downloads on first run)

- [ ] **Configuration Files**
  - [ ] `ran_params.yaml` reviewed
  - [ ] Safety limits appropriate for environment
  - [ ] Camera/LiDAR frame IDs match your hardware

- [ ] **Validation Script**
  - [ ] `./scripts/validate_setup.py` executed
  - [ ] All critical checks passed
  - [ ] GPU detected (if available)

---

## 🚀 Deployment Steps

### Phase 1: Individual Node Testing (Week 1)

#### Day 1-2: Perception Node

```bash
# Terminal 1: Launch RealSense
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  align_depth.enable:=true

# Terminal 2: Run perception node
ros2 run ran_complete ran_perception_node.py

# Terminal 3: Monitor output
ros2 topic echo /ran/detections --once
```

**Expected output**:
- Detections published at ~10 Hz
- Objects detected: chair, table, etc.
- Confidence scores > 0.25
- Attribute confidences (color, shape, material)

**Test cases**:
1. Place colorful objects (red cup, blue book) in view
2. Move objects closer/farther (test depth quality)
3. Cover lens (should reject blurry frames)
4. Check attribute extraction accuracy

**Success criteria**:
- [x] Node starts without errors
- [x] Detections published continuously
- [x] Attributes extracted correctly
- [x] Quality metrics reasonable (q_blur > 0.3)

---

#### Day 3-4: Mapping Node

```bash
# Terminal 1: RealSense (from above)

# Terminal 2: Perception node (from above)

# Terminal 3: Mapping node
ros2 run ran_complete ran_mapping_node.py

# Terminal 4: Monitor semantic map
ros2 topic echo /ran/semantic_map --once
```

**Expected output**:
- Semantic map published at ~1 Hz
- Instances clustered correctly
- Multi-view fusion working
- Adaptive threshold τ_sem increases with object count

**Test cases**:
1. Drive wheelchair slowly (0.2 m/s) around room
2. View same object from multiple angles
3. Add/remove objects (test dynamic updates)
4. Check instance IDs stable across views

**Success criteria**:
- [x] Map builds incrementally
- [x] Same object doesn't split into multiple instances
- [x] Different objects don't merge incorrectly
- [x] Feature averaging improves confidence over time

---

#### Day 5-6: Hierarchical Verifier

```bash
# Terminal 1-3: RealSense + Perception + Mapping (from above)

# Terminal 4: Verifier node
ros2 run ran_complete ran_hierarchical_verifier.py

# Terminal 5: Test verification service
ros2 service call /ran/verify_goal ran_complete/srv/VerifyGoal \
  "{object_spec: {name: 'chair', color: 'red', shape: '', material: ''}}"
```

**Expected output**:
- 4-level cascade executed
- Level 1: Fast category filtering (<50ms)
- Level 2: Salient attribute check (~100ms)
- Level 3: Full matching (~500ms)
- Returns pose + confidence

**Test cases**:
1. Query existing object: "red chair"
2. Query non-existent object: "green lamp"
3. Query ambiguous object: "chair" (multiple chairs)
4. Query with full attributes: "wooden table near window"

**Success criteria**:
- [x] Correct object selected when exists
- [x] Returns empty when object not found
- [x] Confidence scores sensible (0.0-1.0)
- [x] Level 1/2 reject most false candidates

---

#### Day 7: Safety Monitor

```bash
# Terminal 1: Safety monitor
ros2 run ran_complete ran_safety_monitor.py

# Terminal 2: Subscribe to LiDAR
ros2 launch rplidar_ros rplidar_s3_launch.py

# Terminal 3: Publish test velocity commands
ros2 topic pub /cmd_vel_raw geometry_msgs/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.0}}"

# Terminal 4: Monitor safe commands
ros2 topic echo /cmd_vel
```

**Expected output**:
- Velocities clamped to 0.5 m/s linear, 0.3 rad/s angular
- Acceleration limited to 0.2 m/s²
- Speed scaled near obstacles
- Emergency stop triggered <0.3m

**Test cases**:
1. Command high velocity (1.0 m/s) → clamped to 0.5 m/s
2. Sudden acceleration → limited to 0.2 m/s²
3. Place obstacle in front → speed scaled down
4. Move obstacle very close → emergency stop

**Success criteria**:
- [x] All speed limits enforced
- [x] No jerky movements (smooth acceleration)
- [x] Emergency stop prevents collisions
- [x] User override available

---

### Phase 2: Integrated System Testing (Week 2)

#### Full System Launch

```bash
# Single command launch
ros2 launch ran_complete full_system.launch.py
```

**Monitor nodes**:
```bash
ros2 node list
# Expected:
# /ran_perception
# /ran_mapping
# /ran_verifier
# /ran_navigator
# /ran_safety
# /bt_navigator
# /controller_server
# ...
```

#### Test Single-Step Navigation

```bash
# Instruction: Go to single object
ros2 topic pub /ran/instruction std_msgs/String \
  "data: 'Go to the red chair'" --once

# Monitor status
ros2 topic echo /ran/nav_status
```

**Expected behavior**:
1. Navigator parses instruction → "red chair"
2. Verifier queries map → finds candidate
3. Nav2 plans path → wheelchair starts moving
4. Safety monitor enforces limits
5. Wheelchair arrives at chair
6. Level 4 re-verification confirms correct object
7. Status: COMPLETED

**Success criteria**:
- [x] Wheelchair navigates to correct object
- [x] Doesn't navigate to wrong object (e.g., blue chair)
- [x] Safety monitor prevents collisions
- [x] Re-verification confirms goal

---

#### Test Multi-Step Navigation

```bash
# Instruction: Go to multiple objects sequentially
ros2 topic pub /ran/instruction std_msgs/String \
  "data: 'Go to the red chair then the wooden table'" --once
```

**Expected behavior**:
1. Navigator parses → ["red chair", "wooden table"]
2. Subgoal 1: Navigate to red chair
3. Subgoal 1 verified → COMPLETED
4. Subgoal 2: Navigate to wooden table
5. Subgoal 2 verified → COMPLETED
6. Full chain completed!

**Success criteria**:
- [x] Both subgoals completed in order
- [x] No navigation to wrong objects
- [x] Recovery if verification fails
- [x] Full-chain completion logged

---

### Phase 3: Data Collection (Weeks 3-4)

#### Prepare Test Instructions

Use evaluation dataset:
```bash
cat /home/user/wc/evaluation/data/test_instructions.txt
```

Contains:
- 5 single-step instructions
- 3 two-step instructions
- 2 three-step instructions
- 2 four-step instructions
- 10+ attribute-rich instructions

#### Run Experiments

```bash
cd /home/user/wc/evaluation

# Run full evaluation
python3 run_experiments.py \
  --methods ran \
  --environments home,office,lab \
  --instructions data/test_instructions.txt \
  --trials 5
```

**Data collected**:
- Subgoal success rate (SSR)
- Full-chain completion (FCC)
- Navigation time per instruction
- Safety violations
- Recovery attempts
- Failure modes

#### Record ROS Bags

```bash
# Record all topics for later analysis
ros2 bag record -a -o ran_experiment_1

# Or record specific topics
ros2 bag record \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /scan \
  /odom \
  /cmd_vel \
  /ran/detections \
  /ran/semantic_map \
  /ran/nav_status
```

---

## 📊 Performance Metrics

### Target Metrics (for RSS/ICRA 2026 paper)

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Subgoal Success Rate (SSR)** | 72.2% | #successful_subgoals / #total_subgoals |
| **Full-Chain Completion (FCC)** | 54.3% | #full_chains / #multi_step_instructions |
| **Recovery Rate** | 89.2% | #recovered / #verification_failures |
| **Safety Violations** | 0 | Count of collisions/emergency stops |
| **Avg Navigation Time** | <30s | Time from instruction to goal |

### How to Compute Metrics

After running experiments:

```bash
cd /home/user/wc/evaluation/results/<timestamp>

# Open results.csv in Python/Excel
import pandas as pd

df = pd.read_csv('results.csv')

# SSR
ssr = df['subgoals_succeeded'].sum() / df['subgoals_attempted'].sum()
print(f"SSR: {ssr:.1%}")

# FCC
fcc = (df['full_chain'] == True).sum() / len(df)
print(f"FCC: {fcc:.1%}")

# Safety
violations = df['safety_violations'].sum()
print(f"Safety violations: {violations}")
```

---

## 🧪 Ablation Studies

Run 5 configurations to validate each contribution:

### Configuration 1: Full System (Baseline)
```bash
# All features enabled (in ran_params.yaml)
adaptive_tau_sem: true
hierarchical: true
enable_dynamic_updates: true
use_quality_weighting: true
```

### Configuration 2: w/o Hierarchical Verification
```bash
# Disable 4-level cascade
hierarchical: false
```

### Configuration 3: w/o Uncertainty Estimation
```bash
# Disable quality weighting
use_quality_weighting: false
```

### Configuration 4: w/o Adaptive Thresholds
```bash
# Use fixed threshold
adaptive_tau_sem: false
```

### Configuration 5: w/o Dynamic Updates
```bash
# Disable confidence decay
enable_dynamic_updates: false
```

Run each configuration on same test set and compare results.

---

## 🐛 Troubleshooting Common Issues

### Issue 1: Wheelchair not moving

**Symptoms**: Navigation starts, but wheelchair doesn't move

**Diagnosis**:
```bash
ros2 topic echo /cmd_vel
ros2 topic info /cmd_vel
```

**Solutions**:
- Check wheelchair base is subscribed to `/cmd_vel`
- Verify safety monitor not blocking commands
- Check emergency stop not engaged
- Verify Nav2 controller is running

---

### Issue 2: Wrong object selected

**Symptoms**: Navigates to "blue chair" instead of "red chair"

**Diagnosis**:
```bash
# Check detected attributes
ros2 topic echo /ran/detections

# Check map instances
ros2 topic echo /ran/semantic_map
```

**Solutions**:
- Improve lighting (color extraction needs good light)
- Re-run perception with better camera angle
- Lower `level_2_threshold` (less strict color matching)
- Check CLIP embeddings for color terms

---

### Issue 3: Collisions occurring

**Symptoms**: Wheelchair hits obstacles

**Diagnosis**:
```bash
ros2 topic echo /ran/safety_status
ros2 topic echo /scan
```

**Solutions**:
- Increase `collision_distance` from 0.8m to 1.0m
- Lower `max_linear_velocity` from 0.5 to 0.3 m/s
- Check LiDAR is publishing valid scans
- Verify costmap inflation is working

---

### Issue 4: Low detection confidence

**Symptoms**: Confidence scores always < 0.5

**Diagnosis**:
```bash
# Check visual quality metrics
ros2 topic echo /ran/detections
```

**Solutions**:
- Clean camera lens
- Improve lighting
- Move closer to objects (increase q_size)
- Lower `conf_threshold` from 0.25 to 0.15

---

## 📝 Data Collection Log Template

### Experiment Session Log

**Date**: _______________
**Environment**: _______________
**Trial Number**: _______________

#### Pre-Flight Checks
- [ ] Battery: _____%
- [ ] Validation script passed
- [ ] Emergency stop tested
- [ ] All nodes running

#### Test Instructions Run

| Instruction | SSR | FCC | Time (s) | Notes |
|-------------|-----|-----|----------|-------|
| "Go to red chair" | ✓/✗ | ✓/✗ | ____ | _____________ |
| "Go to red chair then table" | ✓/✗ | ✓/✗ | ____ | _____________ |
| ... | | | | |

#### Issues Encountered
1. _____________________________
2. _____________________________

#### Rosbag Files
- File: `ran_trial_01.db3`
- Size: ____ GB
- Duration: ____ min

---

## 🎯 Next Steps After Deployment

### For Paper Submission

1. **Fill Results Tables** (Week 5)
   - Transfer metrics to LaTeX tables
   - Compare with VLMaps and CapNav baselines
   - Generate ablation study table

2. **Create Figures** (Week 6)
   - Trajectory visualizations
   - Confidence heatmaps
   - Failure case analysis plots

3. **Write Paper Sections** (Weeks 7-10)
   - Introduction (motivation + contributions)
   - Related Work (compare to 20+ papers)
   - Method (4 sections for 4 contributions)
   - Experiments (results + ablations + analysis)
   - Conclusion (summary + future work)

4. **Submit to RSS 2026** (Week 11-12)
   - Deadline: February 1, 2026
   - 8 pages + references
   - Supplementary material (video + code)

### For Real Deployment

1. **User Study** (Post-submission)
   - Recruit wheelchair users
   - Collect usability feedback
   - Measure task completion rates

2. **Long-Term Autonomy**
   - Multi-room navigation
   - Elevator integration
   - Voice interface

3. **Commercialization** (Optional)
   - Patent filing
   - Startup formation
   - Clinical trials

---

## ✅ Final Pre-Deployment Checklist

Before running experiments for paper data:

- [ ] All nodes tested individually
- [ ] Integrated system tested (1+ hour operation)
- [ ] At least 10 test navigations successful
- [ ] Safety monitor prevents all collisions
- [ ] Multi-step navigation working
- [ ] Recovery mechanism tested
- [ ] Rosbag recording working
- [ ] Validation script passes
- [ ] Backup power available
- [ ] Emergency stop accessible
- [ ] Calibration data recorded (camera intrinsics, LiDAR-camera extrinsics)

**If all checkboxes ticked → Ready for data collection!**

---

**Document Status**: Complete
**Maintainer**: Siddharth Tiwari (s24035@students.iitmandi.ac.in)
**Last Updated**: 2025-11-22

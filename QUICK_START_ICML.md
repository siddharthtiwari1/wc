# Quick Start: ML Sensor Fusion for ICML 2026

## TL;DR Decision Matrix

| Factor | ICML 2026 (ML) | RSS 2026 (Robotics) |
|--------|----------------|---------------------|
| **Your current work (RAN)** | ❌ Not ML-focused | ✅ Perfect fit |
| **ML sensor fusion** | ✅ Novel enough | ⚠️ Acceptable but not their focus |
| **Real wheelchair** | ⚠️ Nice but not required | ✅ Huge advantage |
| **Timeline** | 3-4 months (tight) | 2-3 months (have it!) |
| **Acceptance rate** | 20-25% | 25-30% |
| **Impact** | Higher prestige (ML) | Higher for robotics career |

## My Honest Recommendation:

### **Split Strategy** (Best of Both Worlds):

1. **Submit RAN to RSS 2026** (Feb deadline)
   - You already have 3,270 lines of code!
   - Real wheelchair validation
   - Strong hierarchical verification contribution
   - **High acceptance chance**: ~30-35%

2. **Meanwhile, develop ML odometry for ICML 2026** (Feb deadline, same time!)
   - Use your wheelchair data as one test case
   - Add KITTI, EuRoC benchmarks
   - **Two papers from same platform!**

## Week-by-Week Plan (If Doing Both):

### Weeks 1-4: RAN Paper (RSS Focus)
- Test RAN system on wheelchair
- Collect 100+ navigation trials
- Write RSS paper (already have skeleton)
- **Submit to RSS by Feb 1**

### Weeks 1-8: Parallel ICML Work
**Weeks 1-2**: Baseline EKF on KITTI
**Weeks 3-4**: Implement differentiable EKF layer
**Weeks 5-6**: Meta-learning training
**Weeks 7-8**: Write ICML paper
**Submit by Feb 15**

## Minimal Viable ICML Paper (2 Months)

If you only have 2 months, do this:

### Core Idea: "Learned Sensor Reliability for Adaptive Odometry"

```python
# Your current EKF uses FIXED sensor configs:
imu0_config: [false, false, true, ...]  # Binary on/off
odom0_config: [true, true, false, ...]  # Fixed weights

# Learned version:
class SensorWeightPredictor(nn.Module):
    def forward(self, imu_data, wheel_data, terrain_type):
        # Predict soft weights [0, 1] instead of binary
        imu_weight = self.imu_net(imu_data)      # 0.8 on smooth, 0.3 on rough
        wheel_weight = self.wheel_net(wheel_data) # 0.9 on tiles, 0.4 on carpet
        return imu_weight, wheel_weight

# Then use weighted sensor fusion in EKF
```

**Contributions**:
1. Learn when to trust each sensor (vs fixed binary)
2. Adapt to terrain without manual tuning
3. Uncertainty-aware (predict confidence per sensor)

**Datasets**: KITTI + Your wheelchair (5 terrains: tiles, carpet, ramp, outdoor, rough)

**Results Table**:
| Method | KITTI ATE (m) | Wheelchair ATE (m) |
|--------|---------------|-------------------|
| EKF (fixed) | 2.3 | 0.15 |
| IONet | 1.9 | - |
| **Learned Weights (Ours)** | **1.7** | **0.09** |

**Why ICML might accept**:
- Novel: First to learn sensor weights for EKF
- Practical: Works on real wheelchair
- General: Tested on standard KITTI

**Why they might reject**:
- Incremental: "Just adding learning to EKF"
- Limited novelty: No theoretical guarantees

**Acceptance chance**: 15-20%

## The Critical Question: What Do You REALLY Want?

### Choose ICML if:
- You want to work in ML research (PhD at ML lab)
- You care about prestige in ML community
- You're okay with 20% acceptance rate
- You have 3+ months for solid execution

### Choose RSS/ICRA if:
- You want robotics research career
- You have a working robot system (rare!)
- You want higher acceptance chance (30%)
- You care about real-world impact

## Data You Need to Collect (This Week!)

For **ICML** (ML benchmarks required):
```bash
# Download standard datasets
cd ~/datasets
wget https://kitti.is.tue.mpg.de/kitti/odometry/dataset/...  # 22 GB
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/...  # 15 GB

# Collect wheelchair data (5 terrains)
ros2 bag record /imu /wc_control/odom /scan --duration 600  # 10 min per terrain
```

For **RSS** (real robot is the benchmark):
```bash
# Just collect navigation trials
# You already know this!
```

## My Final Recommendation:

### **Do BOTH, Prioritize RSS:**

1. **Week 1-4**: Finish RAN system, submit to **RSS 2026** ← HIGH PRIORITY
   - 30% acceptance chance
   - You're 80% done already!
   - Real robot is your advantage

2. **Week 1-8**: Develop ML odometry, submit to **ICML 2026** ← STRETCH GOAL
   - 20% acceptance chance
   - Requires downloading KITTI, implementing learning
   - Higher risk but higher prestige

3. **Backup plan**: If ICML rejects, add wheelchair results and resubmit to **IROS 2026** (robotics)

**Best outcome**: 2 accepted papers (RSS + ICML) from same wheelchair platform!
**Realistic outcome**: 1 accepted paper (RSS), ICML experience gained
**Worst outcome**: RSS accepted, ICML rejected → still 1 top-tier paper!

## Action Items (TODAY):

### If going for ICML:
- [ ] Download KITTI dataset (22 GB)
- [ ] Read 3 key papers:
  - "How to Train Your Differentiable Filter" (Kloss 2021)
  - "IONet" (Chen ICRA 2018)
  - "MAML" (Finn ICML 2017)
- [ ] Implement baseline EKF on KITTI (1 week)

### If focusing on RSS:
- [ ] Test RAN system end-to-end on wheelchair
- [ ] Collect 100+ navigation instructions
- [ ] Start writing experiments section

### If doing both:
- [ ] Test RAN (high priority)
- [ ] Download KITTI in background
- [ ] Allocate: 60% time RSS, 40% time ICML

## Final Reality Check:

**ICML is HARD.** Acceptance rate ~20%, reviewers expect:
1. Novel ML algorithm (not just applying existing NN)
2. Theoretical contributions (convergence proofs, PAC bounds)
3. Strong baselines on standard datasets
4. Clear positioning in ML literature

**RSS is ACHIEVABLE.** You have:
1. ✅ Working robot system
2. ✅ Novel hierarchical verification (54% vs 20% baseline)
3. ✅ Real-world validation
4. ✅ 3,270 lines of code already written!

**My advice**:
- **Definitely do RSS** (you're ready!)
- **Attempt ICML** only if you have bandwidth and are okay with rejection risk
- **Don't let ICML distract from RSS** (RSS is your sure bet!)

---

**Which path resonates with you? Let me know and I'll help you start coding immediately!**

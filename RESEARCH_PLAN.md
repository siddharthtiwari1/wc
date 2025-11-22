# Real-World Attribute-Aware Navigation with Description-First Mapping
## Research Plan for Top-Tier Conference Submission (ICRA/RSS/CoRL 2026)

---

## 1. CRITICAL WEAKNESSES IN CURRENT SOTA (Your Competitive Advantage)

### CapNav (ICLR 2026 Submission - Reviews show major flaws):
**Strengths we'll keep:**
- Attribute-based grounding (color, shape, texture as first-class fields)
- Multi-view consistency via DINOv2 + CLIP
- Pre-goal verification stage

**CRITICAL WEAKNESSES TO EXPLOIT:**
1. ✗ **Single scene evaluation** - only tested on ONE Matterport3D scene
2. ✗ **Simulation only** - no real robot, no sim-to-real transfer analysis
3. ✗ **One baseline comparison** - only compared to VLMaps
4. ✗ **No ablation studies** - can't isolate component contributions
5. ✗ **Poor long-horizon performance** - only 20% full-chain completion
6. ✗ **Static scene assumption** - no dynamic object handling
7. ✗ **Fixed thresholds** - manually tuned, not adaptive
8. ✗ **No uncertainty quantification** - fails silently in ambiguous cases

### VLMaps (Baseline):
- ✓ Real robot experiments
- ✗ No explicit attribute handling
- ✗ Implicit feature embeddings only

### O3D-SIM:
- ✓ Instance-level 3D mapping
- ✗ No attribute verification
- ✗ Category-focused, not description-focused

---

## 2. YOUR NOVEL CONTRIBUTIONS (Solo First-Author Paper)

### Core Thesis:
**"Bridging Sim-to-Real for Attribute-Aware Navigation: A Real-Robot System with Uncertainty-Aware Description Verification"**

### Key Innovations:

#### A. Real Robot Implementation (PRIMARY NOVELTY)
- **First real-robot system** with explicit attribute verification
- RPLidar S3 for robust localization
- RealSense D455 for RGB-D perception
- Handle real-world noise: lighting variation, sensor errors, dynamic obstacles

#### B. Uncertainty-Aware Verification
- **Novel**: Probabilistic attribute confidence scores
- Multi-hypothesis tracking for ambiguous objects
- Adaptive threshold calibration based on scene complexity
- Explicit "I don't know" responses when confidence < threshold

#### C. Long-Horizon Navigation via Hierarchical Verification
- **Address CapNav's 20% failure**: Hierarchical subgoal verification
- Error recovery with backtracking
- Incremental map updates during navigation
- Multi-round verification for uncertain attributes

#### D. Multi-Scene Generalization
- Evaluation on 5+ diverse real-world environments:
  - Homes (cluttered, similar objects)
  - Offices (repetitive furniture)
  - Labs (equipment-heavy)
  - Public spaces (open, sparse)
- Domain adaptation techniques for cross-environment transfer

#### E. Dynamic Scene Handling
- **Novel**: Online map updates when objects move
- Change detection via temporal consistency
- Re-verification when scene changes detected
- Handling of "object not found" vs "object moved"

#### F. Comprehensive Baselines & Ablations
- Compare against: VLMaps, O3D-SIM, ORB-SLAM3 + GPT-4V, LM-Nav
- Ablation studies:
  - w/o attribute verification
  - w/o multi-view fusion
  - w/o uncertainty estimation
  - CLIP-only vs DINOv2+CLIP

---

## 3. SYSTEM ARCHITECTURE

### Hardware Stack (Your Advantage):
```
┌─────────────────────────────────────┐
│   Mobile Robot Platform             │
├─────────────────────────────────────┤
│  • RPLidar S3 (360° 2D LiDAR)      │ → Localization (AMCL/Cartographer)
│  • RealSense D455 (RGB-D Camera)   │ → Perception (640x480@30fps)
│  • Robust Odometry                  │ → Dead reckoning
│  • Compute: Jetson Xavier NX        │ → Onboard inference
│    or Laptop (RTX 3090)             │
└─────────────────────────────────────┘
```

### Software Stack (ROS2 Humble):
```
┌────────────────────────────────────────────────────┐
│                 NAVIGATION LAYER                   │
│  • Nav2 for path planning (A*)                     │
│  • Attribute-aware goal verification              │
│  • Multi-round verification for uncertain cases   │
└────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────┐
│              LANGUAGE GROUNDING LAYER              │
│  • LLM Parser (Llama-3.1-70B / GPT-4)             │
│  • Attribute extraction: category + color +        │
│    shape + texture + spatial relations            │
│  • Multi-modal retrieval (CLIP + LongCLIP)        │
└────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────┐
│            3D SEMANTIC MAPPING LAYER               │
│  • YOLO-World for open-vocab detection            │
│  • SAM2 for segmentation                          │
│  • DINOv2 + CLIP for features                     │
│  • Multi-view fusion with uncertainty             │
│  • Attribute fields: {color, shape, material,     │
│    position, confidence}                          │
└────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────┐
│           PERCEPTION & LOCALIZATION                │
│  • Cartographer (2D SLAM with RPLidar S3)         │
│  • RealSense D455 RGB-D processing                │
│  • TF2 for coordinate transforms                  │
└────────────────────────────────────────────────────┘
```

---

## 4. TECHNICAL APPROACH (Improvements over CapNav)

### Stage 1: Robust Perception with Uncertainty
```python
# For each RealSense frame:
# 1. Detect objects (YOLO-World with custom vocabulary)
# 2. Segment with SAM2
# 3. Extract features: DINOv2 (visual) + CLIP (semantic)
# 4. **NEW**: Compute per-attribute confidence scores
#    - Color confidence: variance across views
#    - Shape confidence: viewpoint coverage
#    - Material confidence: lighting consistency
```

**Novel Contribution**: Explicit confidence estimation per attribute, not just per-object.

### Stage 2: Multi-View Fusion with Adaptive Thresholds
```python
# CapNav uses fixed thresholds (τ_sem=0.65, τ_vol=0.15)
# YOUR IMPROVEMENT: Adaptive thresholds based on:
# - Scene clutter (more objects → higher threshold)
# - Attribute ambiguity (similar colors → require more views)
# - Sensor noise (depth uncertainty → relax geometric constraints)
```

**Novel Contribution**: Self-calibrating clustering that adapts to scene complexity.

### Stage 3: Hierarchical Verification for Long-Horizon Nav
```python
# CapNav achieves only 20% full-chain completion
# YOUR IMPROVEMENT: Multi-level verification
# Level 1 (Coarse): Is category correct? → Fast rejection
# Level 2 (Medium): Are salient attributes present? → Quick check
# Level 3 (Fine): All attributes match? → Full verification
# Level 4 (Re-verify): Approach and re-observe → Confirm before goal
```

**Novel Contribution**: Verification cascade reduces false positives while maintaining speed.

### Stage 4: Dynamic Map Updates
```python
# Monitor for changes:
# - Compare current observations to map
# - Detect moved/removed/added objects
# - Update instance attributes if confidence increases
# - Mark stale instances with decay
```

**Novel Contribution**: First attribute-aware system with online map updates.

---

## 5. EVALUATION PLAN (Address Reviewer Concerns)

### Datasets & Environments:

#### Real Robot (PRIMARY):
1. **Home Environment** (3 homes, 150+ objects)
   - Multi-object navigation: "go to the red mug on the wooden table"
   - Distractors: similar mugs in different rooms

2. **Office Environment** (2 offices, 100+ objects)
   - Repetitive furniture: "go to the black chair with armrests near the window"

3. **Lab Environment** (1 lab, 80+ objects)
   - Technical equipment with fine-grained attributes

4. **Public Space** (1 lobby, 50+ objects)
   - Open space, sparse objects

5. **Dynamic Environment** (1 home)
   - Objects moved between trials

#### Simulation (for scaling):
- Habitat with 20 Matterport3D scenes (not just 1 like CapNav)
- Sim-to-real transfer analysis

### Metrics:

#### Success Metrics:
- **Subgoal Success Rate (SSR)**: % of correct individual subgoals
- **Full-Chain Completion (FCC)**: % of complete multi-step instructions
  - Target: >50% (vs CapNav's 20%)
- **Attribute Precision/Recall**: Did we match ALL specified attributes?
- **False Positive Rate**: Wrong object with similar attributes

#### Robustness Metrics:
- **Sim-to-Real Gap**: Performance drop from Habitat → Real
- **Cross-Environment Transfer**: Train on homes, test on offices
- **Dynamic Robustness**: Performance when objects moved

#### Efficiency Metrics:
- **Map Build Time**: Time to construct semantic map
- **Query Latency**: Time from instruction to goal selection
- **Path Efficiency**: Ratio of actual/optimal path length

### Baselines:
1. **VLMaps** (real robot) - category-only baseline
2. **O3D-SIM** (adapted for real robot) - instance-level baseline
3. **LM-Nav** (if code available) - LLM-based baseline
4. **CapNav** (sim only) - their method in simulation
5. **Ours (sim)** - your method in simulation for fair comparison
6. **Ours (real)** - your method on real robot

### Ablation Studies:
| Component Removed | Expected Impact |
|-------------------|----------------|
| Multi-view fusion | -15% SSR (noisy single-view attributes) |
| Uncertainty estimation | -20% SSR (confident on wrong objects) |
| Hierarchical verification | -25% FCC (false positives propagate) |
| Adaptive thresholds | -10% in cluttered scenes |
| Dynamic updates | -30% when objects move |

---

## 6. PAPER STRUCTURE (ICRA/RSS 8 pages)

### Title:
**"Uncertainty-Aware Attribute Navigation: Real-Robot Description-First Mapping with Hierarchical Verification"**

or

**"From Descriptions to Destinations: Real-World Attribute-Guided Navigation with Confidence Estimation"**

### Abstract (250 words):
```
Language-guided navigation demands robots interpret fine-grained descriptions
("go to the red mug on the wooden table") beyond category labels. While recent
sim-based methods demonstrate attribute-aware retrieval, real-world deployment
faces three challenges: (1) sensor noise corrupts attribute estimates,
(2) ambiguous scenes require uncertainty quantification, (3) dynamic
environments invalidate static maps.

We present [SYSTEM_NAME], a real-robot navigation system with explicit
attribute verification and uncertainty-aware decision making. Our key insight:
attributes carry varying confidence levels—color estimates degrade with
lighting, shape requires multi-view fusion—demanding per-attribute uncertainty
over monolithic embeddings.

We introduce: (i) confidence-weighted attribute fusion aggregating multi-view
observations with quality scores, (ii) hierarchical verification cascading
coarse-to-fine checks to balance speed and accuracy, (iii) adaptive threshold
calibration tuning similarity bounds to scene complexity, and (iv) online map
updates tracking moved objects.

Experiments on a mobile robot (RPLidar S3 + RealSense D455) across five
real-world environments (homes, offices, labs) show [X]% subgoal success and
[Y]% full-chain completion on 100+ multi-step instructions—outperforming
VLMaps by [Z]% while explicitly handling ambiguity. Sim-to-real transfer from
Habitat to real homes drops performance by only [W]%, and our system recovers
from [V]% of object movements. Ablations confirm uncertainty estimation
contributes [U]% of gains.
```

### Sections:

**1. Introduction** (1 page)
- Motivation: Language ≠ categories, need attributes
- Challenge: Sim-to-real gap for attribute perception
- Contributions (bullet list, 4-5 items)

**2. Related Work** (0.8 pages)
- Vision-language navigation (R2R, RoomR, EmbCLIP)
- Open-vocab 3D mapping (VLMaps, ConceptFusion, LERF)
- Instance-level methods (O3D-SIM, OpenMask3D)
- *Position CapNav here*: "concurrent work demonstrates attribute-aware sim
  navigation but lacks real-robot validation and uncertainty handling"

**3. Problem Formulation** (0.3 pages)
- Input: Natural language + RGB-D stream + poses
- Output: Navigation trajectory satisfying all attributes
- Metrics: SSR, FCC, attribute precision/recall

**4. Method** (3 pages)
- 4.1 Confidence-Weighted Attribute Fusion
- 4.2 Hierarchical Verification Cascade
- 4.3 Adaptive Threshold Calibration
- 4.4 Online Map Updates
- Figure: System architecture diagram

**5. Experiments** (2 pages)
- 5.1 Setup (hardware, environments, baselines)
- 5.2 Quantitative Results (tables: SSR/FCC per environment)
- 5.3 Ablation Studies (bar charts)
- 5.4 Qualitative Analysis (failure cases, uncertainty visualization)
- 5.5 Sim-to-Real Transfer Analysis

**6. Conclusion** (0.2 pages)
- Summary + future work (active exploration, manipulation)

**7. References** (0.7 pages)
- 30-40 references (heavy on recent VLN, open-vocab, real robots)

---

## 7. IMPLEMENTATION ROADMAP (12 Weeks to Submission)

### Weeks 1-2: Infrastructure
- [ ] Set up ROS2 workspace with RPLidar S3 + RealSense D455
- [ ] Calibrate sensors, verify TF tree
- [ ] Data collection: Record 5 rosbags in different environments
- [ ] Set up Habitat simulator for sim experiments

### Weeks 3-4: Perception Pipeline
- [ ] YOLO-World integration for open-vocab detection
- [ ] SAM2 for segmentation
- [ ] DINOv2 + CLIP feature extraction
- [ ] **NEW**: Confidence estimation module

### Weeks 5-6: Mapping & Fusion
- [ ] Multi-view instance clustering (improve CapNav's method)
- [ ] **NEW**: Adaptive threshold calibration
- [ ] Attribute field extraction (color, shape, material)
- [ ] 3D map representation (JSON + pointclouds)

### Weeks 7-8: Navigation & Verification
- [ ] Language parser (LLM-based, extract attributes)
- [ ] **NEW**: Hierarchical verification cascade
- [ ] Nav2 integration for path planning
- [ ] **NEW**: Dynamic map updates

### Weeks 9-10: Evaluation
- [ ] Implement all baselines (VLMaps, O3D-SIM on real robot)
- [ ] Run 100+ instructions across 5 environments
- [ ] Collect metrics: SSR, FCC, false positives
- [ ] Ablation experiments (remove each component)

### Weeks 11-12: Paper Writing & Polishing
- [ ] Write all sections
- [ ] Generate figures (architecture, results, qualitative)
- [ ] Internal review and revisions
- [ ] Submission (ICRA deadline: Sep 15, RSS: Feb 1, CoRL: May 31)

---

## 8. CODE REPOSITORY STRUCTURE

```
real_attribute_navigation/
├── README.md                          # Top-notch documentation
├── docs/
│   ├── INSTALLATION.md
│   ├── HARDWARE_SETUP.md
│   ├── EXPERIMENTS.md
│   └── API.md
├── src/
│   ├── ran_perception/                # ROS2 package: perception
│   │   ├── nodes/
│   │   │   ├── yolo_detector.py       # YOLO-World detection
│   │   │   ├── sam_segmenter.py       # SAM2 segmentation
│   │   │   ├── feature_extractor.py   # DINOv2 + CLIP
│   │   │   └── confidence_estimator.py # NEW: Uncertainty quantification
│   │   └── launch/
│   │       └── perception.launch.py
│   ├── ran_mapping/                   # ROS2 package: 3D mapping
│   │   ├── nodes/
│   │   │   ├── instance_clusterer.py  # Multi-view fusion
│   │   │   ├── attribute_extractor.py # Color/shape/material
│   │   │   ├── map_builder.py         # 3D map construction
│   │   │   └── dynamic_updater.py     # NEW: Online updates
│   │   └── launch/
│   │       └── mapping.launch.py
│   ├── ran_navigation/                # ROS2 package: navigation
│   │   ├── nodes/
│   │   │   ├── language_parser.py     # LLM-based parsing
│   │   │   ├── goal_retriever.py      # Attribute-based retrieval
│   │   │   ├── hierarchical_verifier.py # NEW: Cascade verification
│   │   │   └── nav_controller.py      # Nav2 interface
│   │   └── launch/
│   │       └── navigation.launch.py
│   └── ran_bringup/                   # ROS2 package: system launch
│       ├── launch/
│       │   ├── robot.launch.py        # RPLidar + RealSense
│       │   └── full_system.launch.py  # All nodes
│       └── config/
│           ├── robot.yaml
│           └── nav2_params.yaml
├── evaluation/
│   ├── baselines/
│   │   ├── vlmaps_real.py             # VLMaps for real robot
│   │   ├── o3dsim_real.py             # O3D-SIM adaptation
│   │   └── lmnav_real.py
│   ├── metrics/
│   │   ├── success_rate.py
│   │   ├── attribute_precision.py
│   │   └── sim2real_gap.py
│   └── scripts/
│       ├── run_all_experiments.sh
│       └── generate_plots.py
├── paper/
│   ├── main.tex                       # LaTeX source
│   ├── sections/
│   │   ├── intro.tex
│   │   ├── related.tex
│   │   ├── method.tex
│   │   ├── experiments.tex
│   │   └── conclusion.tex
│   ├── figures/
│   └── references.bib
├── data/
│   ├── rosbags/                       # Recorded sensor data
│   ├── maps/                          # 3D semantic maps
│   └── instructions/                  # Evaluation queries
└── requirements.txt
```

---

## 9. TARGET CONFERENCES (Ranked by Fit)

### Tier 1 (Best Fit):
1. **RSS 2026** (Robotics: Science and Systems)
   - Deadline: ~Feb 1, 2026
   - Acceptance: ~25%
   - **Perfect fit**: Real robot, challenging scenarios, systems work

2. **ICRA 2026** (Int'l Conf on Robotics and Automation)
   - Deadline: Sep 15, 2025
   - Acceptance: ~40%
   - **Good fit**: Solid systems paper, real-world validation

3. **CoRL 2026** (Conference on Robot Learning)
   - Deadline: ~May 31, 2026
   - Acceptance: ~30%
   - **Good fit**: Learning-based perception, language grounding

### Tier 2 (Backup):
4. **IROS 2026** (Int'l Conf on Intelligent Robots and Systems)
5. **CVPR 2026** (vision track, if you emphasize 3D vision)

---

## 10. KEY DIFFERENTIATORS FROM CAPNAV (Reviewer Rebuttals)

| CapNav Weakness | Your Solution | Impact |
|----------------|---------------|--------|
| Single scene eval | 5 real + 20 sim scenes | +Generalization proof |
| Simulation only | Real robot (RPLidar+RealSense) | +Deployability |
| One baseline | 4-5 baselines | +Rigorous comparison |
| No ablations | 5 ablation studies | +Component contribution |
| 20% full-chain | Target >50% via hierarchy | +Practical utility |
| Static scenes | Dynamic updates | +Real-world robustness |
| Fixed thresholds | Adaptive calibration | +Scene adaptability |
| No uncertainty | Confidence per attribute | +Failure awareness |

---

## 11. WRITING TIPS FOR TOP CONFERENCES

### What Reviewers Want:
1. **Clear problem statement**: What exactly is hard?
2. **Quantifiable contributions**: Not "better", but "X% better"
3. **Honest failure analysis**: When does it break? Why?
4. **Reproducibility**: Code, data, hyperparameters
5. **Real-world validation**: Sim is not enough for RSS/ICRA

### Common Rejection Reasons:
- ❌ "Engineering exercise" (just integrating existing tools)
  - **Your defense**: Novel uncertainty estimation + hierarchical verification
- ❌ "Limited evaluation" (only sim or one environment)
  - **Your defense**: 5 real environments + 20 sim + baselines
- ❌ "Incremental over X"
  - **Your defense**: First real-robot attribute navigation with uncertainty

### Presentation:
- **Figures**: High-quality, colorblind-friendly, large fonts
- **Tables**: Bold best results, show std deviations
- **Writing**: Active voice, concise, precise claims
- **Video**: Supplement with real-robot demo video (critical for RSS/ICRA)

---

## 12. NEXT STEPS (THIS WEEK)

1. **Finalize target conference**: RSS 2026 (Feb 1 deadline) or ICRA 2026 (Sep 15)
2. **Set up ROS2 workspace**: Get RPLidar + RealSense running
3. **Collect initial data**: Record 1-2 rosbags in your lab/home
4. **Implement baseline**: Get VLMaps running on your robot
5. **Start LaTeX paper**: Create skeleton with sections

**Immediate action**: Let's start building the ROS2 package structure and get your hardware running!

---

## 13. ESTIMATED TIMELINE TO SUBMISSION

| Phase | Duration | Deadline |
|-------|----------|----------|
| Infrastructure setup | 2 weeks | Week 2 |
| Perception pipeline | 2 weeks | Week 4 |
| Mapping & fusion | 2 weeks | Week 6 |
| Navigation & verification | 2 weeks | Week 8 |
| Evaluation & baselines | 2 weeks | Week 10 |
| Paper writing | 2 weeks | Week 12 |
| **Buffer** | 2 weeks | Week 14 |

**Target submission**: 3.5 months from now → **ICRA Sep 15** or **RSS Feb 1**

If starting now (late May 2025) → Target **RSS Feb 1, 2026** (8 months)

---

**Ready to start? Let's build this system step by step! Which component should we tackle first?**

Options:
A. Set up ROS2 workspace + hardware integration
B. Implement perception pipeline (YOLO + SAM + CLIP)
C. Start LaTeX paper skeleton
D. Create detailed system architecture diagram

**Let me know and we'll start coding immediately!**

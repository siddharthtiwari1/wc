# Dense Crowd Navigation Research & Implementation Roadmap
**Project**: Wheelchair Crowd Navigation System
**Target**: ROS2 Jazzy
**Date**: 2025-11-22
**Status**: Planning Phase

---

## Executive Summary

This roadmap outlines the research and implementation of two ROS2 Jazzy packages for dense crowd navigation:

1. **wheelchair_crowd_navigation** - Recreation of CrowdSurfer (VQVAE + Sampling Optimization)
2. **wheelchair_adaptive_navigation** - Novel approach combining Diffusion Models with Model Predictive Control

---

## Part I: Background Research

### 1.1 CrowdSurfer Analysis

**Core Innovation**: Hybrid learning + optimization approach
- **Learning Phase**: VQVAE learns trajectory priors from expert demonstrations
- **Runtime Phase**: Sampling-based optimizer refines trajectories using learned priors
- **Key Advantage**: 40% success rate improvement, 6% travel time reduction vs DRL-VO

**Technical Stack**:
- Vector Quantized VAE (VQVAE) for trajectory encoding
- PixelCNN for autoregressive prior modeling
- JAX-based sampling optimizer for inference-time refinement
- No explicit pedestrian prediction required

**Limitations Identified**:
1. ROS1 dependency (needs ROS2 migration)
2. Requires large expert demonstration datasets
3. VQVAE discrete latent space may limit trajectory diversity
4. Fixed horizon planning (may not adapt to varying crowd densities)

### 1.2 State-of-the-Art Review

**Current Approaches**:

| Method | Type | Strengths | Weaknesses |
|--------|------|-----------|------------|
| DRL-VO | Deep RL | Real-time, learned policy | Sample inefficient, poor generalization |
| Social Forces | Physics-based | Interpretable, no training | Poor in dense crowds |
| ORCA/RVO | Optimization | Collision-free guarantees | Assumes cooperative agents |
| CADRL/SARL | Deep RL | Handles dynamics | Limited to sparse crowds |
| CrowdSurfer | Hybrid | Best performance | Complex training pipeline |

**Research Gaps**:
1. Limited adaptation to varying crowd densities
2. Lack of uncertainty quantification in trajectory generation
3. Insufficient integration with global planning
4. Poor handling of social group dynamics

---

## Part II: Package 1 - CrowdSurfer Recreation

### 2.1 Package Overview

**Name**: `wheelchair_crowd_navigation`
**Type**: ROS2 Jazzy package
**Language**: Python 3.10+ (ML components), C++ (ROS2 nodes)
**Goal**: Faithful recreation with ROS2 integration

### 2.2 Architecture Design

```
wheelchair_crowd_navigation/
├── wheelchair_crowd_navigation/
│   ├── models/
│   │   ├── vqvae.py              # Vector Quantized VAE
│   │   ├── pixelcnn.py           # Autoregressive prior
│   │   ├── encoder.py            # Perception encoder
│   │   └── decoder.py            # Trajectory decoder
│   ├── planning/
│   │   ├── sampling_optimizer.py # JAX-based optimizer
│   │   ├── trajectory_scorer.py  # Cost function evaluator
│   │   └── collision_checker.py  # Safety verification
│   ├── perception/
│   │   ├── lidar_processor.py    # Point cloud processing
│   │   ├── crowd_detector.py     # Pedestrian detection
│   │   └── feature_extractor.py  # Perception features
│   ├── ros2/
│   │   ├── crowd_planner_node.py # Main ROS2 node
│   │   ├── visualization_node.py # RViz2 visualization
│   │   └── bridge.py             # ROS2 message handling
│   └── training/
│       ├── data_collector.py     # Demonstration collection
│       ├── trainer.py            # VQVAE training
│       └── evaluator.py          # Metrics computation
├── config/
│   ├── vqvae_config.yaml         # Model hyperparameters
│   ├── optimizer_config.yaml     # Sampling optimizer config
│   └── perception_config.yaml    # Sensor configuration
├── launch/
│   ├── crowd_navigation.launch.py
│   ├── training.launch.py
│   └── evaluation.launch.py
├── models/                       # Pretrained weights
├── data/                         # Training datasets
├── tests/
└── README.md
```

### 2.3 Implementation Phases

**Phase 1: Core Models (Weeks 1-2)**
- [ ] Implement VQVAE architecture (encoder, decoder, codebook)
- [ ] Implement PixelCNN prior network
- [ ] Create perception feature extractor
- [ ] Unit tests for all model components

**Phase 2: Training Pipeline (Weeks 2-3)**
- [ ] Data collection utilities from simulation/real robot
- [ ] VQVAE training loop with reconstruction loss
- [ ] Codebook learning and quantization
- [ ] Tensorboard logging and checkpointing

**Phase 3: Sampling Optimizer (Weeks 3-4)**
- [ ] JAX-based trajectory sampling
- [ ] Cost function implementation (collision, smoothness, goal-reaching)
- [ ] Cross-entropy method or gradient-based optimization
- [ ] Integration with VQVAE prior

**Phase 4: ROS2 Integration (Weeks 4-5)**
- [ ] ROS2 node architecture
- [ ] Subscribe to /scan (LaserScan) or /points (PointCloud2)
- [ ] Subscribe to /cmd_vel_nav (Twist) for goal
- [ ] Publish to /local_plan (Path)
- [ ] Publish to /crowd_visualization (MarkerArray)
- [ ] Action server for navigation goals

**Phase 5: Testing & Evaluation (Weeks 5-6)**
- [ ] Simulation environment setup (Gazebo + pedsim)
- [ ] Benchmark scenarios (varying crowd densities)
- [ ] Metrics: success rate, travel time, path length, discomfort
- [ ] Comparison with baseline planners

### 2.4 Technical Requirements

**Dependencies**:
```yaml
# Python packages
torch>=2.0.0
jax[cuda12]>=0.4.0
flax>=0.8.0
optax>=0.2.0
hydra-core>=1.3.2
tensorboard>=2.15.0
open3d>=0.18.0
numpy>=1.24.0
scipy>=1.11.0

# ROS2 packages
rclpy
nav_msgs
geometry_msgs
sensor_msgs
visualization_msgs
tf2_ros
nav2_msgs
```

**Hardware Requirements**:
- NVIDIA GPU with CUDA 12.x support
- 16GB+ RAM
- 8GB+ VRAM for training

### 2.5 Key Algorithms

**VQVAE Forward Pass**:
```
Input: Perception features P, Goal G
1. Encode: z_e = Encoder(P, G)
2. Quantize: z_q = Codebook.lookup(argmin_k ||z_e - e_k||)
3. Decode: Trajectory = Decoder(z_q, G)
4. Loss: L_recon + L_commitment + L_codebook
```

**Sampling Optimizer**:
```
Input: VQVAE model, Current state s, Goal g
1. Sample K codes from PixelCNN prior
2. Decode to K candidate trajectories
3. For iteration i in [1, N]:
   a. Score trajectories with cost function
   b. Select top-M elite trajectories
   c. Fit distribution to elites
   d. Sample new K trajectories
4. Return best trajectory
```

---

## Part III: Package 2 - Novel Approach

### 3.1 Innovation: Diffusion-based Adaptive Navigation

**Name**: `wheelchair_adaptive_navigation`
**Type**: ROS2 Jazzy package
**Goal**: Novel approach addressing CrowdSurfer limitations

### 3.2 Core Innovations

**1. Denoising Diffusion Probabilistic Model (DDPM)**
- Replace VQVAE with continuous diffusion model
- Better trajectory diversity and quality
- Conditional generation on crowd density, robot state, goal

**2. Uncertainty-Aware Planning**
- Ensemble of trajectories with uncertainty bounds
- Risk-aware cost function
- Adaptive horizon based on uncertainty

**3. Hierarchical Planning**
- Global: Topological planning with crowd flow prediction
- Local: Diffusion-based trajectory generation
- Reflex: MPC for real-time obstacle avoidance

**4. Social Group Modeling**
- Explicit detection of social groups (F-formations)
- Group-aware cost function (avoid splitting groups)
- Learned social constraints

### 3.3 Architecture Design

```
wheelchair_adaptive_navigation/
├── wheelchair_adaptive_navigation/
│   ├── models/
│   │   ├── diffusion/
│   │   │   ├── unet_trajectory.py    # 1D U-Net for trajectories
│   │   │   ├── diffusion_model.py    # DDPM implementation
│   │   │   ├── noise_scheduler.py    # Variance schedule
│   │   │   └── conditional_encoder.py # Context encoding
│   │   ├── social/
│   │   │   ├── group_detector.py     # F-formation detection
│   │   │   ├── flow_predictor.py     # Crowd flow estimation
│   │   │   └── interaction_model.py  # Social interactions
│   │   └── uncertainty/
│   │       ├── ensemble.py           # Trajectory ensemble
│   │       └── risk_estimator.py     # Uncertainty quantification
│   ├── planning/
│   │   ├── hierarchical/
│   │   │   ├── global_planner.py     # Topological planning
│   │   │   ├── local_planner.py      # Diffusion-based
│   │   │   └── reflex_controller.py  # MPC backup
│   │   ├── adaptive_horizon.py       # Dynamic planning horizon
│   │   ├── risk_aware_cost.py        # Uncertainty-aware costs
│   │   └── social_cost.py            # Group-aware costs
│   ├── perception/
│   │   ├── crowd_analyzer.py         # Density estimation
│   │   ├── tracking.py               # Multi-target tracking
│   │   └── scene_encoder.py          # BEV representation
│   ├── ros2/
│   │   ├── adaptive_planner_node.py  # Main node
│   │   ├── hierarchy_manager.py      # Planning mode switching
│   │   └── social_viz_node.py        # Social groups viz
│   └── training/
│       ├── diffusion_trainer.py      # DDPM training
│       ├── flow_trainer.py           # Flow prediction
│       └── sim_data_gen.py           # Synthetic data
├── config/
│   ├── diffusion_config.yaml
│   ├── social_config.yaml
│   ├── hierarchy_config.yaml
│   └── uncertainty_config.yaml
├── launch/
│   ├── adaptive_navigation.launch.py
│   ├── training_diffusion.launch.py
│   └── sim_evaluation.launch.py
├── models/
├── data/
├── tests/
└── README.md
```

### 3.4 Implementation Phases

**Phase 1: Diffusion Model (Weeks 1-3)**
- [ ] 1D U-Net architecture for trajectory generation
- [ ] DDPM forward/reverse diffusion process
- [ ] Conditional encoding (robot state, goal, crowd)
- [ ] Training pipeline with demonstration data
- [ ] Sampling algorithms (DDPM, DDIM)

**Phase 2: Social Modeling (Weeks 3-4)**
- [ ] F-formation detection (Hough voting, clustering)
- [ ] Social group tracking
- [ ] Crowd flow prediction network
- [ ] Social cost function (proxemics, group coherence)

**Phase 3: Hierarchical Planning (Weeks 4-5)**
- [ ] Global topological planner (graph-based)
- [ ] Local diffusion planner (DDPM sampling)
- [ ] MPC reflex controller (backup)
- [ ] Mode switching logic (crowd density-based)

**Phase 4: Uncertainty Quantification (Weeks 5-6)**
- [ ] Trajectory ensemble generation
- [ ] Uncertainty propagation
- [ ] Risk-aware cost computation
- [ ] Adaptive planning horizon

**Phase 5: ROS2 & Evaluation (Weeks 6-8)**
- [ ] Full ROS2 node implementation
- [ ] Integration with existing wheelchair stack
- [ ] Simulation evaluation (Gazebo + pedsim)
- [ ] Real-world testing preparation
- [ ] Comparative benchmarking

### 3.5 Technical Requirements

**Additional Dependencies**:
```yaml
# Diffusion models
diffusers>=0.25.0
accelerate>=0.26.0

# Social modeling
scikit-learn>=1.3.0
filterpy>=1.4.5  # Kalman filtering for tracking

# Optimization
cvxpy>=1.4.0     # Convex optimization for MPC
casadi>=3.6.0    # Nonlinear optimization
```

### 3.6 Novel Algorithms

**Diffusion Trajectory Generation**:
```
Training:
1. Sample trajectory T from demonstrations
2. Sample timestep t ~ Uniform(0, T_max)
3. Add noise: T_t = sqrt(alpha_t) * T + sqrt(1-alpha_t) * epsilon
4. Predict noise: epsilon_pred = Model(T_t, t, context)
5. Loss: MSE(epsilon, epsilon_pred)

Sampling:
1. Start from noise: T_T ~ N(0, I)
2. For t = T_max to 0:
   a. Predict noise: epsilon = Model(T_t, t, context)
   b. Denoise: T_{t-1} = denoise_step(T_t, epsilon, t)
3. Return trajectory T_0
```

**Risk-Aware Planning**:
```
Input: Ensemble of N trajectories {T_1, ..., T_N}
1. Compute mean trajectory: T_mean
2. Compute uncertainty: Sigma(s) = Var({T_i(s)})
3. Risk cost: R(T) = integral(lambda * ||Sigma(s)||) ds
4. Total cost: C(T) = C_nominal(T) + R(T)
5. Select trajectory minimizing C(T)
```

**Adaptive Horizon**:
```
Input: Crowd density rho, Uncertainty sigma
1. Base horizon: H_base = 3.0 seconds
2. Density factor: f_d = max(0.5, 1.0 - rho/rho_max)
3. Uncertainty factor: f_u = min(1.5, 1.0 + sigma/sigma_threshold)
4. Adaptive horizon: H = H_base * f_d * f_u
Output: Planning horizon in [1.5s, 4.5s]
```

---

## Part IV: Integration Strategy

### 4.1 Existing Wheelchair Stack Integration

**Current Architecture**:
```
wheelchair_bringup
  └─> wheelchair_firmware (hardware interface)
  └─> wheelchair_localization (EKF, AMCL)
  └─> wheelchair_navigation (Nav2 stack)
  └─> wheelchair_planning (global planner)
  └─> wc_control (controllers)
```

**Integration Points**:

1. **Replace Nav2 Local Planner**:
   - Current: DWB or TEB controller
   - New: CrowdSurfer or Adaptive planner as Nav2 plugin

2. **Perception Layer**:
   - Input: LaserScan from wheelchair sensors
   - Input: PointCloud2 if 3D lidar available
   - Input: Odometry from wheelchair_localization
   - Processing: Crowd detection and tracking

3. **Global Planner Coordination**:
   - Receive global path from wheelchair_planning
   - Extract local goal from global path
   - Generate local trajectory avoiding crowds

### 4.2 Nav2 Plugin Architecture

Both packages will implement Nav2 controller plugins:

```cpp
// C++ wrapper for Python planners
class CrowdNavigationController : public nav2_core::Controller
{
public:
  void configure(/* params */) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  // Python planner binding
  pybind11::object planner_;
};
```

### 4.3 Launch Integration

```python
# wheelchair_bringup/launch/navigation.launch.py
def generate_launch_description():
    return LaunchDescription([
        # Existing nodes
        Node(package='wheelchair_localization', ...),
        Node(package='wheelchair_planning', ...),

        # NEW: Crowd navigation option
        DeclareLaunchArgument(
            'crowd_planner',
            default_value='crowdsurfer',
            choices=['crowdsurfer', 'adaptive', 'default']
        ),

        # Conditional crowd planner
        Node(
            package='wheelchair_crowd_navigation',
            executable='crowd_planner_node',
            condition=LaunchConfigurationEquals('crowd_planner', 'crowdsurfer')
        ),
        Node(
            package='wheelchair_adaptive_navigation',
            executable='adaptive_planner_node',
            condition=LaunchConfigurationEquals('crowd_planner', 'adaptive')
        ),
    ])
```

---

## Part V: Evaluation & Benchmarking

### 5.1 Simulation Environment

**Setup**:
- Gazebo Harmonic (ROS2 Jazzy compatible)
- pedsim_ros2 for pedestrian simulation
- Wheelchair robot model from wheelchair_description
- Scenarios: hallway, cafeteria, outdoor plaza

**Crowd Densities**:
- Sparse: 0.1-0.3 people/m²
- Medium: 0.3-0.6 people/m²
- Dense: 0.6-1.0 people/m²
- Very Dense: >1.0 people/m²

### 5.2 Metrics

**Primary**:
- Success Rate (reached goal without collision)
- Travel Time
- Path Length Ratio
- Minimum Distance to Pedestrians

**Secondary**:
- Jerk (comfort metric)
- Social Violation Rate (personal space <0.5m)
- Computational Time
- Planning Frequency

**Novel Metrics (for adaptive planner)**:
- Uncertainty Calibration (predicted vs actual risk)
- Social Group Preservation (split rate)
- Adaptation Time (response to density changes)

### 5.3 Baselines

1. **DWA** (Dynamic Window Approach) - Nav2 default
2. **TEB** (Timed Elastic Band) - Nav2 plugin
3. **DRL-VO** (if available)
4. **Social Forces**
5. **CrowdSurfer** (Package 1)
6. **Adaptive** (Package 2)

### 5.4 Ablation Studies

**For CrowdSurfer**:
- VQVAE only (no optimizer)
- Optimizer only (no learned prior)
- Different codebook sizes
- Different optimizer iterations

**For Adaptive**:
- Diffusion without uncertainty
- Fixed horizon vs adaptive
- With/without social group modeling
- Hierarchical vs single-level

---

## Part VI: Deployment Strategy

### 6.1 Training Data Collection

**Simulation**:
1. Teleoperation in Gazebo with crowds
2. Record: LaserScans, Odometry, Cmd_vel, Crowd positions
3. Generate 50-100 hours of demonstrations
4. Data augmentation (mirroring, scaling)

**Real World** (future):
1. Safe teleoperation in controlled crowds
2. Record same data streams
3. Domain adaptation training

### 6.2 Computational Optimization

**Training**:
- Multi-GPU training (data parallel)
- Mixed precision (FP16)
- Gradient accumulation
- Model checkpointing

**Inference**:
- TorchScript or ONNX export
- TensorRT optimization (Jetson compatibility)
- Model quantization (INT8)
- Batch trajectory generation

### 6.3 Safety Measures

**Runtime Safety**:
1. Collision checking on all trajectories
2. Fallback to emergency stop if no safe path
3. Velocity limits enforcement
4. Timeout detection
5. Obstacle avoidance reflexes (MPC backup)

**Testing Protocol**:
1. Extensive simulation testing
2. Hardware-in-loop testing
3. Controlled real-world trials
4. Gradual crowd density increase

---

## Part VII: Research Questions & Hypotheses

### 7.1 CrowdSurfer Recreation

**Q1**: Can VQVAE + sampling achieve similar performance in ROS2 as original ROS1?
**H1**: Yes, with proper hyperparameter tuning, we expect ≥90% of original performance.

**Q2**: How does codebook size affect trajectory quality and diversity?
**H2**: Larger codebooks (512-1024) improve diversity but may slow inference.

**Q3**: What is the minimum demonstration data needed for acceptable performance?
**H3**: 20-30 hours of diverse crowd scenarios should suffice.

### 7.2 Novel Adaptive Approach

**Q4**: Do diffusion models generate higher quality trajectories than VQVAE?
**H4**: Yes, continuous latent space enables smoother, more diverse trajectories.

**Q5**: Does uncertainty-aware planning improve safety without sacrificing efficiency?
**H5**: Yes, risk-aware costs reduce close passes by 30% with <10% travel time increase.

**Q6**: How effective is adaptive horizon adjustment?
**H6**: Adaptive horizon improves success rate by 15% in variable-density scenarios.

**Q7**: Does social group modeling reduce discomfort?
**H7**: Yes, group-aware costs reduce social violations by 40%.

---

## Part VIII: Timeline & Milestones

### Phase 1: Foundation (Weeks 1-2)
- ✅ Research roadmap completion
- [ ] Package structure creation
- [ ] Base model implementations (VQVAE, Diffusion)
- [ ] Initial unit tests

**Milestone**: Core models pass unit tests

### Phase 2: Training Infrastructure (Weeks 3-4)
- [ ] Data collection pipeline
- [ ] Training loops for both approaches
- [ ] Visualization and logging
- [ ] Initial model training

**Milestone**: Trained models achieve reasonable reconstruction

### Phase 3: Planning Integration (Weeks 5-6)
- [ ] Sampling optimizers
- [ ] Cost functions
- [ ] ROS2 node architecture
- [ ] Basic ROS2 integration

**Milestone**: Planners running in ROS2, publishing trajectories

### Phase 4: Advanced Features (Weeks 7-8)
- [ ] Social modeling (Package 2)
- [ ] Uncertainty quantification (Package 2)
- [ ] Hierarchical planning (Package 2)
- [ ] Nav2 plugin wrappers

**Milestone**: Full feature set operational

### Phase 5: Evaluation (Weeks 9-10)
- [ ] Simulation environment setup
- [ ] Benchmark scenarios
- [ ] Metrics collection
- [ ] Ablation studies

**Milestone**: Comprehensive evaluation results

### Phase 6: Optimization & Documentation (Weeks 11-12)
- [ ] Performance optimization
- [ ] Code cleanup and documentation
- [ ] User guides and tutorials
- [ ] Research paper drafting

**Milestone**: Production-ready packages

---

## Part IX: Expected Outcomes

### 9.1 Technical Deliverables

1. **wheelchair_crowd_navigation** package
   - Fully functional ROS2 Jazzy package
   - Pretrained VQVAE models
   - Nav2 plugin integration
   - Training and evaluation scripts
   - Documentation

2. **wheelchair_adaptive_navigation** package
   - Diffusion-based planner
   - Social group modeling
   - Uncertainty quantification
   - Hierarchical planning
   - Nav2 plugin integration
   - Documentation

3. **Evaluation Results**
   - Benchmark comparisons
   - Ablation study findings
   - Performance profiles
   - Visualization videos

### 9.2 Research Contributions

1. **First ROS2 Jazzy implementation** of CrowdSurfer approach
2. **Novel diffusion-based crowd navigation** with uncertainty awareness
3. **Social group-aware planning** for wheelchair navigation
4. **Adaptive horizon planning** based on crowd dynamics
5. **Comprehensive benchmark** on wheelchair platform

### 9.3 Publications

**Target Venues**:
- ICRA 2026 (International Conference on Robotics and Automation)
- IROS 2026 (Intelligent Robots and Systems)
- RA-L (Robotics and Automation Letters)
- JFR (Journal of Field Robotics)

**Paper Titles**:
1. "Dense Crowd Navigation for Wheelchairs: A Diffusion-Based Approach with Social Awareness"
2. "Uncertainty-Aware Hierarchical Planning for Assistive Robots in Human Crowds"

---

## Part X: Risk Mitigation

### 10.1 Technical Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Insufficient training data | Medium | High | Use simulation + data augmentation |
| Poor sim-to-real transfer | High | High | Domain randomization, real data fine-tuning |
| Slow inference time | Medium | High | Model optimization, TensorRT, pruning |
| Overfitting to scenarios | Medium | Medium | Diverse training scenarios, regularization |
| ROS2 integration issues | Low | Medium | Thorough testing, community support |

### 10.2 Research Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Diffusion not better than VQVAE | Medium | Medium | Hybrid approach, ensemble methods |
| Uncertainty estimation unreliable | Medium | High | Multiple estimation methods, validation |
| Social modeling ineffective | Low | Low | Simplified fallback, ablation studies |
| Computational overhead too high | Medium | High | Optimization, hardware acceleration |

### 10.3 Contingency Plans

**If diffusion underperforms**:
- Fall back to enhanced VQVAE
- Explore alternative generative models (Flow matching, GAN)

**If training data insufficient**:
- Leverage transfer learning from other domains
- Use few-shot learning techniques

**If real-time performance not achieved**:
- Reduce model size
- Use distillation
- Implement asynchronous planning

---

## Part XI: Resources & Requirements

### 11.1 Computing Resources

**Training**:
- GPU: NVIDIA A100/V100 or RTX 4090
- RAM: 64GB+
- Storage: 500GB+ for datasets
- Time: ~1 week per model

**Inference**:
- GPU: NVIDIA Jetson Orin or RTX 3060+
- RAM: 16GB+
- Real-time requirement: 10Hz planning

### 11.2 Software Dependencies

**Core**:
- ROS2 Jazzy
- Python 3.10+
- PyTorch 2.0+
- JAX 0.4+
- CUDA 12.x

**ROS2 Packages**:
- Nav2
- Gazebo (sim_gz)
- pedsim_ros2
- robot_localization

### 11.3 Team & Expertise

**Required Skills**:
- Deep learning (VAE, Diffusion models)
- Robotics (ROS2, motion planning)
- Reinforcement learning (optional, for baselines)
- C++/Python programming
- Simulation (Gazebo)

---

## Part XII: Next Steps

### Immediate Actions (Week 1)

1. ✅ **Roadmap approval** - Review and finalize this document
2. [ ] **Environment setup** - Install dependencies, configure workspace
3. [ ] **Package creation** - Initialize both ROS2 packages
4. [ ] **Model prototyping** - Implement basic VQVAE and Diffusion models
5. [ ] **Data pipeline** - Design demonstration data format

### Short-term (Weeks 2-4)

6. [ ] **Training infrastructure** - Build training loops and logging
7. [ ] **Simulation setup** - Configure Gazebo + pedsim
8. [ ] **Data collection** - Generate 10+ hours of demonstrations
9. [ ] **Initial training** - Train first version of models
10. [ ] **ROS2 skeleton** - Create basic ROS2 node structure

### Medium-term (Weeks 5-8)

11. [ ] **Planning algorithms** - Implement samplers and optimizers
12. [ ] **Social modeling** - Build group detection and costs
13. [ ] **Integration** - Connect planners to Nav2
14. [ ] **Testing** - Unit tests and integration tests
15. [ ] **Benchmarking** - Initial performance evaluation

---

## Part XIII: References & Resources

### Key Papers

1. **CrowdSurfer** - "Sampling Optimization Augmented with VQVAE" (ICRA 2025)
2. **Diffusion Models** - "Denoising Diffusion Probabilistic Models" (NeurIPS 2020)
3. **Social Forces** - Helbing & Molnar (Physical Review E, 1995)
4. **ORCA** - van den Berg et al. (ICRA 2008)
5. **DRL-VO** - Everett et al. (IROS 2018)
6. **Social Groups** - Yamaoka et al. (HRI 2019)

### Code Repositories

- CrowdSurfer: https://github.com/Smart-Wheelchair-RRC/CrowdSurfer
- Nav2: https://github.com/ros-planning/navigation2
- pedsim_ros: https://github.com/srl-freiburg/pedsim_ros
- Diffusers: https://github.com/huggingface/diffusers

### Datasets

- ETH/UCY pedestrian dataset
- ATC shopping center dataset
- Stanford drone dataset (for crowd flow)

---

## Appendix A: Configuration Examples

### VQVAE Config
```yaml
model:
  encoder:
    channels: [64, 128, 256]
    kernel_size: 3
    activation: 'relu'

  codebook:
    num_embeddings: 512
    embedding_dim: 64
    commitment_cost: 0.25

  decoder:
    channels: [256, 128, 64]
    output_dim: 2  # (x, y) trajectory points
    horizon: 30    # 3 seconds at 10Hz

training:
  batch_size: 128
  learning_rate: 0.0003
  num_epochs: 200
  warmup_steps: 1000
```

### Diffusion Config
```yaml
model:
  unet:
    in_channels: 2
    out_channels: 2
    down_block_types: ["DownBlock1D", "DownBlock1D", "AttnDownBlock1D"]
    up_block_types: ["AttnUpBlock1D", "UpBlock1D", "UpBlock1D"]
    block_out_channels: [128, 256, 512]

  noise_scheduler:
    num_train_timesteps: 1000
    beta_schedule: 'squaredcos_cap_v2'
    prediction_type: 'epsilon'

training:
  batch_size: 64
  learning_rate: 0.0001
  num_epochs: 300
  mixed_precision: 'fp16'
```

---

## Document History

- **2025-11-22**: Initial roadmap creation
- **TBD**: Revision 1 after Phase 1 completion
- **TBD**: Revision 2 after initial results

---

**Roadmap Status**: ✅ COMPLETE - Ready for implementation

**Next Document**: See implementation READMEs in respective packages

# wheelchair_adaptive_navigation

**Novel Diffusion-Based Adaptive Navigation for Dense Crowds**

This package implements a novel approach to wheelchair crowd navigation using diffusion models, uncertainty quantification, and social awareness.

## Overview

This package introduces several innovations beyond the CrowdSurfer approach:

### Key Innovations

1. **Diffusion Models** - Replace VQVAE with Denoising Diffusion Probabilistic Models (DDPM)
   - Continuous latent space for smoother trajectories
   - Higher quality and diversity via iterative refinement
   - DDIM sampling for fast inference

2. **Uncertainty-Aware Planning** - Quantify and leverage trajectory uncertainty
   - Ensemble-based uncertainty estimation
   - Risk-aware trajectory selection
   - Safe navigation in uncertain environments

3. **Social Group Modeling** - Explicit modeling of social dynamics
   - F-formation detection using DBSCAN clustering
   - Group-aware cost functions
   - Crowd flow prediction

4. **Hierarchical Planning** - Multi-level planning architecture
   - Global: Topological path planning
   - Local: Diffusion-based trajectory generation
   - Reflex: MPC for emergency avoidance

5. **Adaptive Horizon** - Dynamic planning horizon
   - Adjusts based on crowd density
   - Longer planning in sparse areas
   - Shorter, more reactive in dense crowds

## Architecture

```
Perception → Social Group Detection
    ↓
Conditional Encoder → Context Features
    ↓
Diffusion Model → Trajectory Ensemble
    ↓
Uncertainty Quantification → Risk Estimates
    ↓
Risk-Aware Selection → Best Trajectory
    ↓
Hierarchical Planner → Final Path
```

## Installation

### Prerequisites

- ROS2 Jazzy
- Python 3.10+
- CUDA 12.x (recommended)

### Python Dependencies

```bash
pip install torch>=2.0.0 diffusers>=0.25.0 accelerate>=0.26.0
pip install scikit-learn filterpy tensorboard
```

### Build

```bash
cd ~/wc
colcon build --packages-select wheelchair_adaptive_navigation
source install/setup.bash
```

## Usage

### Running the Planner

```bash
ros2 launch wheelchair_adaptive_navigation adaptive_navigation.launch.py \
    model_path:=/path/to/diffusion_model.pt \
    device:=cuda
```

### Training

```bash
# Train diffusion model
ros2 run wheelchair_adaptive_navigation train_diffusion \
    --config config/diffusion_config.yaml \
    --data_dir ./data/demonstrations
```

### Evaluation

```bash
ros2 run wheelchair_adaptive_navigation evaluate \
    --model_path ./models/diffusion_best.pt \
    --scenarios dense,sparse,dynamic
```

## Configuration

See `config/diffusion_config.yaml`:
- Diffusion model architecture
- Training hyperparameters
- Social modeling parameters
- Uncertainty thresholds

## Key Features

### 1. Diffusion-Based Generation

Unlike VQVAE's discrete latent space, diffusion models operate in continuous space:

```python
# Generate trajectory ensemble
trajectories = diffusion_model.sample(
    batch_size=10,  # Ensemble size
    condition=context_features,
    num_steps=50,  # DDIM steps
)
```

### 2. Uncertainty Quantification

Ensemble provides natural uncertainty estimates:

```python
mean_trajectory, variance = ensemble.compute_uncertainty(trajectories)
risk = risk_estimator.estimate_risk(mean_trajectory, variance, obstacles)
```

### 3. Social Group Detection

DBSCAN-based clustering finds social groups:

```python
groups = social_detector.detect_groups(
    positions=pedestrian_positions,
    velocities=pedestrian_velocities,
)
```

### 4. Adaptive Horizon

Planning horizon adapts to conditions:

```
H = H_base * density_factor * uncertainty_factor
  = 3.0s * (1.0 - ρ/ρ_max) * (1.0 + σ/σ_threshold)
  ∈ [1.5s, 4.5s]
```

## Research Contributions

This package contributes:

1. **First diffusion-based approach** for wheelchair crowd navigation
2. **Uncertainty-aware planning** with risk quantification
3. **Social group modeling** for assistive robotics
4. **Hierarchical architecture** combining global and local planning
5. **Adaptive horizon** based on environmental conditions

## Performance Expectations

**Hypothesized Improvements over CrowdSurfer**:
- **+15% success rate** in variable-density scenarios
- **-10% travel time** due to smoother trajectories
- **-40% social violations** via group-aware planning
- **+30% user comfort** from uncertainty-aware decisions

**Computational Cost**:
- Training: ~2 weeks on A100
- Inference: ~100ms per plan (10Hz capable)
- Memory: ~500MB

## Topics

### Subscriptions
- `/scan` - LaserScan data
- `/odom` - Robot odometry
- `/goal_pose` - Navigation goal

### Publications
- `/local_plan` - Planned trajectory
- `/uncertainty_viz` - Uncertainty visualization
- `/social_groups_viz` - Detected social groups
- `/cmd_vel` - Velocity commands

## Directory Structure

```
wheelchair_adaptive_navigation/
├── wheelchair_adaptive_navigation/
│   ├── models/
│   │   ├── diffusion/      # DDPM, U-Net
│   │   ├── social/         # Group detection, flow
│   │   └── uncertainty/    # Ensemble, risk estimation
│   ├── planning/
│   │   └── hierarchical/   # Global, local, reflex
│   ├── perception/         # Crowd analysis
│   ├── ros2/               # ROS2 nodes
│   └── training/           # Training scripts
├── config/                 # Configuration files
├── launch/                 # Launch files
└── README.md
```

## Citation

If you use this package, please cite our work (to be published):

```bibtex
@article{tiwari2025adaptive,
  title={Adaptive Crowd Navigation for Wheelchairs: A Diffusion-Based Approach with Social Awareness},
  author={Tiwari, Siddharth},
  journal={In preparation},
  year={2025}
}
```

And the original CrowdSurfer paper:

```bibtex
@inproceedings{crowdsurfer2025,
  title={CrowdSurfer: Sampling Optimization Augmented with Vector-Quantized Variational AutoEncoder for Dense Crowd Navigation},
  booktitle={IEEE ICRA},
  year={2025}
}
```

## License

MIT License

## Authors

- Siddharth Tiwari (s24035@students.iitmandi.ac.in)

## References

- [CrowdSurfer](https://github.com/Smart-Wheelchair-RRC/CrowdSurfer)
- [DDPM Paper](https://arxiv.org/abs/2006.11239)
- [DDIM Paper](https://arxiv.org/abs/2010.02502)
- [Social Forces](https://arxiv.org/abs/cond-mat/9805244)

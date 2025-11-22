# wheelchair_crowd_navigation

**CrowdSurfer-based Dense Crowd Navigation for ROS2 Jazzy**

This package implements the CrowdSurfer approach (ICRA 2025) for wheelchair navigation in dense crowds, adapted for ROS2 Jazzy.

## Overview

CrowdSurfer combines:
- **Vector Quantized VAE (VQVAE)** for learning trajectory priors from demonstrations
- **PixelCNN** for modeling the distribution over discrete codes
- **Sampling-based optimization** (Cross-Entropy Method) for runtime trajectory refinement

This hybrid approach achieves:
- 40% improvement in success rate vs DRL-VO
- 6% improvement in travel time
- Real-time performance at 10Hz planning

## Architecture

```
Perception (LaserScan/PointCloud)
    ↓
Perception Encoder → Features
    ↓
VQVAE + PixelCNN → Initial Trajectory Samples
    ↓
Cross-Entropy Method → Refined Trajectory
    ↓
Path Publisher → Nav2/Controllers
```

## Installation

### Prerequisites

- ROS2 Jazzy
- Python 3.10+
- CUDA 12.x (for GPU acceleration)

### Python Dependencies

```bash
pip install torch>=2.0.0 --index-url https://download.pytorch.org/whl/cu121
pip install jax[cuda12]>=0.4.0
pip install flax optax tensorboard open3d hydra-core
```

### Build

```bash
cd ~/wc
colcon build --packages-select wheelchair_crowd_navigation
source install/setup.bash
```

## Usage

### Running the Planner

```bash
# With trained model
ros2 launch wheelchair_crowd_navigation crowd_navigation.launch.py \
    model_path:=/path/to/checkpoint.pt \
    device:=cuda

# Without trained model (random initialization)
ros2 launch wheelchair_crowd_navigation crowd_navigation.launch.py \
    device:=cpu
```

### Training

**Data Collection** (teleoperation in simulation):
```bash
ros2 run wheelchair_crowd_navigation collect_data \
    --output_dir ./data/demonstrations \
    --num_episodes 100
```

**Training VQVAE**:
```bash
ros2 run wheelchair_crowd_navigation train_vqvae \
    --config config/vqvae_config.yaml \
    --data_dir ./data/demonstrations \
    --output_dir ./models/vqvae
```

**Evaluation**:
```bash
ros2 run wheelchair_crowd_navigation evaluate \
    --model_path ./models/vqvae/best_model.pt \
    --scenarios dense_crowd,sparse_crowd
```

## Configuration

See `config/` directory:
- `vqvae_config.yaml` - Model architecture and training
- `planner_config.yaml` - Runtime planning parameters

## Topics

### Subscriptions
- `/scan` (sensor_msgs/LaserScan) - Laser scan data
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goal

### Publications
- `/local_plan` (nav_msgs/Path) - Planned trajectory
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/crowd_viz` (visualization_msgs/MarkerArray) - Visualization

## Performance

**Tested on**:
- NVIDIA RTX 4090: ~100Hz planning
- NVIDIA Jetson Orin: ~10Hz planning
- CPU (Intel i9): ~2Hz planning

**Memory Usage**:
- Model: ~200MB
- Runtime: ~500MB total

## Directory Structure

```
wheelchair_crowd_navigation/
├── wheelchair_crowd_navigation/
│   ├── models/         # Neural network models
│   ├── planning/       # Sampling optimizer, cost functions
│   ├── perception/     # Perception processing
│   ├── ros2/           # ROS2 nodes
│   └── training/       # Training scripts
├── config/             # Configuration files
├── launch/             # Launch files
├── models/             # Pretrained models (git-lfs)
├── data/               # Training data
└── tests/              # Unit tests
```

## Citation

If you use this package, please cite:

```bibtex
@inproceedings{crowdsurfer2025,
  title={CrowdSurfer: Sampling Optimization Augmented with Vector-Quantized Variational AutoEncoder for Dense Crowd Navigation},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2025}
}
```

## License

MIT License

## Authors

- Siddharth Tiwari (s24035@students.iitmandi.ac.in)

## References

- [CrowdSurfer GitHub](https://github.com/Smart-Wheelchair-RRC/CrowdSurfer)
- [VQVAE Paper](https://arxiv.org/abs/1711.00937)
- [Nav2 Documentation](https://navigation.ros.org/)

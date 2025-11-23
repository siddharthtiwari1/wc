# Dense Crowd Navigation System - Complete Implementation Guide

**Author**: Siddharth Tiwari
**Contact**: s24035@students.iitmandi.ac.in
**System**: RTX 5050 (8GB) | CUDA 13.0 | ROS2 Jazzy
**Date**: November 2025

---

## 🎯 Project Overview

This workspace contains TWO production-ready packages for wheelchair dense crowd navigation:

### Package 1: `crowdsurfer_nav`
**CrowdSurfer Recreation** - ICRA 2025 paper implementation
- VQVAE + PixelCNN trajectory generation
- PRIEST sampling optimizer
- Full training pipeline
- Nav2 integration

### Package 2: `diffusion_crowd_nav`
**Novel Diffusion Approach** - Research innovation
- Denoising Diffusion Probabilistic Models (DDPM)
- Social-aware planning with group detection
- Uncertainty quantification
- Adaptive horizon planning

---

## 🚀 Quick Start (Complete Setup in 3 Commands)

```bash
# 1. Run automated setup (installs everything)
./scripts/setup_all.sh

# 2. Download pre-trained models (optional)
./scripts/download_models.sh

# 3. Launch simulation + navigation
ros2 launch crowdsurfer_nav full_simulation.launch.py
```

---

## 📋 Table of Contents

1. [System Requirements](#system-requirements)
2. [Installation](#installation)
3. [Data Collection](#data-collection)
4. [Training](#training)
5. [Deployment](#deployment)
6. [Simulation](#simulation)
7. [Evaluation](#evaluation)
8. [Troubleshooting](#troubleshooting)

---

## 💻 System Requirements

### Hardware
- **GPU**: NVIDIA with ≥8GB VRAM (RTX 5050 ✅)
- **RAM**: 16GB+ recommended
- **Storage**: 50GB free space

### Software
- **OS**: Ubuntu 22.04/24.04
- **ROS2**: Jazzy Jalisco
- **CUDA**: 12.x or 13.x ✅
- **Python**: 3.10+

---

## 🔧 Installation

### Option A: Automated (Recommended)

```bash
cd ~/wc
chmod +x scripts/setup_all.sh
./scripts/setup_all.sh
```

This script will:
1. Check dependencies
2. Install Python packages
3. Build ROS2 workspace
4. Setup simulation environment
5. Download datasets (optional)

### Option B: Manual Installation

#### Step 1: Python Dependencies

```bash
# Create virtual environment
python3 -m venv ~/crowdnav_venv
source ~/crowdnav_venv/bin/activate

# Install PyTorch with CUDA 13.0
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130

# Install JAX with CUDA
pip install "jax[cuda13_pip]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html

# Install other dependencies
pip install -r requirements.txt
```

#### Step 2: ROS2 Packages

```bash
cd ~/wc
source /opt/ros/jazzy/setup.bash

# Install simulation dependencies
sudo apt install -y \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-nav2-bringup \
    ros-jazzy-navigation2 \
    ros-jazzy-turtlebot3* \
    python3-colcon-common-extensions

# Clone HuNavSim for pedestrian simulation
cd src
git clone https://github.com/robotics-upo/hunav_sim.git -b jazzy-devel || \
git clone https://github.com/robotics-upo/hunav_sim.git  # Use main if jazzy branch doesn't exist

# Build workspace
cd ~/wc
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 📊 Data Collection

### Method 1: Teleoperation in Simulation

```bash
# Terminal 1: Launch simulation with pedestrians
ros2 launch crowdsurfer_nav data_collection.launch.py \
    world:=cafeteria \
    num_pedestrians:=20

# Terminal 2: Teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Start recording
ros2 run crowdsurfer_nav collect_demonstrations.py \
    --output_dir ~/datasets/crowdnav_demos \
    --duration 3600  # 1 hour
```

**What gets recorded:**
- LaserScan (`/scan`)
- Odometry (`/odom`)
- Control commands (`/cmd_vel`)
- Pedestrian states (`/pedsim/agents`)
- Timestamp aligned

### Method 2: Use Pre-collected Dataset

```bash
# Download pre-collected demonstrations
./scripts/download_dataset.sh

# Verify data
python3 scripts/verify_dataset.py --data_dir ~/datasets/crowdnav_demos
```

### Data Format

```
~/datasets/crowdnav_demos/
├── episode_0000/
│   ├── metadata.json          # Episode info
│   ├── laser_scans.npz        # [T, 360] LaserScan ranges
│   ├── odometry.npz           # [T, 3] (x, y, theta)
│   ├── commands.npz           # [T, 2] (linear_vel, angular_vel)
│   ├── pedestrians.npz        # [T, N, 4] (x, y, vx, vy)
│   └── timestamps.npz         # [T]
├── episode_0001/
...
└── statistics.json            # Dataset statistics
```

---

## 🏋️ Training

### Package 1: CrowdSurfer (VQVAE + PixelCNN + PRIEST)

#### Step 1: Preprocess Data

```bash
python3 src/crowdsurfer_nav/scripts/preprocess_data.py \
    --input_dir ~/datasets/crowdnav_demos \
    --output_dir ~/datasets/crowdnav_processed \
    --horizon 30 \
    --dt 0.1 \
    --train_split 0.8 \
    --val_split 0.1
```

#### Step 2: Train VQVAE

```bash
# Single GPU training
python3 src/crowdsurfer_nav/scripts/train_vqvae.py \
    --config src/crowdsurfer_nav/config/vqvae_train.yaml \
    --data_dir ~/datasets/crowdnav_processed \
    --output_dir ~/models/vqvae_checkpoints \
    --gpu 0

# Monitor training
tensorboard --logdir ~/models/vqvae_checkpoints/logs
```

**Expected training time**: 6-8 hours on RTX 5050

#### Step 3: Train PixelCNN Prior

```bash
python3 src/crowdsurfer_nav/scripts/train_pixelcnn.py \
    --config src/crowdsurfer_nav/config/pixelcnn_train.yaml \
    --vqvae_checkpoint ~/models/vqvae_checkpoints/best_model.pth \
    --data_dir ~/datasets/crowdnav_processed \
    --output_dir ~/models/pixelcnn_checkpoints \
    --gpu 0
```

**Expected training time**: 4-6 hours on RTX 5050

#### Step 4: Validate Models

```bash
python3 src/crowdsurfer_nav/scripts/validate_models.py \
    --vqvae_checkpoint ~/models/vqvae_checkpoints/best_model.pth \
    --pixelcnn_checkpoint ~/models/pixelcnn_checkpoints/best_model.pth \
    --test_data ~/datasets/crowdnav_processed/test \
    --output_dir ~/validation_results
```

### Package 2: Diffusion Model

#### Train Diffusion Trajectory Generator

```bash
python3 src/diffusion_crowd_nav/scripts/train_diffusion.py \
    --config src/diffusion_crowd_nav/config/diffusion_train.yaml \
    --data_dir ~/datasets/crowdnav_processed \
    --output_dir ~/models/diffusion_checkpoints \
    --gpu 0 \
    --num_diffusion_steps 1000 \
    --batch_size 64
```

**Expected training time**: 8-12 hours on RTX 5050

---

## 🚢 Deployment

### Deploy on Laptop (Inference Only)

#### Step 1: Export Models for Inference

```bash
# Export optimized models
python3 scripts/export_for_deployment.py \
    --vqvae ~/models/vqvae_checkpoints/best_model.pth \
    --pixelcnn ~/models/pixelcnn_checkpoints/best_model.pth \
    --output ~/models/deployed \
    --optimize --quantize
```

#### Step 2: Copy to Laptop

```bash
# On training PC
rsync -avz ~/models/deployed/ laptop:~/wc_models/

# On laptop
cd ~/wc
source install/setup.bash
export WC_MODEL_PATH=~/wc_models
```

#### Step 3: Launch Navigation

```bash
# On laptop (real wheelchair)
ros2 launch crowdsurfer_nav wheelchair_navigation.launch.py \
    model_path:=$WC_MODEL_PATH/vqvae_optimized.pth \
    device:=cuda  # or cpu if no GPU on laptop
```

---

## 🎮 Simulation

### Launch Full Simulation Environment

```bash
# Complete stack: Gazebo + Pedestrians + Navigation
ros2 launch crowdsurfer_nav full_simulation.launch.py \
    world:=cafeteria \
    num_pedestrians:=30 \
    density:=high \
    planner:=crowdsurfer  # or 'diffusion'
```

### Available Scenarios

```bash
# Sparse crowd
ros2 launch crowdsurfer_nav full_simulation.launch.py scenario:=sparse

# Dense crowd (challenging)
ros2 launch crowdsurfer_nav full_simulation.launch.py scenario:=dense

# Dynamic (moving groups)
ros2 launch crowdsurfer_nav full_simulation.launch.py scenario:=dynamic

# Custom
ros2 launch crowdsurfer_nav full_simulation.launch.py \
    num_pedestrians:=50 \
    pedestrian_speed:=1.2 \
    robot_max_speed:=0.5
```

---

## 📈 Evaluation

### Benchmark Against Baselines

```bash
python3 scripts/benchmark.py \
    --scenarios sparse,dense,dynamic \
    --planners crowdsurfer,diffusion,dwa,teb \
    --num_episodes 100 \
    --output ~/benchmark_results

# Generate report
python3 scripts/generate_report.py \
    --results ~/benchmark_results \
    --output ~/paper_figures
```

### Metrics Computed

- **Success Rate**: % of episodes reaching goal without collision
- **Travel Time**: Average time to reach goal
- **Path Length Ratio**: Actual path / optimal path
- **Minimum Distance to Pedestrians**: Safety metric
- **Jerk**: Comfort metric
- **Social Violations**: Personal space (<0.5m) intrusions
- **Computational Time**: Planning frequency

---

## 🐛 Troubleshooting

### CUDA Out of Memory

```bash
# Reduce batch size in config
# For VQVAE training
sed -i 's/batch_size: 128/batch_size: 64/' src/crowdsurfer_nav/config/vqvae_train.yaml

# For Diffusion training
sed -i 's/batch_size: 64/batch_size: 32/' src/diffusion_crowd_nav/config/diffusion_train.yaml
```

### Simulation Crashes

```bash
# Increase Gazebo memory
export GAZEBO_MASTER_URI=http://localhost:11345
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org

# Reduce physics update rate
ros2 param set /gazebo update_rate 100  # Default 1000
```

### Nav2 Plugin Not Loading

```bash
# Verify plugin registration
ros2 pkg executables crowdsurfer_nav

# Check plugin path
ros2 param get /controller_server controller_plugins

# Rebuild with verbose
colcon build --packages-select crowdsurfer_nav --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

---

## 📚 Additional Resources

### Documentation
- **CrowdSurfer Paper**: https://arxiv.org/abs/2409.16011
- **PRIEST Paper**: https://arxiv.org/abs/2309.08235
- **SICNav-Diffusion**: https://arxiv.org/abs/2503.08858

### Code References
- **Original CrowdSurfer**: https://github.com/Smart-Wheelchair-RRC/CrowdSurfer
- **HuNavSim**: https://github.com/robotics-upo/hunav_sim
- **Nav2 Docs**: https://docs.nav2.org/

---

## 📞 Support

**Issues**: Create GitHub issue in this repository
**Email**: s24035@students.iitmandi.ac.in
**Research Group**: Smart Wheelchair Research

---

**Last Updated**: 2025-11-22
**Version**: 1.0.0-production

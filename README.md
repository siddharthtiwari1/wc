# Real-World Attribute-Aware Navigation (RAN)

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)

**Uncertainty-Aware Description-First Navigation for Mobile Robots**

[Paper](paper/main.pdf) | [Demo Video](#) | [Documentation](docs/)

---

## Overview

RAN (Real-world Attribute-aware Navigation) enables mobile robots to navigate to objects described with fine-grained natural language:

> *"Go to the **red mug** on the **wooden table** near the **window**"*

Unlike category-only systems (e.g., "go to mug"), RAN explicitly verifies attributes (color, shape, material) and quantifies uncertainty, enabling robust real-world deployment.

### Key Features

- ✅ **Real-robot validated** on RPLidar S3 + RealSense D455
- ✅ **Attribute-aware** verification (color, shape, texture, spatial relations)
- ✅ **Uncertainty estimation** per attribute with confidence scores
- ✅ **Dynamic scenes** with online map updates
- ✅ **Multi-environment** generalization (homes, offices, labs)
- ✅ **Long-horizon** navigation with hierarchical verification

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     NAVIGATION LAYER                        │
│  Goal Retrieval → Hierarchical Verification → Path Planning │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                  LANGUAGE GROUNDING LAYER                   │
│    LLM Parser → Attribute Extraction → Multi-modal Retrieval│
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                  3D SEMANTIC MAPPING LAYER                  │
│   Detection → Segmentation → Feature Extraction → Clustering│
│   (YOLO-World)  (SAM2)   (DINOv2+CLIP)   (Multi-view Fusion)│
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│              PERCEPTION & LOCALIZATION LAYER                │
│        RPLidar S3 (SLAM) + RealSense D455 (RGB-D)           │
└─────────────────────────────────────────────────────────────┘
```

---

## Hardware Requirements

### Minimum Setup:
- Mobile robot platform (differential drive or omnidirectional)
- **RPLidar S3** (360° 2D LiDAR, 40m range)
- **Intel RealSense D455** (RGB-D camera, 1280×720@30fps)
- Onboard computer: Jetson Xavier NX or laptop with GPU (RTX 3060+)

### Our Test Platform:
- TurtleBot 4 / Custom differential drive
- RPLidar S3 mounted at 15cm height
- RealSense D455 front-facing, 30° downward tilt
- Jetson AGX Orin 32GB (64GB recommended for full pipeline)

---

## Installation

### 1. System Dependencies

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-desktop ros-humble-navigation2 \
    ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-robot-localization

# RealSense SDK
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-*

# RPLidar
sudo apt install ros-humble-rplidar-ros

# Python dependencies
pip install torch torchvision transformers ultralytics \
    segment-anything-2 open-clip-torch sentence-transformers \
    faiss-cpu numpy opencv-python scipy
```

### 2. Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/real_attribute_navigation.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Download Model Weights

```bash
cd ~/ros2_ws/src/real_attribute_navigation
bash scripts/download_models.sh  # Downloads YOLO-World, SAM2, DINOv2, CLIP
```

---

## Quick Start

### 1. Launch Robot Hardware

```bash
# Terminal 1: Launch sensors
ros2 launch ran_bringup robot.launch.py

# Terminal 2: Launch SLAM (Cartographer with RPLidar)
ros2 launch ran_bringup slam.launch.py

# Terminal 3: Launch Nav2
ros2 launch ran_bringup navigation.launch.py
```

### 2. Build Semantic Map

```bash
# Drive robot around environment (use teleop or autonomous exploration)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Build 3D semantic map
ros2 launch ran_mapping build_map.launch.py

# Map saved to: data/maps/<timestamp>_semantic_map.json
```

### 3. Run Attribute-Aware Navigation

```bash
# Launch full navigation system
ros2 launch ran_navigation attribute_nav.launch.py map_file:=data/maps/my_map.json

# Send natural language command
ros2 run ran_navigation send_goal.py \
    --instruction "Go to the red mug on the wooden table near the window"
```

---

## Project Structure

```
real_attribute_navigation/
├── src/
│   ├── ran_perception/        # Object detection, segmentation, feature extraction
│   ├── ran_mapping/           # 3D semantic map building
│   ├── ran_navigation/        # Language grounding, goal retrieval, path planning
│   └── ran_bringup/           # System launch files
├── evaluation/
│   ├── baselines/             # VLMaps, O3D-SIM, LM-Nav implementations
│   ├── metrics/               # Success rate, attribute precision, sim2real gap
│   └── scripts/               # Experiment runners, plotting
├── paper/                     # LaTeX source for publication
├── docs/                      # Detailed documentation
└── data/                      # Maps, rosbags, instructions
```

---

## Citation

If you use this code in your research, please cite:

```bibtex
@inproceedings{ran2026,
  title={Uncertainty-Aware Attribute Navigation: Real-Robot Description-First Mapping},
  author={Your Name},
  booktitle={Robotics: Science and Systems (RSS)},
  year={2026}
}
```

---

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.

---

**Status**: 🚧 Under active development for RSS 2026 submission

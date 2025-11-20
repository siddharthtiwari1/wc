# Wheelchair AMCL Localization

Nav2 AMCL-based localization for the wheelchair robot using pre-built maps.

## Overview

This package provides AMCL (Adaptive Monte Carlo Localization) for localizing the wheelchair robot against a pre-built map created with SLAM Toolbox.

## Workflow

### 1. Create a Map (SLAM Mapping)

First, drive around and create a map:

```bash
ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py
```

Drive the wheelchair around to build the map.

### 2. Save the Map

Once mapping is complete, save the map:

```bash
cd /home/sidd/wc_ws/src/wheelchair_localization/scripts
./save_map.sh <map_name>
```

Example:
```bash
./save_map.sh office_floor1
```

This creates:
- `wheelchair_localization/maps/office_floor1/map.yaml`
- `wheelchair_localization/maps/office_floor1/map.pgm`

### 3. Use AMCL Localization

Launch the complete system with localization:

```bash
ros2 launch wheelchair_bringup wheelchair_full_system.launch.py
ros2 launch wheelchair_localization wheelchair_localization.launch.py map_name:=office_floor1
```

### 4. Set Initial Pose in RViz

1. In RViz, click "2D Pose Estimate"
2. Click and drag on the map to set initial robot position and orientation
3. AMCL will start localizing the robot

## Configuration

### AMCL Parameters (`config/amcl.yaml`)

Key parameters optimized for wheelchair:

- **Motion Model**: Differential drive with accurate encoders
  - `alpha1: 0.08` - Low rotation noise (good encoders)
  - `alpha3: 0.15` - Translation noise

- **Laser Model**: RPLidar S3 (40m range)
  - `laser_max_range: 40.0`
  - `max_beams: 120` - High resolution lidar

- **Particle Filter**:
  - `max_particles: 3000` - Large spaces
  - `min_particles: 800`

- **Update Thresholds**:
  - `update_min_d: 0.15m` - Update every 15cm
  - `update_min_a: 0.15 rad` - Update every 8.6 degrees

## Map Directory Structure

```
wheelchair_localization/
└── maps/
    ├── office_floor1/
    │   ├── map.yaml
    │   └── map.pgm
    ├── warehouse/
    │   ├── map.yaml
    │   └── map.pgm
    └── ...
```

## Troubleshooting

### Poor Localization

1. **Increase particles**: Edit `amcl.yaml`, increase `max_particles`
2. **Check initial pose**: Make sure initial pose estimate is accurate
3. **Verify map quality**: Ensure SLAM map is clean and detailed

### Localization Lost

AMCL has recovery behaviors configured:
- `recovery_alpha_fast: 0.1`
- `recovery_alpha_slow: 0.001`

If lost, use "2D Pose Estimate" in RViz to re-localize.

## Frame Setup

AMCL publishes the `map` → `odom` transform:
- `map` → `odom` (from AMCL)
- `odom` → `base_link` (from EKF)
- `base_link` → robot links (from URDF)

## Services

### Save Map
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/path/to/map'}}"
```

### Global Localization (Kidnapped Robot)
```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

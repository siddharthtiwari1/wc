#!/bin/bash

# Save map from SLAM Toolbox
# Usage: ./save_map.sh <map_name>

if [ -z "$1" ]; then
    echo "Usage: ./save_map.sh <map_name>"
    echo "Example: ./save_map.sh office_floor1"
    exit 1
fi

MAP_NAME=$1
MAPS_DIR="$(ros2 pkg prefix wheelchair_localization)/share/wheelchair_localization/maps"

# Create map directory
mkdir -p "$MAPS_DIR/$MAP_NAME"

echo "Saving map to: $MAPS_DIR/$MAP_NAME/map"

# Call SLAM Toolbox save map service
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$MAPS_DIR/$MAP_NAME/map'}}"

echo "Map saved successfully!"
echo "To use this map for localization, run:"
echo "ros2 launch wheelchair_localization wheelchair_localization.launch.py map_name:=$MAP_NAME"

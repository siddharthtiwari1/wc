#!/bin/bash

# SLAM Configuration Quick Comparison Tool
# Usage: ./compare_configs.sh [version1] [version2]

CONFIG_DIR="/home/sidd/wc/src/wheelchair_localization/config"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default comparison: v2 vs v14
V1="${1:-v2}"
V2="${2:-v14}"

echo "================================================================================"
echo "SLAM Configuration Comparison: ${V1} vs ${V2}"
echo "================================================================================"
echo ""

# Extract key parameters
extract_param() {
    local file=$1
    local param=$2
    grep -A1 "^[[:space:]]*${param}:" "$file" | tail -1 | awk '{print $2}' | tr -d '#'
}

FILE1="${CONFIG_DIR}/slam_toolbox_${V1}.yaml"
FILE2="${CONFIG_DIR}/slam_toolbox_${V2}.yaml"

if [ ! -f "$FILE1" ]; then
    echo -e "${RED}Error: $FILE1 not found${NC}"
    exit 1
fi

if [ ! -f "$FILE2" ]; then
    echo -e "${RED}Error: $FILE2 not found${NC}"
    exit 1
fi

# Function to compare and highlight differences
compare_param() {
    local param_name=$1
    local yaml_key=$2
    local val1=$(extract_param "$FILE1" "$yaml_key")
    local val2=$(extract_param "$FILE2" "$yaml_key")
    
    printf "%-40s | %-15s | %-15s | " "$param_name" "$val1" "$val2"
    
    if [ "$val1" = "$val2" ]; then
        echo -e "${GREEN}SAME${NC}"
    else
        echo -e "${YELLOW}DIFFERENT${NC}"
    fi
}

echo "CRITICAL PARAMETERS:"
echo "--------------------------------------------------------------------------------"
printf "%-40s | %-15s | %-15s | %s\n" "Parameter" "$V1" "$V2" "Status"
echo "--------------------------------------------------------------------------------"

compare_param "Rotation threshold" "minimum_travel_heading"
compare_param "Distance threshold" "minimum_travel_distance"
compare_param "Angle variance penalty (odom trust)" "angle_variance_penalty"
compare_param "Distance variance penalty" "distance_variance_penalty"
compare_param "Map resolution" "resolution"
compare_param "Scan buffer size" "scan_buffer_size"
compare_param "Correlation search space" "correlation_search_space_dimension"
compare_param "Correlation smear deviation" "correlation_search_space_smear_deviation"
compare_param "Loop closure enabled" "do_loop_closing"
compare_param "Loop search distance" "loop_search_maximum_distance"

echo ""
echo "CALCULATED METRICS:"
echo "--------------------------------------------------------------------------------"

# Calculate scans per rotation
heading1=$(extract_param "$FILE1" "minimum_travel_heading")
heading2=$(extract_param "$FILE2" "minimum_travel_heading")

if [ -n "$heading1" ] && [ -n "$heading2" ]; then
    scans1=$(echo "scale=0; 6.28318 / $heading1" | bc)
    scans2=$(echo "scale=0; 6.28318 / $heading2" | bc)
    
    echo "Scans per 360° rotation:"
    printf "  %-15s: %s scans\n" "$V1" "$scans1"
    printf "  %-15s: %s scans\n" "$V2" "$scans2"
fi

echo ""
echo "FILES COMPARED:"
echo "  $FILE1"
echo "  $FILE2"
echo ""
echo "================================================================================"


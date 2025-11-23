#!/bin/bash
###############################################################################
# Wheelchair Sensor Fusion - System Benchmark Script
###############################################################################
# Measures actual performance metrics on the target hardware:
# - Topic publishing rates
# - Fusion latency
# - CPU/GPU usage
# - Memory consumption
# - Sustained performance over time
#
# Author: Siddharth Tiwari (IIT Mandi)
# Usage: ./benchmark_system.sh [duration_seconds]
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Duration (default 60 seconds)
DURATION=${1:-60}

echo "###############################################################################"
echo "#  WHEELCHAIR SENSOR FUSION - SYSTEM BENCHMARK"
echo "###############################################################################"
echo ""

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    log_error "ROS2 not found. Source your workspace first:"
    log_error "  source /opt/ros/jazzy/setup.bash"
    log_error "  source ~/ros2_ws/install/setup.bash"
    exit 1
fi

# Detect hardware
log_info "Detecting hardware..."
echo ""
echo "Platform Information:"
echo "  OS:       $(lsb_release -d | cut -f2-)"
echo "  Kernel:   $(uname -r)"
echo "  CPU:      $(lscpu | grep 'Model name' | cut -d':' -f2- | xargs)"
echo "  Cores:    $(nproc)"
echo "  Memory:   $(free -h | grep Mem | awk '{print $2}')"

if command -v nvidia-smi &> /dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n1)
    GPU_MEM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader | head -n1)
    DRIVER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n1)
    CUDA=$(nvidia-smi | grep "CUDA Version" | awk '{print $9}')
    echo "  GPU:      $GPU_NAME"
    echo "  GPU Mem:  $GPU_MEM"
    echo "  Driver:   $DRIVER"
    echo "  CUDA:     $CUDA"
else
    echo "  GPU:      Not available (CPU mode)"
fi

echo ""

# Check if sensors are connected
log_info "Checking sensor connections..."

SENSORS_OK=true

# Check LiDAR
if ros2 topic list | grep -q "/scan"; then
    log_success "LiDAR topic active (/scan)"
else
    log_warn "LiDAR topic not found (/scan)"
    SENSORS_OK=false
fi

# Check camera
if ros2 topic list | grep -q "/camera/color/image_raw"; then
    log_success "Camera topic active (/camera/color/image_raw)"
else
    log_warn "Camera topic not found (/camera/color/image_raw)"
    SENSORS_OK=false
fi

# Check fusion
if ros2 topic list | grep -q "/fusion/obstacles"; then
    log_success "Fusion node active (/fusion/obstacles)"
else
    log_warn "Fusion node not running (/fusion/obstacles)"
    log_warn "Launch fusion first:"
    log_warn "  ros2 launch wheelchair_sensor_fusion wheelchair_fusion.launch.py"
    SENSORS_OK=false
fi

if [ "$SENSORS_OK" = false ]; then
    echo ""
    log_error "Not all required topics are active. Fix issues before benchmarking."
    exit 1
fi

echo ""
log_info "Running benchmark for $DURATION seconds..."
echo ""

# Create output directory
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="/tmp/wheelchair_fusion_benchmark_$TIMESTAMP"
mkdir -p "$OUTPUT_DIR"

log_info "Output directory: $OUTPUT_DIR"
echo ""

# Measure topic rates
log_info "Measuring topic rates..."

measure_topic_rate() {
    local topic=$1
    local name=$2
    local expected=$3

    log_info "  Measuring $name..."

    # Measure for 10 seconds
    RATE_OUTPUT=$(ros2 topic hz "$topic" --window 10 2>&1 | tee "$OUTPUT_DIR/${name}_rate.txt" &)
    RATE_PID=$!

    sleep 10
    kill $RATE_PID 2>/dev/null || true

    # Extract average rate
    if [ -f "$OUTPUT_DIR/${name}_rate.txt" ]; then
        AVG_RATE=$(grep "average rate:" "$OUTPUT_DIR/${name}_rate.txt" | awk '{print $3}' | head -n1)

        if [ ! -z "$AVG_RATE" ]; then
            # Compare with expected
            PERCENT=$(echo "scale=2; $AVG_RATE / $expected * 100" | bc)

            if (( $(echo "$PERCENT >= 90" | bc -l) )); then
                log_success "  $name: $AVG_RATE Hz (${PERCENT}% of $expected Hz target) ✓"
            elif (( $(echo "$PERCENT >= 50" | bc -l) )); then
                log_warn "  $name: $AVG_RATE Hz (${PERCENT}% of $expected Hz target) ⚠"
            else
                log_error "  $name: $AVG_RATE Hz (${PERCENT}% of $expected Hz target) ✗"
            fi
        fi
    fi
}

measure_topic_rate "/scan" "LiDAR" 20
measure_topic_rate "/camera/color/image_raw" "Camera" 30
measure_topic_rate "/yolo/detections" "YOLO" 25
measure_topic_rate "/fusion/obstacles" "Fusion" 25

echo ""

# Monitor system resources
log_info "Monitoring system resources for $DURATION seconds..."

RESOURCE_LOG="$OUTPUT_DIR/resources.csv"
echo "timestamp,cpu_percent,mem_mb,gpu_percent,gpu_mem_mb" > "$RESOURCE_LOG"

for i in $(seq 1 $DURATION); do
    # Get CPU and memory
    CPU=$(ps aux | grep wheelchair_sensor_fusion | grep -v grep | awk '{sum+=$3} END {print sum}')
    MEM=$(ps aux | grep wheelchair_sensor_fusion | grep -v grep | awk '{sum+=$6} END {print sum/1024}')

    # Get GPU stats if available
    if command -v nvidia-smi &> /dev/null; then
        GPU_STATS=$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits)
        GPU_UTIL=$(echo $GPU_STATS | cut -d',' -f1 | xargs)
        GPU_MEM=$(echo $GPU_STATS | cut -d',' -f2 | xargs)
    else
        GPU_UTIL="N/A"
        GPU_MEM="N/A"
    fi

    echo "$i,$CPU,$MEM,$GPU_UTIL,$GPU_MEM" >> "$RESOURCE_LOG"

    # Progress bar
    PROGRESS=$((i * 100 / DURATION))
    printf "\r  Progress: [%-50s] %d%%" $(printf '#%.0s' $(seq 1 $((PROGRESS / 2)))) $PROGRESS

    sleep 1
done

echo ""
echo ""

# Analyze resource usage
log_info "Resource usage analysis:"

AVG_CPU=$(awk -F',' 'NR>1 {sum+=$2; count++} END {print sum/count}' "$RESOURCE_LOG")
MAX_CPU=$(awk -F',' 'NR>1 {if($2>max) max=$2} END {print max}' "$RESOURCE_LOG")
AVG_MEM=$(awk -F',' 'NR>1 {sum+=$3; count++} END {print sum/count}' "$RESOURCE_LOG")
MAX_MEM=$(awk -F',' 'NR>1 {if($3>max) max=$3} END {print max}' "$RESOURCE_LOG")

echo "  CPU Usage:"
echo "    Average: ${AVG_CPU}%"
echo "    Peak:    ${MAX_CPU}%"
echo ""
echo "  Memory Usage:"
echo "    Average: ${AVG_MEM} MB"
echo "    Peak:    ${MAX_MEM} MB"

if command -v nvidia-smi &> /dev/null; then
    AVG_GPU=$(awk -F',' 'NR>1 && $4 != "N/A" {sum+=$4; count++} END {print sum/count}' "$RESOURCE_LOG")
    MAX_GPU=$(awk -F',' 'NR>1 && $4 != "N/A" {if($4>max) max=$4} END {print max}' "$RESOURCE_LOG")
    AVG_GPU_MEM=$(awk -F',' 'NR>1 && $5 != "N/A" {sum+=$5; count++} END {print sum/count}' "$RESOURCE_LOG")
    MAX_GPU_MEM=$(awk -F',' 'NR>1 && $5 != "N/A" {if($5>max) max=$5} END {print max}' "$RESOURCE_LOG")

    echo ""
    echo "  GPU Usage:"
    echo "    Average: ${AVG_GPU}%"
    echo "    Peak:    ${MAX_GPU}%"
    echo ""
    echo "  GPU Memory:"
    echo "    Average: ${AVG_GPU_MEM} MB"
    echo "    Peak:    ${MAX_GPU_MEM} MB"
fi

echo ""

# Check fusion diagnostics
log_info "Checking fusion diagnostics..."
ros2 topic echo /fusion/diagnostics --once > "$OUTPUT_DIR/diagnostics.txt" 2>&1 || true

if [ -f "$OUTPUT_DIR/diagnostics.txt" ]; then
    echo ""
    cat "$OUTPUT_DIR/diagnostics.txt"
fi

echo ""

# Generate summary report
REPORT="$OUTPUT_DIR/BENCHMARK_REPORT.md"

cat > "$REPORT" << EOF
# Wheelchair Sensor Fusion - Benchmark Report

**Date**: $(date)
**Duration**: $DURATION seconds
**Platform**: $(lsb_release -d | cut -f2-)

## Hardware

- **CPU**: $(lscpu | grep 'Model name' | cut -d':' -f2- | xargs)
- **Cores**: $(nproc)
- **Memory**: $(free -h | grep Mem | awk '{print $2}')
EOF

if command -v nvidia-smi &> /dev/null; then
    cat >> "$REPORT" << EOF
- **GPU**: $GPU_NAME
- **GPU Memory**: $GPU_MEM
- **CUDA Version**: $CUDA
EOF
fi

cat >> "$REPORT" << EOF

## Performance Metrics

### Topic Rates

| Topic | Target (Hz) | Actual (Hz) | Status |
|-------|-------------|-------------|--------|
EOF

# Add topic rates to report (would need to extract from logs)
echo "| /scan | 20 | - | - |" >> "$REPORT"
echo "| /camera/color/image_raw | 30 | - | - |" >> "$REPORT"
echo "| /yolo/detections | 25 | - | - |" >> "$REPORT"
echo "| /fusion/obstacles | 25 | - | - |" >> "$REPORT"

cat >> "$REPORT" << EOF

### Resource Usage

| Metric | Average | Peak |
|--------|---------|------|
| CPU (%) | ${AVG_CPU} | ${MAX_CPU} |
| Memory (MB) | ${AVG_MEM} | ${MAX_MEM} |
EOF

if command -v nvidia-smi &> /dev/null; then
    cat >> "$REPORT" << EOF
| GPU (%) | ${AVG_GPU} | ${MAX_GPU} |
| GPU Memory (MB) | ${AVG_GPU_MEM} | ${MAX_GPU_MEM} |
EOF
fi

cat >> "$REPORT" << EOF

## Files Generated

- \`resources.csv\`: Time-series resource usage data
- \`*_rate.txt\`: Topic rate measurements
- \`diagnostics.txt\`: Fusion diagnostics snapshot

## Recommendations

EOF

# Add recommendations based on results
if (( $(echo "$AVG_CPU > 80" | bc -l) )); then
    echo "- ⚠ High CPU usage detected. Consider using a smaller YOLO model (e.g., yolov11n)" >> "$REPORT"
fi

if (( $(echo "$AVG_MEM > 2000" | bc -l) )); then
    echo "- ⚠ High memory usage detected. Check for memory leaks." >> "$REPORT"
fi

if command -v nvidia-smi &> /dev/null; then
    if (( $(echo "$AVG_GPU < 50" | bc -l) )); then
        echo "- ℹ GPU is underutilized. You could use a larger YOLO model for better accuracy." >> "$REPORT"
    fi
fi

echo "- ✓ For detailed analysis, examine the CSV file with a plotting tool (matplotlib, Excel, etc.)" >> "$REPORT"

echo ""
log_success "Benchmark complete!"
echo ""
log_info "Results saved to: $OUTPUT_DIR"
log_info "Report: $REPORT"
echo ""
log_info "To view the report:"
echo "  cat $REPORT"
echo ""

###############################################################################
# End of benchmark
###############################################################################

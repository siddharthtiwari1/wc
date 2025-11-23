#!/bin/bash
# Download all required models for RAN system

set -e

echo "================================================"
echo "Downloading Models for RAN Complete System"
echo "================================================"

# Create models directory
mkdir -p ~/.ran/models
cd ~/.ran/models

echo "[1/5] Downloading YOLO-World..."
if [ ! -f "yolov8x-worldv2.pt" ]; then
    wget -q --show-progress https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8x-worldv2.pt
    echo "✓ YOLO-World downloaded"
else
    echo "✓ YOLO-World already exists"
fi

echo "[2/5] Downloading SAM2..."
if [ ! -f "sam2_hiera_large.pt" ]; then
    echo "Note: SAM2 will be auto-downloaded on first use"
    echo "✓ SAM2 configured"
else
    echo "✓ SAM2 already exists"
fi

echo "[3/5] DINOv2 (auto-downloaded via torch.hub)"
echo "✓ DINOv2 will be downloaded on first use"

echo "[4/5] CLIP (auto-downloaded via open-clip)"
echo "✓ CLIP will be downloaded on first use"

echo "[5/5] LLaVA (optional, for attribute captioning)"
echo "Note: For full attribute extraction, install LLaVA separately:"
echo "  pip install llava"
echo "✓ LLaVA setup instructions provided"

echo ""
echo "================================================"
echo "Model Setup Complete!"
echo "================================================"
echo ""
echo "Models location: ~/.ran/models/"
echo ""
echo "Next steps:"
echo "1. Update config file with model paths"
echo "2. Run: ros2 launch ran_complete full_system.launch.py"
echo ""

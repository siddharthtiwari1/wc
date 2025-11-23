#!/bin/bash
# Quick environment diagnostic

echo "=========================================="
echo "ENVIRONMENT DIAGNOSTIC"
echo "=========================================="

echo -e "\n1. PYTHON"
echo "   Version: $(python3 --version)"
echo "   Path: $(which python3)"

echo -e "\n2. ROS2"
if [ -n "$ROS_DISTRO" ]; then
    echo "   ✅ ROS_DISTRO: $ROS_DISTRO"
else
    echo "   ⚠️  ROS_DISTRO not set"
    echo "   Run: source /opt/ros/jazzy/setup.bash"
fi

python3 -c "import rclpy; print('   ✅ rclpy: OK')" 2>/dev/null || echo "   ❌ rclpy: NOT FOUND"

echo -e "\n3. CUDA"
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi | grep "CUDA Version" | awk '{print "   ✅ CUDA: " $9}'
    nvidia-smi --query-gpu=name --format=csv,noheader | head -1 | awk '{print "   GPU: " $0}'
else
    echo "   ❌ nvidia-smi not found"
fi

echo -e "\n4. PYTHON PACKAGES"
python3 -c "import torch; print(f'   ✅ PyTorch: {torch.__version__}'); print(f'   CUDA: {torch.cuda.is_available()}')" 2>/dev/null || echo "   ❌ PyTorch not installed"
python3 -c "import jax; print(f'   ✅ JAX: {jax.__version__}')" 2>/dev/null || echo "   ❌ JAX not installed"

echo -e "\n5. ENVIRONMENT"
if [ -n "$VIRTUAL_ENV" ]; then
    echo "   ✅ venv: $VIRTUAL_ENV"
elif [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "   ⚠️  conda: $CONDA_DEFAULT_ENV (not recommended for ROS2)"
else
    echo "   ⚠️  No virtual environment (will use system Python)"
fi

echo -e "\n=========================================="
echo "RECOMMENDATION:"
if python3 -c "import rclpy, torch; assert torch.cuda.is_available()" 2>/dev/null; then
    echo "✅ YOUR ENVIRONMENT IS READY!"
else
    echo "⚠️  Run: ./scripts/setup_all.sh"
fi
echo "=========================================="

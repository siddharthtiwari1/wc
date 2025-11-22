# Environment Setup Guide - The FOOLPROOF Way

**Your System**: RTX 5050 | CUDA 13.0 | ROS2 Jazzy | Ubuntu

---

## ⚠️ The Critical Challenge

**ROS2 + Python ML packages = Environment Hell**

Why?
- ROS2 packages (rclpy, nav2) installed via `apt` → uses **system Python**
- PyTorch/JAX need specific CUDA versions → best managed with **conda/venv**
- Mixing them incorrectly = import errors, CUDA mismatches, pain

---

## ✅ RECOMMENDED: Python venv (Not Conda)

### Why venv > conda for ROS2?

| Aspect | venv | conda |
|--------|------|-------|
| ROS2 compatibility | ✅ Perfect | ⚠️ Can conflict |
| System Python access | ✅ Shares site-packages | ❌ Isolated |
| CUDA libraries | ✅ Uses system CUDA | ⚠️ May bundle own |
| Simplicity | ✅ Simple | ⚠️ Complex |
| ROS2 best practice | ✅ Recommended | ❌ Not recommended |

**The issue with conda**:
- Conda creates completely isolated environment
- ROS2's `rclpy` won't be visible in conda env
- You'd need to reinstall ROS2 in conda (messy!)
- CUDA paths can get confused

---

## 🚀 SOLUTION: venv + --system-site-packages

This gives you **best of both worlds**:
- ✅ Isolated ML packages (PyTorch, JAX)
- ✅ Access to system ROS2 packages
- ✅ Uses your existing CUDA 13.0
- ✅ No conflicts

### Step-by-Step Setup

```bash
# 1. Check your current Python
python3 --version  # Should be 3.10+
which python3      # Should be /usr/bin/python3

# 2. Check ROS2 is working
source /opt/ros/jazzy/setup.bash
python3 -c "import rclpy; print('ROS2 OK')"

# 3. Create venv WITH system packages
python3 -m venv ~/crowdnav_venv --system-site-packages

# 4. Activate
source ~/crowdnav_venv/bin/activate

# 5. Verify ROS2 still accessible
python3 -c "import rclpy; print('✅ ROS2 accessible in venv')"

# 6. Install ML packages
pip install --upgrade pip

# Install PyTorch with CUDA 13.0
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130

# Install JAX with CUDA 13.0
pip install "jax[cuda13_pip]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html

# 7. Install other ML deps
pip install -r ~/wc/requirements.txt

# 8. Verify everything works
python3 -c "
import rclpy
import torch
import jax
print(f'✅ ROS2: {rclpy.__version__}')
print(f'✅ PyTorch: {torch.__version__}')
print(f'✅ CUDA available: {torch.cuda.is_available()}')
print(f'✅ JAX: {jax.__version__}')
"
```

---

## 🧪 Testing Your Current Setup

Run this diagnostic script:

```bash
cat > ~/test_environment.py << 'EOF'
#!/usr/bin/env python3
"""Environment diagnostic script"""

import sys
print("="*60)
print("ENVIRONMENT DIAGNOSTIC")
print("="*60)

# 1. Python version
print(f"\n1. Python: {sys.version}")
print(f"   Executable: {sys.executable}")

# 2. Check ROS2
try:
    import rclpy
    print(f"✅ 2. ROS2 (rclpy): {rclpy.__version__}")
except ImportError as e:
    print(f"❌ 2. ROS2 (rclpy): NOT FOUND - {e}")

# 3. Check PyTorch
try:
    import torch
    print(f"✅ 3. PyTorch: {torch.__version__}")
    print(f"   CUDA available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"   CUDA version: {torch.version.cuda}")
        print(f"   GPU: {torch.cuda.get_device_name(0)}")
        print(f"   GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
except ImportError as e:
    print(f"❌ 3. PyTorch: NOT FOUND - {e}")

# 4. Check JAX
try:
    import jax
    print(f"✅ 4. JAX: {jax.__version__}")
    print(f"   Devices: {jax.devices()}")
except ImportError as e:
    print(f"❌ 4. JAX: NOT FOUND - {e}")

# 5. Check other critical packages
packages = [
    'numpy', 'scipy', 'open3d', 'hydra',
    'diffusers', 'accelerate', 'tensorboard'
]

print("\n5. Other packages:")
for pkg in packages:
    try:
        mod = __import__(pkg)
        version = getattr(mod, '__version__', 'unknown')
        print(f"   ✅ {pkg}: {version}")
    except ImportError:
        print(f"   ❌ {pkg}: NOT FOUND")

print("\n" + "="*60)
print("RECOMMENDATION:")

# Determine recommendation
has_ros2 = False
has_torch = False
try:
    import rclpy
    has_ros2 = True
except:
    pass

try:
    import torch
    has_torch = torch.cuda.is_available()
except:
    pass

if has_ros2 and has_torch:
    print("✅ YOUR ENVIRONMENT IS READY!")
    print("   Just install missing packages from requirements.txt")
elif has_ros2 and not has_torch:
    print("⚠️  ROS2 OK, but PyTorch/CUDA needs installation")
    print("   Run: pip install torch --index-url https://download.pytorch.org/whl/cu130")
elif not has_ros2:
    print("❌ ROS2 not accessible")
    print("   If using conda: deactivate and use venv instead")
    print("   If using venv: recreate with --system-site-packages")
else:
    print("⚠️  Something is wrong. Check setup.")

print("="*60)
EOF

chmod +x ~/test_environment.py

# Run it
python3 ~/test_environment.py
```

---

## 🔧 Your Current Setup Analysis

Let me check what you have:

```bash
# Run this and paste output:
echo "=== SYSTEM INFO ==="
echo "Python: $(python3 --version)"
echo "ROS2: $ROS_DISTRO"
echo ""

echo "=== CONDA CHECK ==="
conda --version 2>/dev/null && echo "Conda: INSTALLED" || echo "Conda: NOT INSTALLED"
echo "CONDA_DEFAULT_ENV: $CONDA_DEFAULT_ENV"
echo ""

echo "=== VIRTUAL ENV CHECK ==="
echo "VIRTUAL_ENV: $VIRTUAL_ENV"
echo ""

echo "=== CUDA CHECK ==="
nvcc --version 2>/dev/null | grep release || echo "nvcc not in PATH"
nvidia-smi | grep "CUDA Version"
echo ""

echo "=== EXISTING PACKAGES ==="
python3 -c "import torch; print(f'PyTorch: {torch.__version__}')" 2>/dev/null || echo "PyTorch: NOT INSTALLED"
python3 -c "import rclpy; print(f'ROS2: OK')" 2>/dev/null || echo "ROS2: NOT ACCESSIBLE"
```

---

## 📋 Decision Tree

### Scenario 1: You have NOTHING installed yet
**→ Use venv (my setup script does this)**
```bash
./scripts/setup_all.sh  # This handles everything
```

### Scenario 2: You already use conda for other projects
**→ Create NEW venv for this project**
```bash
# Keep your conda for other work
# Use venv just for ROS2 + CrowdNav
python3 -m venv ~/crowdnav_venv --system-site-packages
source ~/crowdnav_venv/bin/activate
./scripts/setup_all.sh  # Will detect venv and use it
```

### Scenario 3: You have some packages installed globally
**→ Use venv to avoid breaking existing setup**
```bash
python3 -m venv ~/crowdnav_venv --system-site-packages
source ~/crowdnav_venv/bin/activate
pip install -r requirements.txt
```

### Scenario 4: You insist on using conda
**→ Possible but requires extra work**
```bash
# Create conda env with Python 3.10
conda create -n crowdnav python=3.10 -y
conda activate crowdnav

# Install ROS2 Python packages manually
pip install rclpy rosdep colcon-common-extensions

# Install CUDA toolkit in conda (may conflict!)
conda install cudatoolkit=13.0 -c nvidia

# Install PyTorch
pip install torch --index-url https://download.pytorch.org/whl/cu130

# Install rest
pip install -r requirements.txt

# Every time you use it:
conda activate crowdnav
source /opt/ros/jazzy/setup.bash  # Source ROS2 AFTER conda
```

⚠️ **Not recommended** due to potential conflicts

---

## ✅ RECOMMENDED WORKFLOW

### Daily Usage (after setup):

```bash
# Terminal 1 - Activate once per terminal
source ~/crowdnav_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/wc/install/setup.bash

# Or use the auto-generated helper:
source ~/wc/activate.sh  # Does all 3 above

# Now everything works:
ros2 launch crowdsurfer_nav full_simulation.launch.py
python3 scripts/train_vqvae.py
```

### For Development:

```bash
# Jupyter notebook (optional)
pip install jupyter
jupyter notebook

# VSCode (optional)
# Just select ~/crowdnav_venv/bin/python as interpreter
```

---

## 🚨 Common Issues & Fixes

### Issue 1: "ModuleNotFoundError: No module named 'rclpy'"

**Cause**: venv created without `--system-site-packages`

**Fix**:
```bash
rm -rf ~/crowdnav_venv
python3 -m venv ~/crowdnav_venv --system-site-packages
source ~/crowdnav_venv/bin/activate
pip install -r requirements.txt
```

### Issue 2: "CUDA not available" in PyTorch

**Cause**: Wrong PyTorch wheel (CPU-only)

**Fix**:
```bash
pip uninstall torch torchvision torchaudio
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130
```

### Issue 3: JAX doesn't see GPU

**Cause**: Wrong JAX version or CUDA path

**Fix**:
```bash
pip uninstall jax jaxlib
pip install "jax[cuda13_pip]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html

# Verify
python3 -c "import jax; print(jax.devices())"
```

### Issue 4: Import conflicts between packages

**Cause**: Package version incompatibilities

**Fix**:
```bash
# Nuke and reinstall
pip freeze | grep -v "^-e" | xargs pip uninstall -y
pip install -r requirements.txt
```

---

## 💡 MY RECOMMENDATION FOR YOU

Based on your setup (RTX 5050, CUDA 13.0, ROS2 Jazzy):

1. **Use Python venv** (not conda)
2. **Run my automated setup script**
3. **It handles everything correctly**

### One Command to Rule Them All:

```bash
cd ~/wc
./scripts/setup_all.sh
```

This script:
- ✅ Detects your CUDA 13.0
- ✅ Creates venv with --system-site-packages
- ✅ Installs PyTorch for CUDA 13.0
- ✅ Installs JAX for CUDA 13.0
- ✅ Preserves ROS2 access
- ✅ Builds ROS2 workspace
- ✅ Creates activation helper
- ✅ Tests everything

---

## 🎯 NEXT STEP

**Run the diagnostic script first:**

```bash
cd ~
python3 ~/wc/test_environment.py
```

**Paste the output here**, and I'll tell you:
1. If you need to create a new environment
2. What's missing
3. Exact commands to fix it

Then we proceed with full implementation!

---

**Bottom line**:
- ❌ Don't use conda (causes ROS2 headaches)
- ✅ Use venv with --system-site-packages
- ✅ My setup script handles it all
- ✅ Less than 5 minutes to full working environment

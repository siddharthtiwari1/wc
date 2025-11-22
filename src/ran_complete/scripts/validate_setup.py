#!/usr/bin/env python3
"""
RAN Setup Validation Script

Validates that all dependencies, models, and hardware are properly configured
before deploying the RAN system.

Author: Siddharth Tiwari
"""

import os
import sys
import subprocess
from pathlib import Path
from typing import Tuple, List
import importlib


class Colors:
    """Terminal color codes."""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'


def print_check(passed: bool, message: str):
    """Print colored check result."""
    if passed:
        print(f"{Colors.GREEN}✓{Colors.END} {message}")
    else:
        print(f"{Colors.RED}✗{Colors.END} {message}")


def print_header(title: str):
    """Print section header."""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*70}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{title}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*70}{Colors.END}\n")


def check_ros2() -> Tuple[bool, str]:
    """Check ROS2 Jazzy installation."""
    try:
        result = subprocess.run(
            ['ros2', '--version'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            version = result.stdout.strip()
            if 'jazzy' in version.lower():
                return True, f"ROS2 Jazzy detected: {version}"
            else:
                return False, f"Wrong ROS2 version: {version} (need Jazzy)"
        return False, "ROS2 not responding"
    except Exception as e:
        return False, f"ROS2 check failed: {e}"


def check_python() -> Tuple[bool, str]:
    """Check Python version."""
    version = sys.version_info
    if version.major == 3 and version.minor >= 10:
        return True, f"Python {version.major}.{version.minor}.{version.micro} found"
    return False, f"Python {version.major}.{version.minor} too old (need 3.10+)"


def check_cuda() -> Tuple[bool, str]:
    """Check CUDA availability."""
    try:
        import torch
        if torch.cuda.is_available():
            gpu_name = torch.cuda.get_device_name(0)
            return True, f"CUDA available (GPU: {gpu_name})"
        return False, "CUDA not available (will use CPU - slower)"
    except ImportError:
        return False, "PyTorch not installed"


def check_python_packages() -> Tuple[bool, str]:
    """Check required Python packages."""
    required_packages = [
        ('torch', 'PyTorch'),
        ('torchvision', 'TorchVision'),
        ('ultralytics', 'YOLO-World'),
        ('transformers', 'Transformers'),
        ('cv2', 'OpenCV'),
        ('numpy', 'NumPy'),
        ('scipy', 'SciPy'),
        ('sklearn', 'scikit-learn'),
    ]

    missing = []
    for pkg_import, pkg_name in required_packages:
        try:
            importlib.import_module(pkg_import)
        except ImportError:
            missing.append(pkg_name)

    if not missing:
        return True, f"All {len(required_packages)} required packages installed"
    return False, f"Missing packages: {', '.join(missing)}"


def check_models() -> Tuple[bool, str]:
    """Check if pretrained models are downloaded."""
    models_dir = Path.home() / '.cache' / 'torch' / 'hub' / 'checkpoints'
    models_dir.mkdir(parents=True, exist_ok=True)

    required_models = {
        'yolov8x-worldv2.pt': 237,  # MB
        'sam2_hiera_large.pt': 896,
        # Note: DINOv2 and CLIP download automatically on first use
    }

    found_models = []
    missing_models = []

    for model_name, expected_size_mb in required_models.items():
        model_path = models_dir / model_name
        if model_path.exists():
            size_mb = model_path.stat().st_size / (1024 * 1024)
            found_models.append(f"{model_name} ({size_mb:.0f} MB)")
        else:
            missing_models.append(model_name)

    if not missing_models:
        return True, f"All models downloaded:\n    " + "\n    ".join(found_models)
    return False, f"Missing models: {', '.join(missing_models)}\n    Run: ./scripts/download_models.sh"


def check_realsense() -> Tuple[bool, str]:
    """Check RealSense camera connection."""
    try:
        # Try to list devices using rs-enumerate-devices
        result = subprocess.run(
            ['rs-enumerate-devices', '-s'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0 and 'D455' in result.stdout:
            return True, "RealSense D455 connected"
        elif result.returncode == 0:
            return False, "RealSense connected but not D455"
        return False, "No RealSense camera detected"
    except FileNotFoundError:
        return False, "rs-enumerate-devices not found (install librealsense2-tools)"
    except Exception as e:
        return False, f"RealSense check failed: {e}"


def check_rplidar() -> Tuple[bool, str]:
    """Check RPLidar connection."""
    # Check for common USB serial ports
    usb_devices = list(Path('/dev').glob('ttyUSB*'))

    if not usb_devices:
        return False, "No /dev/ttyUSB* devices found (RPLidar likely disconnected)"

    # Check permissions
    for device in usb_devices:
        if os.access(device, os.R_OK | os.W_OK):
            return True, f"RPLidar detected at {device}"

    return False, f"USB devices found but no read/write permissions: {usb_devices[0]}\n    Run: sudo chmod 666 {usb_devices[0]}"


def check_ros2_topics() -> Tuple[bool, str]:
    """Check if required ROS2 topics are being published."""
    try:
        # This check only works if ROS2 nodes are running
        # So we make it a warning, not a failure
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')

            required_topics = ['/camera/color/image_raw', '/scan', '/odom']
            found = [t for t in required_topics if any(t in topic for topic in topics)]

            if len(found) == len(required_topics):
                return True, "All required topics active"
            elif found:
                return False, f"Some topics active: {found}\n    Missing: {set(required_topics) - set(found)}"
            else:
                return False, "No required topics active (launch hardware nodes first)"
        return False, "Could not list ROS2 topics"
    except Exception as e:
        return False, f"ROS2 topics check skipped: {e}"


def check_workspace() -> Tuple[bool, str]:
    """Check if ran_complete package is built."""
    ws_path = Path('/home/user/wc')
    install_path = ws_path / 'install' / 'ran_complete'

    if install_path.exists():
        return True, f"ran_complete package built in {ws_path}"
    return False, f"ran_complete not built. Run: cd {ws_path} && colcon build"


def main():
    """Run all validation checks."""
    print(f"\n{Colors.BOLD}RAN SETUP VALIDATION{Colors.END}")
    print(f"{Colors.BOLD}Checking system readiness for wheelchair navigation{Colors.END}\n")

    checks = [
        ("System Requirements", [
            ("ROS2 Jazzy", check_ros2),
            ("Python Version", check_python),
            ("CUDA/GPU", check_cuda),
            ("Python Packages", check_python_packages),
            ("Workspace Build", check_workspace),
        ]),
        ("Pretrained Models", [
            ("Model Files", check_models),
        ]),
        ("Hardware Connections", [
            ("RealSense D455", check_realsense),
            ("RPLidar S3", check_rplidar),
        ]),
        ("Runtime Checks (Optional)", [
            ("ROS2 Topics", check_ros2_topics),
        ]),
    ]

    all_passed = True
    critical_failed = False

    for section_name, section_checks in checks:
        print_header(section_name)

        for check_name, check_func in section_checks:
            passed, message = check_func()
            print_check(passed, f"{check_name}:")
            if message:
                for line in message.split('\n'):
                    print(f"    {line}")

            if not passed:
                all_passed = False
                # Critical checks
                if section_name in ["System Requirements", "Pretrained Models"]:
                    critical_failed = True

    # Final summary
    print_header("VALIDATION SUMMARY")

    if all_passed:
        print(f"{Colors.GREEN}{Colors.BOLD}✓ ALL CHECKS PASSED!{Colors.END}")
        print(f"\n{Colors.GREEN}System ready for deployment!{Colors.END}")
        print(f"\nNext steps:")
        print(f"  1. ros2 launch ran_complete full_system.launch.py")
        print(f"  2. ros2 topic pub /ran/instruction std_msgs/String \"data: 'Go to the red chair'\"")
        sys.exit(0)
    elif critical_failed:
        print(f"{Colors.RED}{Colors.BOLD}✗ CRITICAL CHECKS FAILED{Colors.END}")
        print(f"\n{Colors.RED}System NOT ready. Fix errors above before deployment.{Colors.END}")
        sys.exit(1)
    else:
        print(f"{Colors.YELLOW}{Colors.BOLD}⚠ SOME CHECKS FAILED{Colors.END}")
        print(f"\n{Colors.YELLOW}System may work with reduced functionality.{Colors.END}")
        print(f"Fix warnings above for full performance.")
        sys.exit(0)


if __name__ == '__main__':
    main()

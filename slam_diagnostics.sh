#!/bin/bash
# SLAM Diagnostic Script - Find the root cause of random obstacles

echo "========================================="
echo "SLAM DIAGNOSTIC TOOL"
echo "========================================="
echo ""

echo "1. Checking TF Tree..."
echo "---"
ros2 run tf2_tools view_frames
echo "✓ TF tree saved to frames.pdf"
echo ""

echo "2. Checking Odometry Data Quality..."
echo "---"
echo "Check if odometry is publishing reasonable values:"
timeout 5 ros2 topic echo /wc_control/odom --once
echo ""

echo "3. Checking EKF Filtered Odometry..."
echo "---"
timeout 5 ros2 topic echo /odometry/filtered --once
echo ""

echo "4. Checking Scan Data..."
echo "---"
echo "Scan topic info:"
ros2 topic info /scan
echo ""
echo "One scan sample:"
timeout 5 ros2 topic echo /scan --field ranges --once | head -20
echo ""

echo "5. Checking Topic Rates..."
echo "---"
echo "Odometry rate (should be ~50Hz):"
timeout 5 ros2 topic hz /wc_control/odom
echo ""
echo "Filtered odometry rate (should be ~30Hz):"
timeout 5 ros2 topic hz /odometry/filtered
echo ""
echo "Scan rate (should be 10-20Hz for S3):"
timeout 5 ros2 topic hz /scan
echo ""

echo "6. Checking TF Delays..."
echo "---"
ros2 run tf2_ros tf2_echo odom base_link
echo ""

echo "7. Checking for TF warnings..."
echo "---"
echo "Run: ros2 topic echo /tf_static"
echo "Look for duplicate transforms or conflicts"
echo ""

echo "========================================="
echo "ANALYSIS CHECKLIST:"
echo "========================================="
echo "[ ] TF tree is correct (odom->base_link from EKF, map->odom from SLAM)"
echo "[ ] /wc_control/odom shows reasonable x,y,theta values"
echo "[ ] /odometry/filtered is being published"
echo "[ ] /scan has valid ranges (not all inf or 0)"
echo "[ ] All topic rates are in expected range"
echo "[ ] No TF transform delays or warnings"
echo ""
echo "If any checks fail, that's your root cause!"
echo "========================================="

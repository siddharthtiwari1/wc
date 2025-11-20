#!/bin/bash
# Wrapper to run square odometry test
source /home/sidd/wc_ws/install/setup.bash
python3 /home/sidd/wc_ws/src/scripts/scripts/square_odometry_test.py "$@"

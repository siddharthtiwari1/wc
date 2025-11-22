#!/bin/bash
###############################################################################
# Sensor Testing Script
#
# Tests RealSense camera and RPLidar connectivity
#
# Author: Siddharth Tiwari
###############################################################################

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Wheelchair Sensor Test${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Test RealSense
echo -e "${YELLOW}[1/2] Testing RealSense camera...${NC}"
if command -v rs-enumerate-devices &> /dev/null; then
    if rs-enumerate-devices | grep -q "Intel RealSense"; then
        echo -e "${GREEN}✓ RealSense camera detected${NC}"
        rs-enumerate-devices | grep "Name\|Serial Number"
    else
        echo -e "${RED}✗ No RealSense camera found${NC}"
        echo -e "${YELLOW}  Make sure the camera is connected${NC}"
    fi
else
    echo -e "${RED}✗ RealSense tools not installed${NC}"
    echo -e "${YELLOW}  Run: sudo apt install librealsense2-utils${NC}"
fi

echo ""

# Test RPLidar
echo -e "${YELLOW}[2/2] Testing RPLidar...${NC}"
if ls /dev/ttyUSB* &> /dev/null; then
    echo -e "${GREEN}✓ USB serial devices found:${NC}"
    ls -l /dev/ttyUSB*

    # Check permissions
    if [ -r /dev/ttyUSB0 ] && [ -w /dev/ttyUSB0 ]; then
        echo -e "${GREEN}✓ Permissions OK for /dev/ttyUSB0${NC}"
    else
        echo -e "${RED}✗ No read/write permissions for /dev/ttyUSB0${NC}"
        echo -e "${YELLOW}  Run: sudo chmod 666 /dev/ttyUSB0${NC}"
        echo -e "${YELLOW}  Or run setup_permissions.sh script${NC}"
    fi
else
    echo -e "${RED}✗ No USB serial devices found${NC}"
    echo -e "${YELLOW}  Make sure the LiDAR is connected${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Test Complete${NC}"
echo -e "${GREEN}========================================${NC}"

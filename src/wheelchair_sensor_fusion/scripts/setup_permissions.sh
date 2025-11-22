#!/bin/bash
###############################################################################
# Setup USB Permissions for LiDAR and Camera
#
# This script sets up udev rules for automatic USB permission handling
#
# Author: Siddharth Tiwari
###############################################################################

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Setting up USB permissions...${NC}"

# Create udev rules for RPLidar
echo -e "${YELLOW}Creating udev rules for RPLidar...${NC}"
sudo tee /etc/udev/rules.d/99-rplidar.rules > /dev/null <<EOF
# RPLidar USB Serial
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
EOF

# Create udev rules for RealSense
echo -e "${YELLOW}Creating udev rules for RealSense...${NC}"
sudo tee /etc/udev/rules.d/99-realsense-libusb.rules > /dev/null <<EOF
# Intel RealSense D400 series
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0666", GROUP:="plugdev"
EOF

# Add user to dialout group
echo -e "${YELLOW}Adding user to dialout group...${NC}"
sudo usermod -a -G dialout $USER

# Reload udev rules
echo -e "${YELLOW}Reloading udev rules...${NC}"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo -e "${GREEN}USB permissions configured!${NC}"
echo -e "${YELLOW}Note: You may need to log out and log back in for group changes to take effect${NC}"
echo ""

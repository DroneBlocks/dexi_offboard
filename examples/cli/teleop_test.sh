#!/bin/bash
# DEXI Keyboard Teleop Test Script
#
# This script helps test the keyboard teleop functionality by:
# 1. Starting the offboard heartbeat
# 2. Arming the drone (optional)
# 3. Running the keyboard teleop for yaw control
#
# Usage: ./teleop_test.sh

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

TOPIC="/dexi/offboard_manager"

echo -e "${BLUE}DEXI Keyboard Teleop Test${NC}"
echo ""

# Function to send command
send_command() {
    local command=$1
    local value=${2:-0.0}
    local description=$3

    echo -e "${GREEN}$description${NC}"
    ros2 topic pub --once $TOPIC dexi_interfaces/msg/OffboardNavCommand "{command: '$command', distance_or_degrees: $value}"
    sleep 2
}

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS2 not found. Please source your ROS2 environment.${NC}"
    exit 1
fi

echo -e "${YELLOW}Step 1: Starting offboard heartbeat${NC}"
send_command "start_offboard_heartbeat" 0.0 "Enabling offboard mode..."

echo ""
echo -e "${YELLOW}Optional: Arm the drone? (y/N)${NC}"
read -p "Arm drone for flight testing? " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    send_command "arm" 0.0 "Arming drone..."
    echo -e "${YELLOW}Drone is now armed! Be careful!${NC}"
else
    echo -e "${YELLOW}Drone remains disarmed - safe for testing${NC}"
fi

echo ""
echo -e "${GREEN}Now starting keyboard teleop...${NC}"
echo -e "${YELLOW}In the teleop window:${NC}"
echo "  - Press 'A' to yaw LEFT 15 degrees"
echo "  - Press 'D' to yaw RIGHT 15 degrees"
echo "  - Press 'Q' to quit"
echo ""
echo -e "${BLUE}Starting keyboard teleop in 3 seconds...${NC}"
sleep 3

# Start keyboard teleop
ros2 run dexi_offboard keyboard_teleop
#!/bin/bash
# DEXI Offboard Manager - Basic Flight Script
#
# This script demonstrates a simple arm, takeoff, and land sequence using
# command line ROS2 topic publishing.
#
# Usage: ./basic_flight.sh
#
# Prerequisites:
# - ROS2 environment sourced
# - dexi_offboard node running
# - dexi_interfaces package available

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Flight parameters
TAKEOFF_ALTITUDE=2.0
TOPIC="/dexi/offboard_manager"

echo -e "${BLUE}DEXI Offboard Manager - Basic Flight Script${NC}"
echo -e "${YELLOW}This script will execute: arm -> takeoff -> land -> disarm${NC}"
echo ""

# Function to send command
send_command() {
    local command=$1
    local value=${2:-0.0}
    local description=$3

    echo -e "${GREEN}$description${NC}"
    ros2 topic pub --once $TOPIC dexi_interfaces/msg/OffboardNavCommand "{command: '$command', distance_or_degrees: $value}"

    # Wait for command to be processed
    sleep 2
}

# Function to wait with countdown
wait_with_countdown() {
    local seconds=$1
    local message=$2

    echo -e "${YELLOW}$message${NC}"
    for ((i=seconds; i>0; i--)); do
        echo -n "$i... "
        sleep 1
    done
    echo "✓"
}

# Main flight sequence
main() {
    echo -e "${BLUE}Starting flight sequence...${NC}"
    echo ""

    # Step 1: Start offboard heartbeat
    send_command "start_offboard_heartbeat" 0.0 "Step 1: Starting offboard heartbeat"
    wait_with_countdown 3 "Waiting for offboard mode to initialize..."

    # Step 2: Arm vehicle
    send_command "arm" 0.0 "Step 2: Arming vehicle"
    wait_with_countdown 3 "Waiting for arm confirmation..."

    # Step 3: Takeoff
    send_command "takeoff" $TAKEOFF_ALTITUDE "Step 3: Taking off to ${TAKEOFF_ALTITUDE}m"
    wait_with_countdown 8 "Waiting for takeoff to complete..."

    # Step 4: Hold position
    wait_with_countdown 5 "Holding position for safety..."

    # Step 5: Land
    send_command "land" 0.0 "Step 5: Landing"
    wait_with_countdown 8 "Waiting for landing to complete..."

    # Step 6: Disarm
    send_command "disarm" 0.0 "Step 6: Disarming vehicle"
    wait_with_countdown 2 "Waiting for disarm confirmation..."

    # Step 7: Stop heartbeat
    send_command "stop_offboard_heartbeat" 0.0 "Step 7: Stopping offboard heartbeat"

    echo ""
    echo -e "${GREEN}✓ Flight sequence completed successfully!${NC}"
}

# Error handling
cleanup() {
    echo ""
    echo -e "${RED}Script interrupted! Attempting emergency shutdown...${NC}"

    # Emergency landing sequence
    echo -e "${YELLOW}Sending emergency land command...${NC}"
    ros2 topic pub --once $TOPIC dexi_interfaces/msg/OffboardNavCommand "{command: 'land'}" || true
    sleep 5

    echo -e "${YELLOW}Sending disarm command...${NC}"
    ros2 topic pub --once $TOPIC dexi_interfaces/msg/OffboardNavCommand "{command: 'disarm'}" || true
    sleep 2

    echo -e "${YELLOW}Stopping offboard heartbeat...${NC}"
    ros2 topic pub --once $TOPIC dexi_interfaces/msg/OffboardNavCommand "{command: 'stop_offboard_heartbeat'}" || true

    echo -e "${RED}Emergency shutdown completed.${NC}"
    exit 1
}

# Set up signal handling
trap cleanup SIGINT SIGTERM

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS2 not found. Please source your ROS2 environment.${NC}"
    exit 1
fi

# Check if dexi_interfaces is available
if ! ros2 interface show dexi_interfaces/msg/OffboardNavCommand &> /dev/null; then
    echo -e "${RED}Error: dexi_interfaces package not found. Please ensure it's built and sourced.${NC}"
    exit 1
fi

# Safety confirmation
echo -e "${YELLOW}Safety Check:${NC}"
echo "- Is the offboard manager node running?"
echo "- Is the vehicle ready for flight?"
echo "- Is the flight area clear?"
echo ""
read -p "Continue with flight? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Flight cancelled."
    exit 0
fi

# Execute main flight sequence
main
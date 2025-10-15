#!/bin/bash
# DEXI Offboard Manager - Manual Command Reference
#
# This script provides easy-to-use functions for manually controlling the drone
# via command line. Source this file to get access to helper functions.
#
# Usage:
#   source manual_commands.sh
#   start_heartbeat
#   arm_drone
#   takeoff_drone 3.0
#   land_drone
#   etc.

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Topic configuration
TOPIC="/dexi/offboard_manager"

# Helper function to send commands
send_dexi_command() {
    local command=$1
    local value=${2:-0.0}
    local description=${3:-"Sending command: $command"}

    echo -e "${GREEN}$description${NC}"
    ros2 topic pub --once $TOPIC dexi_interfaces/msg/OffboardNavCommand "{command: '$command', distance_or_degrees: $value}"
}

# =============================================================================
# BASIC CONTROL FUNCTIONS
# =============================================================================

# Start offboard heartbeat (required first)
start_heartbeat() {
    send_dexi_command "start_offboard_heartbeat" 0.0 "Starting offboard heartbeat..."
}

# Stop offboard heartbeat
stop_heartbeat() {
    send_dexi_command "stop_offboard_heartbeat" 0.0 "Stopping offboard heartbeat..."
}

# Arm the drone
arm_drone() {
    send_dexi_command "arm" 0.0 "Arming drone..."
}

# Disarm the drone
disarm_drone() {
    send_dexi_command "disarm" 0.0 "Disarming drone..."
}

# Takeoff to specified altitude (default 2m)
takeoff_drone() {
    local altitude=${1:-2.0}
    send_dexi_command "takeoff" $altitude "Taking off to ${altitude}m..."
}

# Land the drone
land_drone() {
    send_dexi_command "land" 0.0 "Landing drone..."
}

# =============================================================================
# MOVEMENT FUNCTIONS
# =============================================================================

# Move forward by distance (meters)
move_forward() {
    local distance=${1:-1.0}
    send_dexi_command "fly_forward" $distance "Moving forward ${distance}m..."
}

# Move backward by distance (meters)
move_backward() {
    local distance=${1:-1.0}
    send_dexi_command "fly_backward" $distance "Moving backward ${distance}m..."
}

# Move left by distance (meters)
move_left() {
    local distance=${1:-1.0}
    send_dexi_command "fly_left" $distance "Moving left ${distance}m..."
}

# Move right by distance (meters)
move_right() {
    local distance=${1:-1.0}
    send_dexi_command "fly_right" $distance "Moving right ${distance}m..."
}

# Move up by distance (meters)
move_up() {
    local distance=${1:-1.0}
    send_dexi_command "fly_up" $distance "Moving up ${distance}m..."
}

# Move down by distance (meters)
move_down() {
    local distance=${1:-1.0}
    send_dexi_command "fly_down" $distance "Moving down ${distance}m..."
}

# =============================================================================
# ROTATION FUNCTIONS
# =============================================================================

# Rotate left by angle (degrees)
rotate_left() {
    local angle=${1:-45.0}
    send_dexi_command "yaw_left" $angle "Rotating left ${angle}°..."
}

# Rotate right by angle (degrees)
rotate_right() {
    local angle=${1:-45.0}
    send_dexi_command "yaw_right" $angle "Rotating right ${angle}°..."
}

# =============================================================================
# CONVENIENCE FUNCTIONS
# =============================================================================

# Emergency stop - land and disarm immediately
emergency_stop() {
    echo -e "${RED}EMERGENCY STOP INITIATED${NC}"
    land_drone
    sleep 3
    disarm_drone
    stop_heartbeat
}

# Quick takeoff sequence
quick_takeoff() {
    local altitude=${1:-2.0}
    echo -e "${BLUE}Quick takeoff sequence to ${altitude}m${NC}"
    start_heartbeat
    sleep 2
    arm_drone
    sleep 2
    takeoff_drone $altitude
}

# Quick landing sequence
quick_land() {
    echo -e "${BLUE}Quick landing sequence${NC}"
    land_drone
    sleep 5
    disarm_drone
    sleep 1
    stop_heartbeat
}

# Test connection to offboard manager
test_connection() {
    echo -e "${BLUE}Testing connection to offboard manager...${NC}"

    # Check if topic exists
    if ros2 topic list | grep -q "$TOPIC"; then
        echo -e "${GREEN}✓ Topic $TOPIC found${NC}"
    else
        echo -e "${RED}✗ Topic $TOPIC not found${NC}"
        return 1
    fi

    # Check if we can see the topic info
    if ros2 topic info $TOPIC &> /dev/null; then
        echo -e "${GREEN}✓ Topic info available${NC}"
    else
        echo -e "${RED}✗ Cannot get topic info${NC}"
        return 1
    fi

    echo -e "${GREEN}✓ Connection test successful${NC}"
}

# =============================================================================
# HELP FUNCTION
# =============================================================================

show_help() {
    echo -e "${BLUE}DEXI Offboard Manager - Manual Commands${NC}"
    echo ""
    echo -e "${YELLOW}Basic Control:${NC}"
    echo "  start_heartbeat        - Start offboard heartbeat (required first)"
    echo "  stop_heartbeat         - Stop offboard heartbeat"
    echo "  arm_drone              - Arm the drone"
    echo "  disarm_drone           - Disarm the drone"
    echo "  takeoff_drone [alt]    - Takeoff to altitude (default: 2.0m)"
    echo "  land_drone             - Land the drone"
    echo ""
    echo -e "${YELLOW}Movement (distance in meters):${NC}"
    echo "  move_forward [dist]    - Move forward (default: 1.0m)"
    echo "  move_backward [dist]   - Move backward (default: 1.0m)"
    echo "  move_left [dist]       - Move left (default: 1.0m)"
    echo "  move_right [dist]      - Move right (default: 1.0m)"
    echo "  move_up [dist]         - Move up (default: 1.0m)"
    echo "  move_down [dist]       - Move down (default: 1.0m)"
    echo ""
    echo -e "${YELLOW}Rotation (angle in degrees):${NC}"
    echo "  rotate_left [angle]    - Rotate left (default: 45°)"
    echo "  rotate_right [angle]   - Rotate right (default: 45°)"
    echo ""
    echo -e "${YELLOW}Convenience Functions:${NC}"
    echo "  quick_takeoff [alt]    - Complete takeoff sequence"
    echo "  quick_land             - Complete landing sequence"
    echo "  emergency_stop         - Emergency land and disarm"
    echo "  test_connection        - Test connection to offboard manager"
    echo "  show_help              - Show this help message"
    echo ""
    echo -e "${YELLOW}Example Usage:${NC}"
    echo "  quick_takeoff 3.0      # Takeoff to 3 meters"
    echo "  move_forward 5.0       # Move forward 5 meters"
    echo "  rotate_right 90        # Turn right 90 degrees"
    echo "  quick_land             # Land and shutdown"
}

# Display help when sourced
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo -e "${GREEN}DEXI manual commands loaded successfully!${NC}"
    echo "Type 'show_help' to see available functions."
    test_connection
else
    echo "This script should be sourced, not executed directly."
    echo "Usage: source manual_commands.sh"
    exit 1
fi
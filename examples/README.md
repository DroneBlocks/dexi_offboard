# DEXI Offboard Manager Examples

This directory contains practical examples for using the DEXI Offboard Manager in different ways.

## Directory Structure

```
examples/
├── python/           # Python script examples
├── cli/             # Command line interface examples
├── node_red/        # Node-RED flow examples (coming soon)
└── README.md        # This file
```

## Python Examples

### `python/simple_flight.py`
Basic flight sequence demonstrating:
- Starting offboard heartbeat
- Arming and takeoff
- Holding position
- Landing and disarming

**Usage:**
```bash
# Ensure dexi_offboard node is running
ros2 launch dexi_offboard px4_offboard_manager.launch.py

# Run the example
cd examples/python
python3 simple_flight.py
```

### `python/square_pattern.py`
More advanced example showing:
- Complex flight patterns
- Multiple movement commands
- Rotation control
- Error handling

**Usage:**
```bash
cd examples/python
python3 square_pattern.py
```

## CLI Examples

### `cli/basic_flight.sh`
Bash script for simple flight operations with:
- Safety confirmations
- Error handling
- Progress feedback
- Emergency stop capability

**Usage:**
```bash
# Make executable
chmod +x examples/cli/basic_flight.sh

# Run the script
./examples/cli/basic_flight.sh
```

### `cli/manual_commands.sh`
Source this file to get convenient shell functions for manual control:

**Usage:**
```bash
# Source the command functions
source examples/cli/manual_commands.sh

# Use the functions interactively
quick_takeoff 3.0        # Takeoff to 3 meters
move_forward 5.0         # Move forward 5 meters
rotate_right 90          # Turn right 90 degrees
quick_land               # Land and shutdown

# Get help
show_help
```

**Available Functions:**
- `start_heartbeat` / `stop_heartbeat`
- `arm_drone` / `disarm_drone`
- `takeoff_drone [altitude]` / `land_drone`
- `move_forward/backward/left/right/up/down [distance]`
- `rotate_left/right [angle]`
- `quick_takeoff [altitude]` / `quick_land`
- `emergency_stop`

## Node-RED Examples (Coming Soon)

The `node_red/` directory will contain:
- Flow examples for drone control
- Dashboard interfaces
- Integration with other systems

## Safety Notes

⚠️ **Important:** Always follow these safety guidelines:

1. **Test in simulation first** - Use SITL before real hardware
2. **Check your environment** - Ensure adequate space and no obstacles
3. **Have emergency stop ready** - Keep manual RC control available
4. **Start small** - Begin with low altitudes and short distances
5. **Monitor the drone** - Watch for unexpected behavior

## Prerequisites

Before running any examples:

1. **ROS2 Environment:**
   ```bash
   source /opt/ros/humble/setup.bash  # Or your ROS2 installation
   source ~/ros2_ws/install/setup.bash  # Your workspace
   ```

2. **Required packages:**
   - `dexi_offboard`
   - `dexi_interfaces`
   - `px4_msgs`

3. **Running nodes:**
   ```bash
   # For SITL simulation
   ros2 launch dexi_offboard offboard_sitl.launch.py

   # OR for hardware
   ros2 launch dexi_offboard px4_offboard_manager.launch.py
   ```

## Troubleshooting

### Common Issues

1. **"Command not found" errors:**
   - Ensure ROS2 environment is sourced
   - Check that dexi_interfaces package is built

2. **No response from drone:**
   - Verify offboard manager node is running
   - Check topic connections: `ros2 topic list`
   - Ensure heartbeat is started first

3. **Python import errors:**
   - Verify dexi_interfaces is in Python path
   - Check ROS2 workspace is properly sourced

### Debug Commands

```bash
# Check if offboard manager is running
ros2 node list | grep offboard

# Monitor command topic
ros2 topic echo /dexi/offboard_manager

# Check vehicle status (if connected to PX4)
ros2 topic echo /fmu/out/vehicle_status
```

## Contributing

To add new examples:

1. Follow the existing code structure
2. Include comprehensive error handling
3. Add safety checks and confirmations
4. Document usage and prerequisites
5. Test thoroughly in simulation

## Support

For issues with examples:
- Check the main README.md troubleshooting section
- Submit issues via GitHub
- Contact: Dennis Baldwin (db@droneblocks.com)
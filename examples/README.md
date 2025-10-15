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
- Arming and takeoff using offboard mode
- Holding position
- Landing and disarming

**Key Feature:** Uses `offboard_takeoff` to maintain offboard mode throughout the flight,
ensuring the drone stays responsive to all commands.

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
- Complex flight patterns using offboard mode
- Multiple movement commands (fly_forward, fly_right, fly_backward, fly_left)
- Rotation control (360-degree spin)
- Error handling with emergency landing

**Key Feature:** Demonstrates continuous offboard flight with multiple waypoints.
Uses `offboard_takeoff` so all movement commands work without mode switching.

**Usage:**
```bash
cd examples/python
python3 square_pattern.py
```

### `python/circle_orbit.py`
Circular orbit pattern with configurable heading modes:
- Smooth circular flight path with 1m diameter (configurable)
- **Tangent mode**: Drone nose follows the flight path (default)
- **Center-facing mode**: Drone always faces toward the center of the circle
- Configurable number of orbits
- Real-time position tracking and trajectory updates

**Key Parameters (configurable in script):**
- `circle_diameter`: Size of the circle (default: 1.0m)
- `flight_speed`: Speed along the path (default: 0.3 m/s)
- `heading_mode`: 'tangent' or 'center' (default: 'tangent')
- `num_orbits`: Number of complete circles (default: 2)
- `takeoff_altitude`: Height for the pattern (default: 3.0m)

**Usage:**
```bash
cd examples/python
python3 circle_orbit.py
```

**How it works:**
The circle is generated using parametric equations:
- x(t) = r × cos(t)
- y(t) = r × sin(t)

Heading modes:
- **Tangent**: heading = atan2(dy/dt, dx/dt) - nose follows direction of travel
- **Center**: heading = atan2(-y, -x) - nose always points to circle center

To change heading mode, edit line 55 in the script:
```python
self.heading_mode = 'center'  # Change from 'tangent' to 'center'
```

**Note:** This script stops the offboard manager's heartbeat after takeoff and takes direct control
of trajectory setpoints to avoid conflicts. The script publishes setpoints at 20Hz during the pattern.

### `python/figure_eight_pattern.py`
Advanced trajectory control demonstrating:
- Smooth figure 8 (lemniscate) flight pattern
- Direct trajectory setpoint publishing
- Dynamic heading control (nose follows the path)
- Parametric curve generation with tangent calculations
- Real-time position tracking

This example directly publishes to PX4's trajectory setpoint topic for smooth,
continuous motion along a figure 8 path. The drone's heading is continuously
updated to stay tangent to the flight path.

**Parameters (configurable in script):**
- `takeoff_altitude`: Height for the pattern (default: 3.0m)
- `figure8_size`: Size of the pattern (default: 3.0m)
- `flight_speed`: Speed along the path (default: 0.5 m/s)
- `update_rate`: Trajectory update frequency (default: 20 Hz)

**Usage:**
```bash
cd examples/python
python3 figure_eight_pattern.py
```

**How it works:**
The figure 8 is generated using parametric equations:
- x(t) = A × sin(t)
- y(t) = (A/2) × sin(2t)

The heading is calculated from the tangent vector:
- heading = atan2(dy/dt, dx/dt)

This ensures the drone's nose always points in the direction of travel.

**Note:** This script stops the offboard manager's heartbeat after takeoff and takes direct control
of trajectory setpoints to avoid conflicts. The script publishes setpoints at 20Hz during the pattern.

## Important: Offboard Mode vs Auto Mode

When writing scripts for continuous offboard control, it's critical to understand the difference
between PX4 flight modes:

### Why Use `offboard_takeoff` Instead of `takeoff`?

**Problem with `takeoff` command:**
```
start_offboard_heartbeat → drone enters OFFBOARD mode
arm → drone arms
takeoff → drone switches to AUTO mode for takeoff
→ after takeoff completes, drone enters HOLD/LOITER mode
fly_forward → IGNORED (drone is in HOLD, not OFFBOARD) ✗
```

**Solution with `offboard_takeoff`:**
```
start_offboard_heartbeat → drone enters OFFBOARD mode
arm → drone arms
offboard_takeoff → drone stays in OFFBOARD mode, climbs via setpoints
fly_forward → WORKS (drone is in OFFBOARD) ✓
fly_right → WORKS (drone is in OFFBOARD) ✓
```

### Rule of Thumb
- **Use `takeoff`**: For simple arm → takeoff → land sequences without mid-flight navigation
- **Use `offboard_takeoff`**: For scripts with movement commands (fly_forward, fly_right, etc.)

All Python examples in this directory use `offboard_takeoff` for seamless offboard control.

### Advanced Pattern Scripts (circle_orbit.py, figure_eight_pattern.py)

These scripts use a special technique for smooth trajectory control:

1. **Initial phase**: Use offboard manager for arm, takeoff, and stabilization
2. **Pattern phase**: Stop offboard manager heartbeat and take direct control
3. **Why?** Prevents conflicts between manager's position-hold setpoints and script's trajectory setpoints
4. **Result**: Smooth, continuous motion along complex paths

**Key implementation details:**
- Scripts call `stop_offboard_heartbeat` before starting the pattern
- Use `rclpy.spin_once()` during waits to keep receiving position updates
- Publish both `OffboardControlMode` and `TrajectorySetpoint` at 20Hz
- Proper QoS settings (BEST_EFFORT, TRANSIENT_LOCAL) for PX4 communication

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

4. **QoS mismatch warnings:**
   - If you see "incompatible QoS" warnings, ensure your publisher uses:
     - Reliability: BEST_EFFORT
     - Durability: TRANSIENT_LOCAL
   - All example scripts have correct QoS settings

5. **Position data not received (circle/figure-8 patterns):**
   - Ensure PX4 SITL/hardware is running and connected
   - Check `/fmu/out/vehicle_local_position` topic is publishing:
     ```bash
     ros2 topic echo /fmu/out/vehicle_local_position --once
     ```
   - Verify MicroXRCEAgent is running (for SITL)

6. **Pattern doesn't execute, drone just yaws:**
   - This was a bug that's now fixed in the scripts
   - The scripts properly stop the offboard manager heartbeat before pattern execution

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
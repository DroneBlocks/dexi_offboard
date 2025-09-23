# DEXI Offboard Manager

A ROS2 package providing simplified PX4 offboard control for DEXI drones. This package offers both command-line interface and programmatic API access for autonomous flight operations.

## Overview

The DEXI Offboard Manager provides a high-level interface for controlling PX4-powered drones in offboard mode. It handles the complex PX4 communication protocols and exposes simple commands for:

- Arming/disarming
- Takeoff/landing
- Position control (relative movements)
- Heading control
- Offboard mode management

## Quick Start

### 1. Launch the Offboard Manager

```bash
# For hardware deployment
ros2 launch dexi_offboard px4_offboard_manager.launch.py

# For SITL simulation
ros2 launch dexi_offboard offboard_sitl.launch.py
```

### 2. Basic Flight Sequence

Once the node is running, you can control the drone using ROS2 topics:

```bash
# 1. Start offboard heartbeat (required before other commands)
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'start_offboard_heartbeat'}"

# 2. Arm the vehicle
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'arm'}"

# 3. Takeoff to 2 meters
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'takeoff', distance_or_degrees: 2.0}"

# 4. Land
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'land'}"

# 5. Disarm
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'disarm'}"

# 6. Stop offboard heartbeat
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'stop_offboard_heartbeat'}"
```

## Available Commands

### Flight Control
- `start_offboard_heartbeat` - Begin offboard mode heartbeat (required first)
- `stop_offboard_heartbeat` - Stop offboard mode heartbeat
- `arm` - Arm the vehicle
- `disarm` - Disarm the vehicle
- `takeoff` - Takeoff to specified altitude (meters)
- `land` - Land at current position

### Movement Commands
- `fly_forward` - Move forward by specified distance (meters)
- `fly_backward` - Move backward by specified distance (meters)
- `fly_left` - Move left by specified distance (meters)
- `fly_right` - Move right by specified distance (meters)
- `fly_up` - Move up by specified distance (meters)
- `fly_down` - Move down by specified distance (meters)

### Heading Control
- `yaw_left` - Rotate left by specified angle (degrees)
- `yaw_right` - Rotate right by specified angle (degrees)

## Command Line Examples

### Example Flight Pattern

```bash
# Start offboard mode
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'start_offboard_heartbeat'}"

# Arm and takeoff
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'arm'}"
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'takeoff', distance_or_degrees: 3.0}"

# Wait for takeoff to complete, then fly a square pattern
sleep 5
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'fly_forward', distance_or_degrees: 5.0}"
sleep 3
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'fly_right', distance_or_degrees: 5.0}"
sleep 3
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'fly_backward', distance_or_degrees: 5.0}"
sleep 3
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'fly_left', distance_or_degrees: 5.0}"

# Land and disarm
sleep 3
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'land'}"
sleep 5
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'disarm'}"
ros2 topic pub --once /dexi/offboard_manager dexi_interfaces/msg/OffboardNavCommand "{command: 'stop_offboard_heartbeat'}"
```

## Prerequisites

- ROS2 Humble or later
- PX4 autopilot (hardware or SITL)
- `dexi_interfaces` package
- `px4_msgs` package

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/DroneBlocks/dexi_offboard.git
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select dexi_offboard
```

4. Source the workspace:
```bash
source install/setup.bash
```

## 3D Visualization with RVIZ

The package includes a pose publisher that converts PX4 odometry to standard ROS2 visualization messages.

### Launch with Visualization
```bash
# Launch offboard manager with pose publisher
ros2 launch dexi_offboard offboard_with_visualization.launch.py

# In another terminal, start RVIZ
rviz2 -d $(ros2 pkg prefix dexi_offboard)/share/dexi_offboard/rviz/dexi_visualization.rviz
```

### What You'll See in RVIZ
- **Red arrow**: Current drone pose and orientation
- **TF frames**: Coordinate system visualization
- **Grid**: Reference coordinate plane

### Available Visualization Topics
- `/dexi/pose` - geometry_msgs/PoseStamped for current position
- `/tf` - Transform tree showing drone coordinate frames

## Usage with Python and Node-RED

See the `examples/` directory for:
- Python scripts for programmatic control
- Node-RED flow examples
- Integration examples with other ROS2 nodes

## Safety Notes

⚠️ **Important Safety Considerations:**

1. **Always test in simulation first** using the SITL launch file
2. **Ensure adequate space** for flight operations
3. **Keep manual override ready** - have a safety pilot with RC transmitter
4. **Start with low altitudes** and short distances
5. **Monitor battery levels** and flight time
6. **Check GPS lock** before outdoor flights

## Troubleshooting

### Common Issues

1. **"No heartbeat" errors**: Ensure `start_offboard_heartbeat` is called first
2. **Arm rejected**: Check vehicle status, GPS lock, and pre-arm checks
3. **Commands ignored**: Verify the vehicle is in offboard mode
4. **Connection issues**: Check PX4 SITL or hardware connection

### Debugging Commands

```bash
# Check vehicle status
ros2 topic echo /fmu/out/vehicle_status

# Monitor vehicle position
ros2 topic echo /fmu/out/vehicle_local_position

# Check offboard manager logs
ros2 node info /dexi/px4_offboard_manager
```

## Contributing

This package is part of the DEXI drone ecosystem. For issues and contributions:
- GitHub: https://github.com/DroneBlocks/dexi_offboard
- Issues: Submit via GitHub Issues

## License

[License information to be added]

## Maintainer

Dennis Baldwin (db@droneblocks.com)
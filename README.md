# Minimal MAVROS with Custom MAVLink Messages

A streamlined ROS2 MAVROS implementation with built-in custom MAVLink message handling. Supports bidirectional communication with autopilots using custom dialects.

## Features

- ✅ **Minimal MAVROS** - Only essential plugins included
- ✅ **Custom MAVLink messages** - Direct plugin integration for TOTA dialect
- ✅ **Configurable namespace** - Set via config.yaml
- ✅ **Bidirectional communication** - Receive sensor data, send commands
- ✅ **Built-in router** - No external routing needed

## Quick Start

### Prerequisites
- ROS2 Humble
- Python 3 with PyYAML

### Building
```bash
cd ~/ros2_mav_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select mavros
source install/setup.bash
```

### Configuration

Edit `config.yaml` to set your robot name and network ports:
```yaml
robot_name: "tota1"     # Robot namespace
mavlink:
  input_port: 14551     # Port to send commands TO autopilot
  output_port: 14555    # Port to receive data FROM autopilot
  host: "127.0.0.1"
```

### Running

Simply launch MAVROS:
```bash
cd ~/ros2_mav_ws
python3 launch_mavros.py
```

That's it! The built-in MAVROS router handles all communication.

## Network Setup

```
[Autopilot]
    ↕ UDP
[MAVROS Built-in Router]
    ├── Receives on port 14555
    └── Sends to port 14551
```

## Available Topics

### Receiving Data (from autopilot)
All topics are prefixed with `/{robot_name}/`

**Custom Messages:**
- `tof_ranges/raw` - 8x8 TOF distance array (meters)
- `tof_ranges/cloud` - PointCloud2 visualization
- `tof_meta/status` - Zone status values
- `tof_meta/confidence` - Zone confidence values
- `wheel_encoders/counts` - Encoder tick counts
- `wheel_encoders/velocities` - Wheel velocities (rad/s)
- `wheel_encoders/twist` - Robot twist

**Standard Telemetry:**
- `imu/data` - IMU with orientation
- `imu/data_raw` - Raw IMU data
- `battery` - Battery status
- `state` - System state
- `radio_status` - Radio telemetry

### Sending Commands (to autopilot)
- `velocity_control/cmd_vel` - Send Twist messages for velocity control

## Testing

### Check data flow:
```bash
# Monitor TOF sensor data
ros2 topic hz /tota1/tof_ranges/raw

# View system state
ros2 topic echo /tota1/state

# Send velocity commands
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/tota1/velocity_control/cmd_vel
```

### Monitor MAVLink traffic:
```bash
# Incoming messages
ros2 topic echo /tota1/mavlink_source

# Outgoing commands
ros2 topic echo /tota1/mavlink_sink
```

## Custom Messages Supported

| Message | ID | Description |
|---------|-----|-------------|
| TOF_L7CX_RANGES | 42001 | 8x8 TOF sensor array |
| TOF_L7CX_META | 42002 | TOF metadata |
| WHEEL_ENCODERS | 42003 | Differential drive encoders |

## Development

To add new functionality, see:
- [Development Guide](docs/dev_guide.md) - Add new message handlers and commands

## Troubleshooting

### No data received?
1. Check autopilot is sending to port 14555
2. Verify with: `nc -lu 14555`

### Commands not sent?
1. Check autopilot listens on port 14551
2. Monitor with: `ros2 topic echo /tota1/mavlink_sink`

### Stop all processes:
```bash
pkill -f mavros_node
```
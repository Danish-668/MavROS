# TOTA MAVROS - Custom Dialect Integration

A specialized ROS2 MAVROS implementation with integrated TOTA dialect support. Features proper typed message handlers and follows MAVROS conventions for custom MAVLink message handling.

## Features

- ✅ **TOTA Dialect Integration** - Native support for TOTA custom messages
- ✅ **Typed Message Handlers** - Proper MAVROS plugin architecture with auto-deserialization
- ✅ **Symlink Build Support** - Fast development with `colcon build --symlink-install`
- ✅ **Parallel Build Support** - Efficient builds with `--parallel-workers`
- ✅ **Built-in Custom Plugins** - TOF sensors, wheel encoders, velocity control
- ✅ **Standard MAVROS Compatibility** - Works with existing MAVROS ecosystem

## Quick Start

### Prerequisites
- ROS2 Humble
- Python 3 with PyYAML

### Building
```bash
cd ~/ros2_mav_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --parallel-workers 3
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

Launch MAVROS with TOTA dialect:
```bash
cd ~/ros2_mav_ws
source install/setup.bash
ros2 launch tota_mavros mavros.launch.py namespace:=tota1
```

The system will automatically load TOTA dialect and custom plugins.

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

**Custom TOTA Messages:**
- `tof_ranges/raw` - 8x8 TOF distance array (Int32MultiArray, mm)
- `tof_ranges/cloud` - PointCloud2 visualization
- `tof_meta/status` - Zone status values (UInt8MultiArray)
- `tof_meta/signal` - Signal strength values (UInt16MultiArray)
- `wheel_encoders/joint_states` - Joint positions and velocities
- `wheel_encoders/raw_ticks` - Raw encoder tick counts

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

## TOTA Dialect Implementation

This implementation uses proper TOTA dialect integration with typed message handlers, following MAVROS conventions:

### Custom Messages Supported

| Message | ID | Description | Plugin |
|---------|-----|-------------|--------|
| TOF_L7CX_RANGES | 42001 | 8x8 TOF sensor array | tof_ranges |
| TOF_L7CX_META | 42002 | TOF metadata (status/signal) | tof_meta |
| WHEEL_ENCODERS | 42003 | Differential drive encoders | wheel_encoders |

### Key Implementation Details

- **Dialect Integration**: TOTA dialect is the primary dialect (includes common messages)
- **Typed Handlers**: All plugins use `mavlink::tota_dialect::msg::MESSAGE_NAME` objects
- **Auto-Deserialization**: MAVLink messages automatically parsed to typed objects
- **Filter Support**: Uses `plugin::filter::SystemAndOk` for message filtering
- **MAVROS Conventions**: Follows standard MAVROS plugin architecture

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
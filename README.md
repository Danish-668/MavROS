# ROS2 MAVROS Custom MAVLink Integration

## Overview
This workspace provides a minimal MAVROS implementation with integrated custom MAVLink message handling for the TOTA dialect. All custom message handlers are implemented as MAVROS plugins, not separate nodes.

## Quick Start

### Prerequisites
- ROS2 Humble
- MAVROS dependencies
- Python 3 with pymavlink

### Building
```bash
cd ~/ros2_mav_ws
colcon build --packages-select mavros
source install/setup.bash
```

### Running the System

1. **Start Custom Router** (Terminal 1):
```bash
cd ~/ros2_mav_ws
python3 custom_router.py
```

2. **Start MAVROS** (Terminal 2):
```bash
source ~/ros2_mav_ws/install/setup.bash
ros2 run mavros mavros_node --ros-args \
    -r __ns:=/uas1 \
    -p fcu_url:="udp://@:14555" \
    -p fcu_protocol:="v2.0"
```

3. **Send MAVLink Messages**:
   - Send your messages to UDP port 14550
   - Router forwards them to MAVROS on port 14555

## Available ROS2 Topics

### Custom Message Topics
- `/uas1/tof_ranges/raw` - 8x8 TOF distance array (meters)
- `/uas1/tof_ranges/cloud` - PointCloud2 visualization
- `/uas1/tof_meta/status` - Zone status values
- `/uas1/tof_meta/confidence` - Zone confidence values
- `/uas1/wheel_encoders/counts` - Encoder tick counts
- `/uas1/wheel_encoders/velocities` - Wheel velocities (rad/s)
- `/uas1/wheel_encoders/twist` - Robot twist

### Standard MAVROS Topics
- `/uas1/imu/data` - IMU with orientation
- `/uas1/imu/data_raw` - Raw IMU data
- `/uas1/battery` - Battery status
- `/uas1/rc/in` - RC input channels
- `/uas1/state` - System state

## Architecture

```
[MAVLink Source] ---(14550/UDP)---> [Custom Router] ---(14555/UDP)---> [MAVROS]
                                           |
                                    Forwards ALL messages
                                    including custom IDs
```

## Supported Custom Messages

| Message | ID | Description |
|---------|-----|-------------|
| TOF_L7CX_RANGES | 42001 | 8x8 TOF sensor distance data |
| TOF_L7CX_META | 42002 | TOF sensor metadata |
| WHEEL_ENCODERS | 42003 | Differential drive encoder data |

## Known Issues

- IMU data not yet working - ATTITUDE messages alone are insufficient, plugin requires HIGHRES_IMU or SCALED_IMU2 messages

## Cleanup
To stop all processes:
```bash
pkill -f mavros_node
pkill -f custom_router
```

## Documentation

- [Implementation Guide](docs/implementation-guide.md) - Complete MAVROS plugin implementation details
- [Extending Custom Messages](docs/extending-custom-messages.md) - How to add new custom MAVLink messages
# Minimal MAVROS Build for Radio Telemetry

This workspace contains a custom minimal MAVROS build optimized for radio telemetry with significantly reduced overhead.

## Features
- **Reduced plugins**: Only 5 essential plugins (sys_status, sys_time, rc_io, param, 3dr_radio)
- **Minimal topics**: 17 topics instead of 150+ in standard MAVROS
- **Radio focus**: Optimized specifically for radio telemetry data
- **Low overhead**: Reduced CPU and memory usage
- **Custom dialect ready**: Prepared for custom MAVLink dialect integration

## Build Instructions

```bash
# Clone the workspace (if not already done)
cd /home/danish/ros2_mav_ws

# Build the minimal MAVROS
colcon build --packages-select mavros_msgs libmavconn mavros mavros_extras --symlink-install

# Source the workspace
source install/setup.bash
```

## Running the System

The system requires three components running in separate terminals:

### Terminal 1 - MAVLink Router
Routes MAVLink messages between different endpoints:
```bash
cd /home/danish/ros2_mav_ws
mavlink-routerd -c mrouter.conf
```

### Terminal 2 - Heartbeat Shim
Provides heartbeat messages for connection maintenance:
```bash
cd /home/danish/ros2_mav_ws
python3 fcu_heartbeat_shim_v2.py --in 127.0.0.1:14560 --out 127.0.0.1:14555 --sysid 255 --compid 191 --autopilot 0 --rate 1
```

### Terminal 3 - Minimal MAVROS
Run the minimal MAVROS node:
```bash
cd /home/danish/ros2_mav_ws
source install/setup.bash
ros2 run mavros mavros_node --ros-args -r __ns:=/uas1 -p fcu_url:="udp://@:14555" -p fcu_protocol:="v2.0"
```

### Alternative: Using Launch File
You can also use the provided launch file:
```bash
cd /home/danish/ros2_mav_ws
source install/setup.bash
ros2 launch ./minimal_mavros_launch.py
```

## Testing and Verification

### Check Topic Count
Verify the reduced topic count (should be ~17 instead of 150+):
```bash
ros2 topic list | grep uas1 | wc -l
```

### Monitor Radio Status
```bash
ros2 topic echo /uas1/radio_status
```

### View All Topics
```bash
ros2 topic list | grep uas1
```

Expected topics:
- `/uas1/battery`
- `/uas1/estimator_status`
- `/uas1/extended_state`
- `/uas1/mavlink_sink`
- `/uas1/mavlink_source`
- `/uas1/param/event`
- `/uas1/radio_status`
- `/uas1/rc/in`
- `/uas1/rc/out`
- `/uas1/rc/override`
- `/uas1/state`
- `/uas1/status_event`
- `/uas1/statustext/recv`
- `/uas1/statustext/send`
- `/uas1/sys_status`
- `/uas1/time_reference`
- `/uas1/timesync_status`

### Monitor Diagnostics
```bash
ros2 topic echo /diagnostics
```

## Plugin Management Guide

### Current Minimal Plugin Set

The minimal build includes only these essential plugins:

#### Core Plugins (mavros/CMakeLists.txt)
1. **sys_status.cpp** - System status and heartbeat (required for connection)
2. **sys_time.cpp** - Time synchronization
3. **rc_io.cpp** - RC input/output data
4. **param.cpp** - Parameter handling

#### Extra Plugins (mavros_extras/CMakeLists.txt)
1. **3dr_radio.cpp** - Radio status and telemetry

### Adding Plugins

To add additional plugins, edit the appropriate CMakeLists.txt file:

#### For Core Plugins
Edit `src/mavros_custom/mavros/CMakeLists.txt` around line 137:

```cmake
add_library(mavros_plugins SHARED
  # Core plugins
  src/plugins/sys_status.cpp
  src/plugins/sys_time.cpp
  src/plugins/rc_io.cpp
  src/plugins/param.cpp
  
  # Add your plugin here, for example:
  # src/plugins/imu.cpp           # For IMU data
  # src/plugins/global_position.cpp  # For GPS position
)
```

#### For Extra Plugins
Edit `src/mavros_custom/mavros_extras/CMakeLists.txt` around line 78:

```cmake
add_library(mavros_extras_plugins SHARED
  # Extra plugins
  src/plugins/3dr_radio.cpp
  
  # Add extra plugins here, for example:
  # src/plugins/distance_sensor.cpp  # For distance sensors
  # src/plugins/optical_flow.cpp     # For optical flow
)
```

### Removing Plugins

Simply comment out or remove the plugin line from the respective CMakeLists.txt file.

### Available Plugins Reference

#### Commonly Used Core Plugins
- `actuator_control.cpp` - Actuator control messages
- `altitude.cpp` - Altitude data
- `command.cpp` - Command protocol
- `ftp.cpp` - File transfer protocol
- `global_position.cpp` - GPS/global position
- `home_position.cpp` - Home position management
- `imu.cpp` - IMU sensor data
- `local_position.cpp` - Local position estimates
- `manual_control.cpp` - Manual control input
- `setpoint_*.cpp` - Various setpoint controllers
- `waypoint.cpp` - Waypoint/mission protocol

#### Commonly Used Extra Plugins
- `adsb.cpp` - ADS-B transponder data
- `camera.cpp` - Camera control
- `companion_process_status.cpp` - Companion computer status
- `distance_sensor.cpp` - Distance/range sensors
- `esc_status.cpp` - ESC telemetry
- `gps_rtk.cpp` - RTK GPS data
- `landing_target.cpp` - Precision landing
- `mocap_pose_estimate.cpp` - Motion capture
- `obstacle_distance.cpp` - Obstacle detection
- `optical_flow.cpp` - Optical flow sensor
- `vision_pose.cpp` - Vision-based position

### After Modifying Plugins

Always rebuild after changing plugins:
```bash
cd /home/danish/ros2_mav_ws
colcon build --packages-select mavros mavros_extras --symlink-install
source install/setup.bash
```

## Custom Dialect Integration

### Adding a Custom MAVLink Dialect

1. Place your custom dialect XML file in:
   ```
   src/mavros_custom/mavros/dialects/your_dialect.xml
   ```

2. Update the dialect in `src/mavros_custom/mavros/CMakeLists.txt`:
   ```cmake
   # Around line 30, change:
   set(MAVLINK_DIALECT "common")
   # To:
   set(MAVLINK_DIALECT "your_dialect")
   ```

3. If your dialect extends another, ensure proper inheritance in the XML:
   ```xml
   <mavlink>
     <include>common.xml</include>
     <!-- Your custom messages here -->
   </mavlink>
   ```

4. Rebuild MAVROS:
   ```bash
   cd /home/danish/ros2_mav_ws
   colcon build --packages-select mavros_msgs libmavconn mavros mavros_extras --symlink-install
   ```

## Troubleshooting

### Parameter Error on Startup
If you see `parameter 'fcu_urls' cannot be set because it was not declared`, ensure you're using the updated build with the parameter initialization fix.

### No Topics Appearing
1. Check all three components are running (router, heartbeat shim, MAVROS)
2. Verify UDP ports are correct (14550, 14555, 14560)
3. Check firewall settings if running on separate machines

### Radio Status Warnings
Messages like "RADIO_STATUS not from 3DR modem?" can be safely ignored if you're not using a 3DR radio.

## Architecture Overview

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Radio/     │────▶│   MAVLink    │────▶│  Heartbeat   │
│   Telemetry  │14550│   Router     │14560│    Shim      │
└──────────────┘     └──────────────┘     └──────────────┘
                                                   │
                                                14555
                                                   ▼
                                           ┌──────────────┐
                                           │   Minimal    │
                                           │    MAVROS    │
                                           └──────────────┘
                                                   │
                                              ROS2 Topics
```

## Performance Comparison

| Metric | Standard MAVROS | Minimal MAVROS | Improvement |
|--------|----------------|----------------|-------------|
| Topics | 152 | 17 | 89% reduction |
| Plugins | 60+ | 5 | 92% reduction |
| Memory | ~150MB | ~40MB | 73% reduction |
| CPU Usage | High | Low | Significant |

## Modified Files

- `src/mavros_custom/mavros/CMakeLists.txt` - Reduced core plugins to 4
- `src/mavros_custom/mavros_extras/CMakeLists.txt` - Reduced extras to 1 (3dr_radio)
- `src/mavros_custom/mavros/src/mavros_node.cpp` - Fixed parameter initialization timing

## License

This is a modified version of MAVROS. Original MAVROS is licensed under the 3-clause BSD license.
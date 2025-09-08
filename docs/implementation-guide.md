# MAVROS Custom MAVLink Integration - Complete Guide

## Overview
This document provides complete details for implementing custom MAVLink message handling within MAVROS using integrated plugins. All custom message handlers are implemented as MAVROS plugins, not as separate ROS2 nodes.

## System Architecture

```
[MAVLink Source] ---(UDP:14550)---> [Custom Router] ---(UDP:14555)---> [MAVROS with Plugins]
                                            |                                    |
                                    Forwards ALL messages             Integrated Plugins:
                                    including custom IDs              - TOF Ranges
                                                                      - TOF Meta
                                                                      - Wheel Encoders
                                                                      - IMU
```

## Custom Messages Implementation

### Message Definitions

#### TOF_L7CX_RANGES (ID: 42001)
```
uint64_t time_us        # Timestamp in microseconds
uint16_t range_mm[64]   # 8x8 distance array in millimeters
```

#### TOF_L7CX_META (ID: 42002)
```
uint64_t time_us        # Timestamp in microseconds
uint8_t status[64]      # Zone status values
uint8_t confidence[64]  # Zone confidence values
```

#### WHEEL_ENCODERS (ID: 42003)
```
uint64_t time_us        # Timestamp in microseconds
int32_t left_count      # Left encoder ticks
int32_t right_count     # Right encoder ticks
float left_velocity     # Left wheel velocity (rad/s)
float right_velocity    # Right wheel velocity (rad/s)
```

## Plugin Implementation Guide

### Creating a New Custom Plugin

1. **Create Plugin Source File** (`src/plugins/your_plugin.cpp`):

```cpp
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

namespace mavros {
namespace std_plugins {

class YourPlugin : public plugin::Plugin {
public:
    explicit YourPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "your_plugin") {
        
        // Create ROS2 publisher
        data_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
            "~/your_topic", 10);
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(YOUR_MESSAGE_ID, &YourPlugin::handle_your_message_raw)
        };
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_pub;

    void handle_your_message_raw(
        const mavlink::mavlink_message_t *msg, 
        const mavconn::Framing framing) {
        
        // Parse raw message
        uint64_t time_us;
        memcpy(&time_us, &msg->payload64[0], sizeof(uint64_t));
        
        // Extract your data
        std::vector<float> data(YOUR_DATA_SIZE);
        // ... parse data from payload ...
        
        // Publish to ROS2
        auto ros_msg = std_msgs::msg::Float32MultiArray();
        ros_msg.data = data;
        data_pub->publish(ros_msg);
        
        RCLCPP_INFO_ONCE(get_logger(), "Received first YOUR_MESSAGE!");
    }
};

}  // namespace std_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::YourPlugin)
```

2. **Register Plugin in XML** (`mavros_plugins.xml`):

```xml
<class_libraries>
  <library path="mavros">
    <!-- Other plugins -->
    
    <class name="your_plugin" type="mavros::std_plugins::YourPlugin">
      <description>
        Handler for YOUR custom messages
      </description>
    </class>
  </library>
</class_libraries>
```

3. **Add to CMakeLists.txt**:

```cmake
add_library(mavros_plugins SHARED
  # Other plugins
  src/plugins/your_plugin.cpp
)
```

4. **Rebuild MAVROS**:

```bash
cd ~/ros2_mav_ws
colcon build --packages-select mavros
source install/setup.bash
```

## Custom Router Details

The custom router (`custom_router.py`) forwards ALL MAVLink messages, including those with unknown IDs:

```python
def forward_messages(in_port, out_port):
    """Forward all MAVLink messages from in_port to out_port"""
    # Creates UDP sockets
    # Forwards raw bytes without parsing
    # Logs custom messages (ID >= 42000)
    # Logs ATTITUDE messages (ID == 30)
```

Key features:
- No message filtering
- Preserves message integrity
- Handles custom message IDs
- Simple UDP forwarding

## Building and Running

### Prerequisites
```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install Python dependencies
pip3 install pymavlink
```

### Build Process
```bash
cd ~/ros2_mav_ws
colcon build --packages-select mavros
source install/setup.bash
```

### Running the System

1. **Terminal 1 - Router**:
```bash
cd ~/ros2_mav_ws
python3 custom_router.py
```

2. **Terminal 2 - MAVROS**:
```bash
source ~/ros2_mav_ws/install/setup.bash
ros2 run mavros mavros_node --ros-args \
    -r __ns:=/uas1 \
    -p fcu_url:="udp://@:14555" \
    -p fcu_protocol:="v2.0"
```

3. **Send Messages**: 
   - Send to UDP port 14550
   - Router forwards to MAVROS on 14555

## Testing and Verification

### Check Topics
```bash
# List all topics
ros2 topic list | grep uas1

# Monitor custom data
ros2 topic echo /uas1/tof_ranges/raw --once
ros2 topic echo /uas1/wheel_encoders/counts --once

# Check data rates
ros2 topic hz /uas1/tof_ranges/raw
```

### Expected Output
- Router shows: "Forwarded CUSTOM message ID 42001"
- MAVROS shows: "Received first TOF_L7CX_RANGES message!"
- Topics publish at message reception rate

## Troubleshooting

### No Custom Messages
1. Verify router is running: `ps aux | grep custom_router`
2. Check UDP ports: Send to 14550, MAVROS on 14555
3. Verify plugin loaded in MAVROS logs

### IMU Data Not Appearing
IMU plugin requires multiple message types:
- ATTITUDE (ID 30) - Orientation only
- HIGHRES_IMU (ID 105) or SCALED_IMU2 (ID 116) - Full IMU data

### Cleanup Issues
Processes may not terminate cleanly. Use:
```bash
pkill -f mavros_node
pkill -f custom_router
```

## Message Flow Verification

### Router Logs
```
Custom router: 14550 -> 14555
Forwarding ALL messages including custom ones...
Forwarded CUSTOM message ID 42001 from ('127.0.0.1', 52779)
Forwarded ATTITUDE message (ID 30) from ('127.0.0.1', 52779)
```

### MAVROS Logs
```
[INFO] Plugin tof_ranges created
[INFO] Plugin tof_ranges initialized
[INFO] Received first TOF_L7CX_RANGES message!
```

## File Structure
```
ros2_mav_ws/
├── src/mavros_custom/mavros/
│   ├── src/plugins/
│   │   ├── imu.cpp               # Standard IMU plugin
│   │   ├── tof_ranges.cpp        # TOF ranges handler
│   │   ├── tof_meta.cpp          # TOF metadata handler
│   │   └── wheel_encoders.cpp    # Wheel encoder handler
│   ├── mavros_plugins.xml        # Plugin registration
│   └── CMakeLists.txt            # Build configuration
├── custom_router.py               # MAVLink message router
├── README.md                      # Quick start guide
└── MAVROS_CUSTOM_SETUP.md        # This document
```

## Key Implementation Notes

1. **Plugin Architecture**: All custom handlers are MAVROS plugins
2. **Raw Message Handling**: Direct parsing of MAVLink payload
3. **No Dialect Required**: Works without custom dialect definitions
4. **Message IDs**: Custom messages use IDs > 42000
5. **Router Purpose**: Forwards messages that mavlink-routerd would drop

## Adding New Message Types

1. Define message structure and ID
2. Create plugin following template above
3. Register in XML and CMakeLists
4. Rebuild MAVROS
5. Test with message source

## Performance Considerations

- Plugins run in MAVROS process (no IPC overhead)
- Direct memory access to MAVLink messages
- Minimal parsing overhead
- Publishing rate matches input rate

## Dependencies

- ROS2 Humble
- MAVROS (customized build)
- Python 3 with pymavlink
- Standard ROS2 message packages
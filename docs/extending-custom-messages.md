# Custom MAVLink Dialect Implementation Notes

## Overview
This document explains how to extend the MAVROS system with additional custom MAVLink messages beyond the initial TOF and wheel encoder implementations.

## Adding New Custom Messages - Step by Step

### 1. Define Your Message Structure
First, determine your message ID (use >42000 for custom messages) and payload structure:

```
Example: CUSTOM_SENSOR (ID: 42004)
- uint64_t timestamp_us
- float sensor_value
- uint8_t sensor_id
- uint8_t status
```

### 2. Create the Plugin File

Create a new file in `src/mavros_custom/mavros/src/plugins/`:

```cpp
// custom_sensor.cpp
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include <std_msgs/msg/float32.hpp>

namespace mavros {
namespace std_plugins {

class CustomSensorPlugin : public plugin::Plugin {
public:
    explicit CustomSensorPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "custom_sensor") {
        
        sensor_pub = node->create_publisher<std_msgs::msg::Float32>(
            "~/value", 10);
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(42004, &CustomSensorPlugin::handle_custom_sensor_raw)
        };
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sensor_pub;

    void handle_custom_sensor_raw(
        const mavlink::mavlink_message_t *msg, 
        const mavconn::Framing framing) {
        
        // Parse the raw payload
        uint64_t timestamp_us;
        float sensor_value;
        uint8_t sensor_id, status;
        
        memcpy(&timestamp_us, &msg->payload64[0], sizeof(uint64_t));
        memcpy(&sensor_value, msg->payload + 8, sizeof(float));
        sensor_id = msg->payload[12];
        status = msg->payload[13];
        
        // Publish to ROS2
        auto ros_msg = std_msgs::msg::Float32();
        ros_msg.data = sensor_value;
        sensor_pub->publish(ros_msg);
        
        RCLCPP_INFO_ONCE(get_logger(), 
            "Received first CUSTOM_SENSOR message!");
    }
};

}  // namespace std_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::CustomSensorPlugin)
```

### 3. Register the Plugin

Add to `src/mavros_custom/mavros/mavros_plugins.xml`:

```xml
<class name="custom_sensor" type="mavros::std_plugins::CustomSensorPlugin">
  <description>
    Handler for CUSTOM_SENSOR messages
  </description>
</class>
```

### 4. Update Build Configuration

Add to `src/mavros_custom/mavros/CMakeLists.txt`:

```cmake
add_library(mavros_plugins SHARED
  # ... existing plugins ...
  src/plugins/custom_sensor.cpp
)
```

### 5. Rebuild and Test

```bash
cd ~/ros2_mav_ws
colcon build --packages-select mavros
source install/setup.bash
```

## Payload Parsing Guidelines

### Data Type Alignment
- Always respect data alignment when parsing
- Use memcpy for safe extraction of multi-byte values
- MAVLink uses little-endian byte order

### Common Patterns

**Fixed Arrays:**
```cpp
// For array[64] at offset 8
uint16_t array[64];
memcpy(array, msg->payload + 8, sizeof(array));
```

**Mixed Types:**
```cpp
// Example: timestamp(8) + float(4) + uint16(2) + uint8(1)
uint64_t timestamp;
float value;
uint16_t count;
uint8_t status;

memcpy(&timestamp, &msg->payload64[0], 8);
memcpy(&value, msg->payload + 8, 4);
memcpy(&count, msg->payload + 12, 2);
status = msg->payload[14];
```

## Message ID Allocation

Recommended ID ranges for custom messages:
- 42001-42099: Sensor data
- 42100-42199: Actuator commands
- 42200-42299: System status
- 42300-42399: Configuration

Current allocations:
- 42001: TOF_L7CX_RANGES
- 42002: TOF_L7CX_META
- 42003: WHEEL_ENCODERS

## Testing Custom Messages

### Quick Test with Python
```python
from pymavlink import mavutil
import struct
import time

# Connect to router input
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Create custom message payload
timestamp = int(time.time() * 1e6)
sensor_value = 123.45
sensor_id = 1
status = 0

payload = struct.pack('<Qfbb', timestamp, sensor_value, sensor_id, status)

# Send as MAVLink v2 message
mav.mav.send(mavutil.mavlink.MAVLink_message(
    msgid=42004,
    payload=payload
))
```

### Verify Reception
```bash
# Check if topic exists
ros2 topic list | grep custom_sensor

# Monitor data
ros2 topic echo /uas1/custom_sensor/value
```

## Troubleshooting

### Message Not Received
1. Verify message ID in plugin matches sender
2. Check router is forwarding (should log custom messages)
3. Verify plugin loaded: `ros2 node info /uas1/mavros_node`
4. Check MAVROS logs for plugin initialization

### Data Corruption
1. Verify payload structure matches between sender and receiver
2. Check byte order (little-endian)
3. Ensure proper data alignment
4. Use memcpy for multi-byte values

### Performance Issues
1. Minimize processing in message handler
2. Use RCLCPP_INFO_ONCE for one-time logs
3. Consider message throttling if high frequency
4. Profile with `ros2 topic hz`

## Best Practices

1. **Naming Convention**: Use descriptive plugin names matching message purpose
2. **Topic Structure**: Follow pattern `~/message_type/data_field`
3. **Error Handling**: Validate data before publishing
4. **Documentation**: Comment unusual payload structures
5. **Testing**: Always test with actual message source before deployment

## Integration with Existing Systems

### Coordinate with Standard Messages
If your custom message relates to standard MAVLink messages:
- Ensure consistent units (SI preferred)
- Match timestamp formats
- Use similar topic naming patterns

### ROS2 Message Types
Choose appropriate ROS2 message types:
- Sensor data: sensor_msgs types when applicable
- Primitives: std_msgs for simple values
- Custom: Create custom messages only when necessary

## Router Considerations

The custom router (`custom_router.py`) forwards all messages without filtering. For production:
- Consider adding rate limiting for specific message IDs
- Implement connection retry logic
- Add configurable ports via command line args
- Consider using mavlink-routerd with custom patches for production

## Future Enhancements

Potential improvements to consider:
1. Dynamic plugin loading without rebuild
2. Configuration file for message ID mappings
3. Automatic ROS2 message generation from MAVLink XML
4. Built-in message validation and checksums
5. Performance monitoring and statistics
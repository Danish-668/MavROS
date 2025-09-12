# MAVROS Development Guide

Complete guide for adding new functionality to the minimal MAVROS system - receiving data from autopilot and sending commands.

## Table of Contents
- [Receiving Data from Autopilot](#receiving-data-from-autopilot)
- [Sending Commands to Autopilot](#sending-commands-to-autopilot)
- [Common Examples](#common-examples)
- [Troubleshooting](#troubleshooting)

---

## 📥 Receiving Data from Autopilot

### Step 1: Update MAVLink Dialect (if using custom messages)

If your message isn't in the common MAVLink dialect:

1. **Locate dialect directory**:
```bash
src/mavros_custom/mavros/libmavconn/cmake/Modules/mavlink/message_definitions/v1.0/
```

2. **Add message to dialect XML** (e.g., `tota.xml`):
```xml
<message id="42004" name="MY_SENSOR_DATA">
  <description>Custom sensor data from autopilot</description>
  <field type="uint32_t" name="timestamp">Timestamp in ms</field>
  <field type="float[16]" name="values">Sensor readings</field>
  <field type="uint8_t" name="status">Status flags</field>
</message>
```

3. **Rebuild MAVLink headers**:
```bash
colcon build --packages-select mavros
```

### Step 2: Create MAVROS Plugin

1. **Create plugin file**:
```bash
touch src/mavros_custom/mavros/src/plugins/my_sensor.cpp
```

2. **Plugin template for receiving data**:
```cpp
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace mavros {
namespace extra_plugins {

class MySensorPlugin : public plugin::Plugin {
public:
  explicit MySensorPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "my_sensor")  // This becomes the topic prefix
  {
    // Create publishers for your ROS2 topics
    raw_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "~/raw", 10);
    cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/cloud", 10);
    
    RCLCPP_INFO(get_logger(), 
      "My Sensor plugin initialized - waiting for MY_SENSOR_DATA messages (ID 42004)");
  }

  Subscriptions get_subscriptions() override
  {
    // Register handler for your MAVLink message ID
    return {
      make_handler(&MySensorPlugin::handle_my_sensor_data),
    };
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raw_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
  
  void handle_my_sensor_data(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    const mavlink::tota::msg::MY_SENSOR_DATA & sensor_data,  // Your dialect namespace
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // Log first message received
    static bool first_msg = true;
    if (first_msg) {
      RCLCPP_INFO(get_logger(), "Received first MY_SENSOR_DATA message!");
      first_msg = false;
    }
    
    // Convert to ROS2 Float32MultiArray
    auto raw_msg = std_msgs::msg::Float32MultiArray();
    raw_msg.data.resize(16);
    for (int i = 0; i < 16; i++) {
      raw_msg.data[i] = sensor_data.values[i];
    }
    raw_pub->publish(raw_msg);
    
    // Convert to PointCloud2 for visualization
    auto cloud_msg = sensor_msgs::msg::PointCloud2();
    cloud_msg.header.stamp = node->now();
    cloud_msg.header.frame_id = "sensor_frame";
    cloud_msg.height = 4;
    cloud_msg.width = 4;
    // ... populate point cloud fields ...
    cloud_pub->publish(cloud_msg);
  }
};

}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::MySensorPlugin)
```

### Step 3: Register Plugin

1. **Add to CMakeLists.txt**:
```cmake
# Find the add_library(mavros_plugins section and add your file:
add_library(mavros_plugins SHARED
  # ... existing plugins ...
  src/plugins/my_sensor.cpp  # Add this line
)
```

2. **Add to mavros_plugins.xml** (before `</library>`):
```xml
<class name="my_sensor" type="mavros::plugin::PluginFactoryTemplate&lt;mavros::extra_plugins::MySensorPlugin&gt;" base_class_type="mavros::plugin::PluginFactory">
  <description>My custom sensor data handler
@plugin my_sensor

Handles MY_SENSOR_DATA custom messages (ID: 42004)</description>
</class>
```

### Step 4: Build and Test

```bash
# Build
colcon build --packages-select mavros

# Launch
python3 launch_mavros.py

# Verify plugin loaded
# Should see: "My Sensor plugin initialized - waiting for MY_SENSOR_DATA messages"

# Check topics
ros2 topic list | grep my_sensor
# Should show:
# /tota1/my_sensor/raw
# /tota1/my_sensor/cloud

# Monitor data
ros2 topic echo /tota1/my_sensor/raw
```

---

## 📤 Sending Commands to Autopilot

### Step 1: Create Command Plugin

1. **Create plugin file**:
```bash
touch src/mavros_custom/mavros/src/plugins/my_commander.cpp
```

2. **Plugin template for sending commands**:
```cpp
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

namespace mavros {
namespace std_plugins {

class MyCommanderPlugin : public plugin::Plugin {
public:
  explicit MyCommanderPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "my_commander")
  {
    // Subscribe to ROS2 topics for commands
    position_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/set_position", 10,
      std::bind(&MyCommanderPlugin::position_cb, this, std::placeholders::_1));
    
    throttle_sub = node->create_subscription<std_msgs::msg::Float32>(
      "~/set_throttle", 10,
      std::bind(&MyCommanderPlugin::throttle_cb, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "My Commander plugin initialized");
  }

  Subscriptions get_subscriptions() override
  {
    return {};  // Not receiving MAVLink, only sending
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub;

  void position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Send position target
    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED pos_cmd{};
    
    pos_cmd.time_boot_ms = get_time_boot_ms();
    pos_cmd.target_system = uas->get_tgt_system();
    pos_cmd.target_component = uas->get_tgt_component();
    pos_cmd.coordinate_frame = static_cast<uint8_t>(
      mavlink::common::MAV_FRAME::LOCAL_NED);
    
    // Set position
    pos_cmd.x = msg->pose.position.x;
    pos_cmd.y = msg->pose.position.y;
    pos_cmd.z = msg->pose.position.z;
    
    // Type mask: use position only (0 = use, 1 = ignore)
    pos_cmd.type_mask = 
      0b0000111111111000;  // Use xyz position, ignore rest
    
    uas->send_message(pos_cmd);
    
    RCLCPP_INFO(get_logger(), 
      "Sent position: x=%.2f, y=%.2f, z=%.2f",
      pos_cmd.x, pos_cmd.y, pos_cmd.z);
  }
  
  void throttle_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Send manual control with throttle
    mavlink::common::msg::MANUAL_CONTROL manual{};
    
    manual.target = uas->get_tgt_system();
    manual.x = 0;     // Pitch (none)
    manual.y = 0;     // Roll (none)
    manual.z = msg->data * 1000;  // Throttle (0-1000)
    manual.r = 0;     // Yaw (none)
    manual.buttons = 0;
    
    uas->send_message(manual);
    
    RCLCPP_INFO(get_logger(), "Sent throttle: %.2f", msg->data);
  }
  
  uint32_t get_time_boot_ms()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
};

}  // namespace std_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::MyCommanderPlugin)
```

### Step 2: Sending Custom MAVLink Messages

For custom messages not in common dialect:

```cpp
void send_custom_command(float param1, int32_t param2)
{
  // Method 1: Using dialect message class
  mavlink::tota::msg::MY_CUSTOM_COMMAND cmd{};
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.timestamp = get_time_boot_ms();
  
  uas->send_message(cmd);
  
  // Method 2: Raw message construction
  mavlink_message_t raw_msg;
  mavlink_msg_my_custom_command_pack(
    uas->get_mav_id(),      // System ID
    uas->get_mav_comp_id(),  // Component ID
    &raw_msg,
    param1, param2, get_time_boot_ms()
  );
  
  uas->send_message(raw_msg);
}
```

### Step 3: Register and Build

Same as receiving - add to CMakeLists.txt and mavros_plugins.xml, then rebuild.

---

## Common Examples

### Example: GPS Input
```cpp
// Send GPS data to autopilot
void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  mavlink::common::msg::GPS_INPUT gps{};
  
  gps.time_usec = node->now().nanoseconds() / 1000;
  gps.gps_id = 0;
  gps.lat = msg->latitude * 1e7;   // Convert to degE7
  gps.lon = msg->longitude * 1e7;
  gps.alt = msg->altitude;
  gps.hdop = 1.0;
  gps.vdop = 1.0;
  gps.vn = 0;  // North velocity
  gps.ve = 0;  // East velocity
  gps.vd = 0;  // Down velocity
  gps.speed_accuracy = 0.3;
  gps.horiz_accuracy = 1.0;
  gps.vert_accuracy = 1.5;
  gps.satellites_visible = 10;
  gps.fix_type = 3;  // 3D fix
  
  uas->send_message(gps);
}
```

### Example: Arming/Disarming
```cpp
void arm_vehicle(bool arm)
{
  mavlink::common::msg::COMMAND_LONG cmd{};
  
  cmd.target_system = uas->get_tgt_system();
  cmd.target_component = uas->get_tgt_component();
  cmd.command = static_cast<uint16_t>(
    mavlink::common::MAV_CMD::COMPONENT_ARM_DISARM);
  cmd.param1 = arm ? 1.0 : 0.0;  // 1 to arm, 0 to disarm
  cmd.param2 = 0;  // Force (0 = normal)
  
  uas->send_message(cmd);
  
  RCLCPP_INFO(get_logger(), "%s vehicle", arm ? "Arming" : "Disarming");
}
```

### Example: Set Mode
```cpp
void set_mode(const std::string& mode_name)
{
  mavlink::common::msg::COMMAND_LONG cmd{};
  
  cmd.target_system = uas->get_tgt_system();
  cmd.target_component = uas->get_tgt_component();
  cmd.command = static_cast<uint16_t>(mavlink::common::MAV_CMD::DO_SET_MODE);
  
  // Mode mapping (example for ArduPilot Rover)
  uint8_t custom_mode = 0;
  if (mode_name == "MANUAL") custom_mode = 0;
  else if (mode_name == "GUIDED") custom_mode = 4;
  else if (mode_name == "AUTO") custom_mode = 10;
  
  cmd.param1 = 1;  // Mode: 1 = custom mode
  cmd.param2 = custom_mode;
  
  uas->send_message(cmd);
}
```

### Example: Request Data Stream
```cpp
void request_data_stream(uint8_t stream_id, uint16_t rate_hz)
{
  mavlink::common::msg::REQUEST_DATA_STREAM req{};
  
  req.target_system = uas->get_tgt_system();
  req.target_component = uas->get_tgt_component();
  req.req_stream_id = stream_id;
  req.req_message_rate = rate_hz;
  req.start_stop = 1;  // 1 = start, 0 = stop
  
  uas->send_message(req);
  
  // Stream IDs:
  // 0 = All
  // 1 = Raw sensors
  // 2 = Extended status
  // 3 = RC channels
  // 4 = Raw controller
  // 6 = Position
  // 10 = Extra1
  // 11 = Extra2
  // 12 = Extra3
}
```

---

## Troubleshooting

### Plugin Not Loading
```bash
# Check for load errors
python3 launch_mavros.py 2>&1 | grep -A2 "my_sensor"

# Common issues:
# - CMakeLists.txt missing entry
# - mavros_plugins.xml syntax error
# - Compilation error in plugin
```

### No Data Received
```bash
# Check MAVLink traffic
ros2 topic echo /tota1/mavlink_source | grep msgid

# Verify message ID matches
# Check port configuration in config.yaml
```

### Commands Not Sent
```bash
# Monitor outgoing messages
ros2 topic echo /tota1/mavlink_sink

# Check if autopilot receives on correct port (14551)
```

### Debug Logging
Add to your plugin:
```cpp
// In constructor
RCLCPP_INFO(get_logger(), "Plugin initialized");

// In message handler
RCLCPP_DEBUG(get_logger(), "Received msg ID: %d", msg->msgid);

// Enable debug output
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=1
```

---

## Quick Reference

### File Locations
- **Plugins**: `src/mavros_custom/mavros/src/plugins/`
- **CMakeLists**: `src/mavros_custom/mavros/CMakeLists.txt`
- **Plugin Registry**: `src/mavros_custom/mavros/mavros_plugins.xml`
- **Dialects**: `src/mavros_custom/mavros/libmavconn/cmake/Modules/mavlink/message_definitions/v1.0/`

### Build Commands
```bash
# Build only MAVROS
colcon build --packages-select mavros

# Clean build
rm -rf build/mavros install/mavros
colcon build --packages-select mavros

# With verbose output
colcon build --packages-select mavros --event-handlers console_direct+
```

### Testing Commands
```bash
# List all topics
ros2 topic list | grep tota1

# Monitor specific topic
ros2 topic hz /tota1/my_sensor/raw

# Check MAVLink traffic
ros2 topic echo /tota1/mavlink_source --once
ros2 topic echo /tota1/mavlink_sink --once
```
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcpputils/asserts.hpp>

namespace mavros {
namespace std_plugins {

using namespace std::placeholders;

class VelocityControlPlugin : public plugin::Plugin {
public:
  explicit VelocityControlPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "velocity_control")
  {
    cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", 10,
      std::bind(&VelocityControlPlugin::cmd_vel_cb, this, _1));
    
    RCLCPP_INFO(get_logger(), "Velocity Control plugin initialized - subscribing to cmd_vel");
  }

  Subscriptions get_subscriptions() override
  {
    return {};
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Convert linear.x (forward/backward) and angular.z (rotation) to MAVLink
    float linear_x = msg->linear.x;   // m/s forward velocity
    float angular_z = msg->angular.z; // rad/s yaw rate
    
    RCLCPP_INFO(get_logger(), "Received cmd_vel: linear.x=%.2f, angular.z=%.2f", 
                linear_x, angular_z);

    // Send as MAVLink SET_POSITION_TARGET_LOCAL_NED (for rover mode)
    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED cmd{};
    
    cmd.time_boot_ms = get_time_boot_ms();
    cmd.target_system = uas->get_tgt_system();
    cmd.target_component = uas->get_tgt_component();
    cmd.coordinate_frame = static_cast<uint8_t>(mavlink::common::MAV_FRAME::LOCAL_NED);
    
    // Type mask: only use vx and yaw_rate (ignore position and other velocities)
    cmd.type_mask = 
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::X_IGNORE) |
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::Y_IGNORE) |
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::Z_IGNORE) |
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::VY_IGNORE) |
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::VZ_IGNORE) |
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::AX_IGNORE) |
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::AY_IGNORE) |
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::AZ_IGNORE) |
      static_cast<uint16_t>(mavlink::common::POSITION_TARGET_TYPEMASK::YAW_IGNORE);
    
    // Set velocity commands
    cmd.vx = linear_x;  // Forward velocity in m/s
    cmd.vy = 0.0;       // No lateral velocity for differential drive
    cmd.vz = 0.0;       // No vertical velocity for ground robot
    cmd.yaw_rate = angular_z;  // Yaw rate in rad/s
    
    // Send the command
    uas->send_message(cmd);
    
    RCLCPP_INFO(get_logger(), "Sent velocity command: vx=%.2f, yaw_rate=%.2f",
                cmd.vx, cmd.yaw_rate);
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
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::VelocityControlPlugin)
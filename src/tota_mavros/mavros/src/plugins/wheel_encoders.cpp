/**
 * @brief Wheel Encoders plugin
 * @file wheel_encoders.cpp
 * @author Danish
 *
 * Plugin for WHEEL_ENCODERS custom MAVLink message (ID: 42003)
 */

#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mavros_msgs/msg/mavlink.hpp>

namespace mavros {
namespace extra_plugins {

/**
 * @brief Wheel Encoders plugin
 * 
 * Handles WHEEL_ENCODERS messages (ID: 42003) containing wheel encoder
 * counts and velocities for differential drive robots.
 */
class WheelEncodersPlugin : public plugin::Plugin {
public:
    explicit WheelEncodersPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "wheel_encoders")
    {
        enable_node_watch_parameters();

        // Publishers for wheel encoder data
        counts_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("~/counts", 10);
        velocities_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("~/velocities", 10);
        odom_twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("~/twist", 10);
        
        // Parameters for wheel odometry calculation
        wheel_separation = node->declare_parameter("wheel_separation", 0.5);  // meters
        wheel_radius = node->declare_parameter("wheel_radius", 0.1);  // meters
        
        RCLCPP_INFO(get_logger(), "Wheel Encoders plugin initialized - waiting for WHEEL_ENCODERS messages (ID 42003)");
    }

    Subscriptions get_subscriptions() override
    {
        // Register raw handler for WHEEL_ENCODERS message (ID 42003)
        return {
            make_handler(42003, &WheelEncodersPlugin::handle_wheel_encoders_raw)
        };
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr counts_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocities_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr odom_twist_pub;
    
    double wheel_separation;
    double wheel_radius;
    
    void handle_wheel_encoders_raw(const mavlink::mavlink_message_t *msg, const mavconn::Framing framing)
    {
        // Parse the custom message manually
        // Message structure:
        // - uint64_t time_us (8 bytes)
        // - int32_t left_count (4 bytes)
        // - int32_t right_count (4 bytes)
        // - float left_velocity (4 bytes) - rad/s
        // - float right_velocity (4 bytes) - rad/s
        
        if (!msg || msg->msgid != 42003) {
            return;
        }
        
        RCLCPP_INFO_ONCE(get_logger(), "Received first WHEEL_ENCODERS message!");

        // Extract data from payload
        uint64_t time_us;
        int32_t left_count, right_count;
        float left_velocity, right_velocity;
        
        const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg->payload64);
        
        memcpy(&time_us, payload, sizeof(uint64_t));
        memcpy(&left_count, payload + 8, sizeof(int32_t));
        memcpy(&right_count, payload + 12, sizeof(int32_t));
        memcpy(&left_velocity, payload + 16, sizeof(float));
        memcpy(&right_velocity, payload + 20, sizeof(float));

        // Publish encoder counts
        auto counts_msg = std_msgs::msg::Float32MultiArray();
        counts_msg.layout.dim.resize(1);
        counts_msg.layout.dim[0].label = "wheel";
        counts_msg.layout.dim[0].size = 2;
        counts_msg.layout.dim[0].stride = 2;
        counts_msg.data.resize(2);
        counts_msg.data[0] = static_cast<float>(left_count);
        counts_msg.data[1] = static_cast<float>(right_count);
        
        counts_pub->publish(counts_msg);

        // Publish wheel velocities
        auto vel_msg = std_msgs::msg::Float32MultiArray();
        vel_msg.layout = counts_msg.layout;
        vel_msg.data.resize(2);
        vel_msg.data[0] = left_velocity;
        vel_msg.data[1] = right_velocity;
        
        velocities_pub->publish(vel_msg);

        // Calculate and publish robot twist from differential drive kinematics
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = uas->now();
        twist_msg.header.frame_id = "base_link";
        
        // Convert wheel velocities (rad/s) to linear velocities (m/s)
        float v_left = left_velocity * wheel_radius;
        float v_right = right_velocity * wheel_radius;
        
        // Differential drive kinematics
        twist_msg.twist.linear.x = (v_left + v_right) / 2.0;  // Forward velocity
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = (v_right - v_left) / wheel_separation;  // Angular velocity
        
        odom_twist_pub->publish(twist_msg);
        
        // Log periodically
        static int msg_count = 0;
        if (++msg_count % 10 == 0) {
            RCLCPP_DEBUG(get_logger(), 
                "Wheel Encoders: received %d messages, counts: [%d, %d], velocities: [%.2f, %.2f] rad/s", 
                msg_count, left_count, right_count, left_velocity, right_velocity);
        }
    }
};

}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::WheelEncodersPlugin)
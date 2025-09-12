/**
 * @brief TOF L7CX Meta plugin
 * @file tof_meta.cpp
 * @author Danish
 *
 * Plugin for TOF_L7CX_META custom MAVLink message (ID: 42002)
 */

#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <mavros_msgs/msg/mavlink.hpp>

namespace mavros {
namespace extra_plugins {

/**
 * @brief TOF L7CX Meta plugin
 * 
 * Handles TOF_L7CX_META messages (ID: 42002) containing 8x8 zone metadata
 * (status and confidence) from VL53L7CX sensor.
 */
class TofMetaPlugin : public plugin::Plugin {
public:
    explicit TofMetaPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "tof_meta")
    {
        enable_node_watch_parameters();

        // Publishers for status and confidence
        status_pub = node->create_publisher<std_msgs::msg::UInt8MultiArray>("~/status", 10);
        confidence_pub = node->create_publisher<std_msgs::msg::UInt8MultiArray>("~/confidence", 10);
        
        RCLCPP_INFO(get_logger(), "TOF Meta plugin initialized - waiting for TOF_META messages (ID 42002)");
    }

    Subscriptions get_subscriptions() override
    {
        // Register raw handler for TOF_L7CX_META message (ID 42002)
        return {
            make_handler(42002, &TofMetaPlugin::handle_tof_meta_raw)
        };
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr status_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr confidence_pub;
    
    void handle_tof_meta_raw(const mavlink::mavlink_message_t *msg, const mavconn::Framing framing)
    {
        // Parse the custom message manually
        // Message structure:
        // - uint64_t time_us (8 bytes)
        // - uint8_t status[64] (64 bytes)
        // - uint8_t confidence[64] (64 bytes)
        
        if (!msg || msg->msgid != 42002) {
            return;
        }
        
        RCLCPP_INFO_ONCE(get_logger(), "Received first TOF_L7CX_META message!");

        // Extract timestamp (first 8 bytes of payload)
        uint64_t time_us;
        memcpy(&time_us, &msg->payload64[0], sizeof(uint64_t));

        // Extract status array (next 64 bytes)
        std::vector<uint8_t> status(64);
        memcpy(status.data(), reinterpret_cast<const uint8_t*>(&msg->payload64[1]), 64);

        // Extract confidence array (next 64 bytes after status)
        std::vector<uint8_t> confidence(64);
        memcpy(confidence.data(), reinterpret_cast<const uint8_t*>(&msg->payload64[1]) + 64, 64);

        // Publish status array
        auto status_msg = std_msgs::msg::UInt8MultiArray();
        status_msg.layout.dim.resize(2);
        status_msg.layout.dim[0].label = "rows";
        status_msg.layout.dim[0].size = 8;
        status_msg.layout.dim[0].stride = 64;
        status_msg.layout.dim[1].label = "cols";
        status_msg.layout.dim[1].size = 8;
        status_msg.layout.dim[1].stride = 8;
        status_msg.data = status;
        
        status_pub->publish(status_msg);

        // Publish confidence array
        auto conf_msg = std_msgs::msg::UInt8MultiArray();
        conf_msg.layout = status_msg.layout;  // Same structure
        conf_msg.data = confidence;
        
        confidence_pub->publish(conf_msg);
        
        // Log periodically
        static int msg_count = 0;
        if (++msg_count % 10 == 0) {
            RCLCPP_DEBUG(get_logger(), 
                "TOF Meta: received %d messages, latest timestamp: %lu us", 
                msg_count, time_us);
        }
    }
};

}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TofMetaPlugin)
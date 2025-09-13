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
#include <std_msgs/msg/u_int16_multi_array.hpp>

namespace mavros {
namespace extra_plugins {

/**
 * @brief TOF L7CX Meta plugin
 * 
 * Handles TOF_L7CX_META messages (ID: 42002) containing status and signal
 * strength for each VL53L7CX zone.
 */
class TofMetaPlugin : public plugin::Plugin {
public:
    explicit TofMetaPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "tof_meta")
    {
        enable_node_watch_parameters();

        // Publishers for metadata
        status_pub = node->create_publisher<std_msgs::msg::UInt8MultiArray>("~/status", 10);
        signal_pub = node->create_publisher<std_msgs::msg::UInt16MultiArray>("~/signal", 10);
        
        RCLCPP_INFO(get_logger(), "TOF Meta plugin initialized - waiting for TOF_L7CX_META messages (ID 42002)");
    }

    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&TofMetaPlugin::handle_tof_meta)
        };
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr status_pub;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr signal_pub;
    
    bool first_msg_received = false;

    void handle_tof_meta(
        const mavlink::mavlink_message_t * msg [[maybe_unused]],
        mavlink::tota_dialect::msg::TOF_L7CX_META & meta_msg,
        plugin::filter::SystemAndOk filter [[maybe_unused]])
    {
        if (!first_msg_received) {
            RCLCPP_INFO(get_logger(), "Received first TOF_L7CX_META message!");
            first_msg_received = true;
        }

        // Publish status array (0 = valid)
        auto status_msg = std_msgs::msg::UInt8MultiArray();
        status_msg.layout.dim.resize(2);
        status_msg.layout.dim[0].label = "rows";
        status_msg.layout.dim[0].size = 8;
        status_msg.layout.dim[0].stride = 64;
        status_msg.layout.dim[1].label = "cols";
        status_msg.layout.dim[1].size = 8;
        status_msg.layout.dim[1].stride = 8;
        
        status_msg.data.reserve(64);
        for (int i = 0; i < 64; i++) {
            status_msg.data.push_back(meta_msg.status[i]);
        }
        status_pub->publish(status_msg);
        
        // Publish signal strength array (in kcps)
        auto signal_msg = std_msgs::msg::UInt16MultiArray();
        signal_msg.layout.dim.resize(2);
        signal_msg.layout.dim[0].label = "rows";
        signal_msg.layout.dim[0].size = 8;
        signal_msg.layout.dim[0].stride = 64;
        signal_msg.layout.dim[1].label = "cols";
        signal_msg.layout.dim[1].size = 8;
        signal_msg.layout.dim[1].stride = 8;
        
        signal_msg.data.reserve(64);
        for (int i = 0; i < 64; i++) {
            signal_msg.data.push_back(meta_msg.signal[i]);
        }
        signal_pub->publish(signal_msg);
    }
};

}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TofMetaPlugin)
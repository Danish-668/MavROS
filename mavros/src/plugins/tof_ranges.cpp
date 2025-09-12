/**
 * @brief TOF L7CX Ranges plugin
 * @file tof_ranges.cpp
 * @author Danish
 *
 * Plugin for TOF_L7CX_RANGES custom MAVLink message (ID: 42001)
 */

#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mavros_msgs/msg/mavlink.hpp>

namespace mavros {
namespace extra_plugins {

/**
 * @brief TOF L7CX Ranges plugin
 * 
 * Handles TOF_L7CX_RANGES messages (ID: 42001) containing 8x8 zone distances
 * from VL53L7CX sensor and publishes them as ROS2 topics.
 */
class TofRangesPlugin : public plugin::Plugin {
public:
    explicit TofRangesPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "tof_ranges")
    {
        enable_node_watch_parameters();

        // Publishers for different representations
        ranges_raw_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("~/raw", 10);
        ranges_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud", 10);
        
        RCLCPP_INFO(get_logger(), "TOF Ranges plugin initialized - waiting for TOF messages (ID 42001)");
    }

    Subscriptions get_subscriptions() override
    {
        // Register raw handler for TOF_L7CX_RANGES message (ID 42001)
        return {
            make_handler(42001, &TofRangesPlugin::handle_tof_ranges_raw)
        };
    }
    
    void connection_cb(bool connected) override
    {
        if (connected) {
            RCLCPP_INFO(get_logger(), "TOF plugin connected - ready to receive messages");
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ranges_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ranges_cloud_pub;
    
    void handle_tof_ranges_raw(const mavlink::mavlink_message_t *msg, const mavconn::Framing framing)
    {
        // Parse the custom message manually
        // Message structure:
        // - uint64_t time_us (8 bytes)
        // - uint16_t range_mm[64] (128 bytes)
        
        if (!msg || msg->msgid != 42001) {
            return;
        }
        
        RCLCPP_INFO_ONCE(get_logger(), "Received first TOF_L7CX_RANGES message!");

        // Extract timestamp (first 8 bytes of payload)
        uint64_t time_us;
        memcpy(&time_us, &msg->payload64[0], sizeof(uint64_t));

        // Extract ranges (next 128 bytes as 64 uint16_t values)
        std::vector<uint16_t> ranges_mm(64);
        // payload64[1] onwards contains the range data (payload64 is array of uint64_t)
        memcpy(ranges_mm.data(), &msg->payload64[1], 64 * sizeof(uint16_t));

        // Publish as raw array
        auto raw_msg = std_msgs::msg::Float32MultiArray();
        raw_msg.layout.dim.resize(2);
        raw_msg.layout.dim[0].label = "rows";
        raw_msg.layout.dim[0].size = 8;
        raw_msg.layout.dim[0].stride = 64;
        raw_msg.layout.dim[1].label = "cols";
        raw_msg.layout.dim[1].size = 8;
        raw_msg.layout.dim[1].stride = 8;
        
        raw_msg.data.resize(64);
        for (size_t i = 0; i < 64; i++) {
            raw_msg.data[i] = ranges_mm[i] / 1000.0f;  // Convert mm to meters
        }
        
        ranges_raw_pub->publish(raw_msg);

        // Also publish as point cloud
        publish_as_pointcloud(ranges_mm, time_us);
        
        // Log periodically
        static int msg_count = 0;
        if (++msg_count % 10 == 0) {
            RCLCPP_DEBUG(get_logger(), 
                "TOF Ranges: received %d messages, latest timestamp: %lu us", 
                msg_count, time_us);
        }
    }

    void publish_as_pointcloud(const std::vector<uint16_t>& ranges_mm, uint64_t time_us)
    {
        auto cloud_msg = sensor_msgs::msg::PointCloud2();
        
        // Setup message metadata
        cloud_msg.header.stamp = uas->now();
        cloud_msg.header.frame_id = "tof_sensor";
        cloud_msg.height = 8;
        cloud_msg.width = 8;
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        
        // Define point cloud fields (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(64);
        
        // Fill point cloud data
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        
        // TOF sensor field of view (approximate)
        const float fov_deg = 45.0f;  // 45 degree field of view
        const float fov_rad = fov_deg * M_PI / 180.0f;
        const float angle_per_zone = fov_rad / 8.0f;
        
        for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 8; col++) {
                int idx = row * 8 + col;
                float range_m = ranges_mm[idx] / 1000.0f;
                
                // Calculate angles for this zone
                float angle_x = (col - 3.5f) * angle_per_zone;
                float angle_y = (row - 3.5f) * angle_per_zone;
                
                // Convert to Cartesian coordinates
                *iter_z = range_m;
                *iter_x = range_m * tan(angle_x);
                *iter_y = range_m * tan(angle_y);
                
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }
        
        ranges_cloud_pub->publish(cloud_msg);
    }
};

}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TofRangesPlugin)
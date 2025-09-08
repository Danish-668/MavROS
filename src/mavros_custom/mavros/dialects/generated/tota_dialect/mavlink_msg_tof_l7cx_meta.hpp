// MESSAGE TOF_L7CX_META support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief TOF_L7CX_META message
 *
 * Status (0 = valid) and signal strength for each VL53L7CX zone.
 */
struct TOF_L7CX_META : mavlink::Message {
    static constexpr msgid_t MSG_ID = 42002;
    static constexpr size_t LENGTH = 200;
    static constexpr size_t MIN_LENGTH = 200;
    static constexpr uint8_t CRC_EXTRA = 120;
    static constexpr auto NAME = "TOF_L7CX_META";


    uint64_t time_us; /*< [us]  */
    std::array<uint8_t, 64> status; /*<   */
    std::array<uint16_t, 64> signal; /*< [kcps]  */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  time_us: " << time_us << std::endl;
        ss << "  status: [" << to_string(status) << "]" << std::endl;
        ss << "  signal: [" << to_string(signal) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_us;                       // offset: 0
        map << signal;                        // offset: 8
        map << status;                        // offset: 136
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_us;                       // offset: 0
        map >> signal;                        // offset: 8
        map >> status;                        // offset: 136
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink

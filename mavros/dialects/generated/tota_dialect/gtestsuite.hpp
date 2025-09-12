/** @file
 *	@brief MAVLink comm testsuite protocol generated from tota_dialect.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "tota_dialect.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(tota_dialect, TOF_L7CX_RANGES)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet_in{};
    packet_in.time_us = 93372036854775807ULL;
    packet_in.range_mm = {{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714 }};

    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet1{};
    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_us, packet2.time_us);
    EXPECT_EQ(packet1.range_mm, packet2.range_mm);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, TOF_L7CX_RANGES)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_tof_l7cx_ranges_t packet_c {
         93372036854775807ULL, { 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714 }
    };

    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet_in{};
    packet_in.time_us = 93372036854775807ULL;
    packet_in.range_mm = {{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714 }};

    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet2{};

    mavlink_msg_tof_l7cx_ranges_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_us, packet2.time_us);
    EXPECT_EQ(packet_in.range_mm, packet2.range_mm);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, TOF_L7CX_META)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::TOF_L7CX_META packet_in{};
    packet_in.time_us = 93372036854775807ULL;
    packet_in.status = {{ 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220 }};
    packet_in.signal = {{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714 }};

    mavlink::tota_dialect::msg::TOF_L7CX_META packet1{};
    mavlink::tota_dialect::msg::TOF_L7CX_META packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_us, packet2.time_us);
    EXPECT_EQ(packet1.status, packet2.status);
    EXPECT_EQ(packet1.signal, packet2.signal);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, TOF_L7CX_META)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_tof_l7cx_meta_t packet_c {
         93372036854775807ULL, { 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714 }, { 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220 }
    };

    mavlink::tota_dialect::msg::TOF_L7CX_META packet_in{};
    packet_in.time_us = 93372036854775807ULL;
    packet_in.status = {{ 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220 }};
    packet_in.signal = {{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714 }};

    mavlink::tota_dialect::msg::TOF_L7CX_META packet2{};

    mavlink_msg_tof_l7cx_meta_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_us, packet2.time_us);
    EXPECT_EQ(packet_in.status, packet2.status);
    EXPECT_EQ(packet_in.signal, packet2.signal);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, WHEEL_ENCODERS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet_in{};
    packet_in.time_us = 93372036854775807ULL;
    packet_in.left_ticks = 963497880;
    packet_in.right_ticks = 963498088;
    packet_in.cam_pos = 18067;

    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet1{};
    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_us, packet2.time_us);
    EXPECT_EQ(packet1.left_ticks, packet2.left_ticks);
    EXPECT_EQ(packet1.right_ticks, packet2.right_ticks);
    EXPECT_EQ(packet1.cam_pos, packet2.cam_pos);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, WHEEL_ENCODERS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_wheel_encoders_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 18067
    };

    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet_in{};
    packet_in.time_us = 93372036854775807ULL;
    packet_in.left_ticks = 963497880;
    packet_in.right_ticks = 963498088;
    packet_in.cam_pos = 18067;

    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet2{};

    mavlink_msg_wheel_encoders_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_us, packet2.time_us);
    EXPECT_EQ(packet_in.left_ticks, packet2.left_ticks);
    EXPECT_EQ(packet_in.right_ticks, packet2.right_ticks);
    EXPECT_EQ(packet_in.cam_pos, packet2.cam_pos);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

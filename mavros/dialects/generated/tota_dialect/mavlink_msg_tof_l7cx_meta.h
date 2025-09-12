#pragma once
// MESSAGE TOF_L7CX_META PACKING

#define MAVLINK_MSG_ID_TOF_L7CX_META 42002


typedef struct __mavlink_tof_l7cx_meta_t {
 uint64_t time_us; /*< [us] */
 uint16_t signal[64]; /*< [kcps] */
 uint8_t status[64]; /*<  */
} mavlink_tof_l7cx_meta_t;

#define MAVLINK_MSG_ID_TOF_L7CX_META_LEN 200
#define MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN 200
#define MAVLINK_MSG_ID_42002_LEN 200
#define MAVLINK_MSG_ID_42002_MIN_LEN 200

#define MAVLINK_MSG_ID_TOF_L7CX_META_CRC 120
#define MAVLINK_MSG_ID_42002_CRC 120

#define MAVLINK_MSG_TOF_L7CX_META_FIELD_SIGNAL_LEN 64
#define MAVLINK_MSG_TOF_L7CX_META_FIELD_STATUS_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TOF_L7CX_META { \
    42002, \
    "TOF_L7CX_META", \
    3, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tof_l7cx_meta_t, time_us) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 64, 136, offsetof(mavlink_tof_l7cx_meta_t, status) }, \
         { "signal", NULL, MAVLINK_TYPE_UINT16_T, 64, 8, offsetof(mavlink_tof_l7cx_meta_t, signal) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TOF_L7CX_META { \
    "TOF_L7CX_META", \
    3, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tof_l7cx_meta_t, time_us) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 64, 136, offsetof(mavlink_tof_l7cx_meta_t, status) }, \
         { "signal", NULL, MAVLINK_TYPE_UINT16_T, 64, 8, offsetof(mavlink_tof_l7cx_meta_t, signal) }, \
         } \
}
#endif

/**
 * @brief Pack a tof_l7cx_meta message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] 
 * @param status  
 * @param signal [kcps] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tof_l7cx_meta_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_us, const uint8_t *status, const uint16_t *signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_L7CX_META_LEN];
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, signal, 64);
    _mav_put_uint8_t_array(buf, 136, status, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOF_L7CX_META_LEN);
#else
    mavlink_tof_l7cx_meta_t packet;
    packet.time_us = time_us;
    mav_array_memcpy(packet.signal, signal, sizeof(uint16_t)*64);
    mav_array_memcpy(packet.status, status, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOF_L7CX_META_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TOF_L7CX_META;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_CRC);
}

/**
 * @brief Pack a tof_l7cx_meta message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] 
 * @param status  
 * @param signal [kcps] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tof_l7cx_meta_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_us, const uint8_t *status, const uint16_t *signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_L7CX_META_LEN];
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, signal, 64);
    _mav_put_uint8_t_array(buf, 136, status, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOF_L7CX_META_LEN);
#else
    mavlink_tof_l7cx_meta_t packet;
    packet.time_us = time_us;
    mav_array_memcpy(packet.signal, signal, sizeof(uint16_t)*64);
    mav_array_memcpy(packet.status, status, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOF_L7CX_META_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TOF_L7CX_META;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN);
#endif
}

/**
 * @brief Pack a tof_l7cx_meta message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us [us] 
 * @param status  
 * @param signal [kcps] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tof_l7cx_meta_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_us,const uint8_t *status,const uint16_t *signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_L7CX_META_LEN];
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, signal, 64);
    _mav_put_uint8_t_array(buf, 136, status, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOF_L7CX_META_LEN);
#else
    mavlink_tof_l7cx_meta_t packet;
    packet.time_us = time_us;
    mav_array_memcpy(packet.signal, signal, sizeof(uint16_t)*64);
    mav_array_memcpy(packet.status, status, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOF_L7CX_META_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TOF_L7CX_META;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_CRC);
}

/**
 * @brief Encode a tof_l7cx_meta struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tof_l7cx_meta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tof_l7cx_meta_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tof_l7cx_meta_t* tof_l7cx_meta)
{
    return mavlink_msg_tof_l7cx_meta_pack(system_id, component_id, msg, tof_l7cx_meta->time_us, tof_l7cx_meta->status, tof_l7cx_meta->signal);
}

/**
 * @brief Encode a tof_l7cx_meta struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tof_l7cx_meta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tof_l7cx_meta_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tof_l7cx_meta_t* tof_l7cx_meta)
{
    return mavlink_msg_tof_l7cx_meta_pack_chan(system_id, component_id, chan, msg, tof_l7cx_meta->time_us, tof_l7cx_meta->status, tof_l7cx_meta->signal);
}

/**
 * @brief Encode a tof_l7cx_meta struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tof_l7cx_meta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tof_l7cx_meta_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tof_l7cx_meta_t* tof_l7cx_meta)
{
    return mavlink_msg_tof_l7cx_meta_pack_status(system_id, component_id, _status, msg,  tof_l7cx_meta->time_us, tof_l7cx_meta->status, tof_l7cx_meta->signal);
}

/**
 * @brief Send a tof_l7cx_meta message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us [us] 
 * @param status  
 * @param signal [kcps] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tof_l7cx_meta_send(mavlink_channel_t chan, uint64_t time_us, const uint8_t *status, const uint16_t *signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_L7CX_META_LEN];
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, signal, 64);
    _mav_put_uint8_t_array(buf, 136, status, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_META, buf, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_CRC);
#else
    mavlink_tof_l7cx_meta_t packet;
    packet.time_us = time_us;
    mav_array_memcpy(packet.signal, signal, sizeof(uint16_t)*64);
    mav_array_memcpy(packet.status, status, sizeof(uint8_t)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_META, (const char *)&packet, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_CRC);
#endif
}

/**
 * @brief Send a tof_l7cx_meta message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tof_l7cx_meta_send_struct(mavlink_channel_t chan, const mavlink_tof_l7cx_meta_t* tof_l7cx_meta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tof_l7cx_meta_send(chan, tof_l7cx_meta->time_us, tof_l7cx_meta->status, tof_l7cx_meta->signal);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_META, (const char *)tof_l7cx_meta, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_CRC);
#endif
}

#if MAVLINK_MSG_ID_TOF_L7CX_META_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tof_l7cx_meta_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_us, const uint8_t *status, const uint16_t *signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, signal, 64);
    _mav_put_uint8_t_array(buf, 136, status, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_META, buf, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_CRC);
#else
    mavlink_tof_l7cx_meta_t *packet = (mavlink_tof_l7cx_meta_t *)msgbuf;
    packet->time_us = time_us;
    mav_array_memcpy(packet->signal, signal, sizeof(uint16_t)*64);
    mav_array_memcpy(packet->status, status, sizeof(uint8_t)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_META, (const char *)packet, MAVLINK_MSG_ID_TOF_L7CX_META_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_LEN, MAVLINK_MSG_ID_TOF_L7CX_META_CRC);
#endif
}
#endif

#endif

// MESSAGE TOF_L7CX_META UNPACKING


/**
 * @brief Get field time_us from tof_l7cx_meta message
 *
 * @return [us] 
 */
static inline uint64_t mavlink_msg_tof_l7cx_meta_get_time_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field status from tof_l7cx_meta message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_tof_l7cx_meta_get_status(const mavlink_message_t* msg, uint8_t *status)
{
    return _MAV_RETURN_uint8_t_array(msg, status, 64,  136);
}

/**
 * @brief Get field signal from tof_l7cx_meta message
 *
 * @return [kcps] 
 */
static inline uint16_t mavlink_msg_tof_l7cx_meta_get_signal(const mavlink_message_t* msg, uint16_t *signal)
{
    return _MAV_RETURN_uint16_t_array(msg, signal, 64,  8);
}

/**
 * @brief Decode a tof_l7cx_meta message into a struct
 *
 * @param msg The message to decode
 * @param tof_l7cx_meta C-struct to decode the message contents into
 */
static inline void mavlink_msg_tof_l7cx_meta_decode(const mavlink_message_t* msg, mavlink_tof_l7cx_meta_t* tof_l7cx_meta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tof_l7cx_meta->time_us = mavlink_msg_tof_l7cx_meta_get_time_us(msg);
    mavlink_msg_tof_l7cx_meta_get_signal(msg, tof_l7cx_meta->signal);
    mavlink_msg_tof_l7cx_meta_get_status(msg, tof_l7cx_meta->status);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TOF_L7CX_META_LEN? msg->len : MAVLINK_MSG_ID_TOF_L7CX_META_LEN;
        memset(tof_l7cx_meta, 0, MAVLINK_MSG_ID_TOF_L7CX_META_LEN);
    memcpy(tof_l7cx_meta, _MAV_PAYLOAD(msg), len);
#endif
}

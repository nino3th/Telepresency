// MESSAGE ARLOBOT_DIST_SENSOR_DATA PACKING

#define MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA 219

typedef struct __mavlink_arlobot_dist_sensor_data_t
{
 uint16_t ping_distance[6]; ///< 6 PING))) ultrasonic distance sensor value (cm)
 uint16_t ir_distance[3]; ///< Infrared distance sensor value (cm.)
} mavlink_arlobot_dist_sensor_data_t;

#define MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN 18
#define MAVLINK_MSG_ID_219_LEN 18

#define MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_CRC 121
#define MAVLINK_MSG_ID_219_CRC 121

#define MAVLINK_MSG_ARLOBOT_DIST_SENSOR_DATA_FIELD_PING_DISTANCE_LEN 6
#define MAVLINK_MSG_ARLOBOT_DIST_SENSOR_DATA_FIELD_IR_DISTANCE_LEN 3

#define MAVLINK_MESSAGE_INFO_ARLOBOT_DIST_SENSOR_DATA { \
	"ARLOBOT_DIST_SENSOR_DATA", \
	2, \
	{  { "ping_distance", NULL, MAVLINK_TYPE_UINT16_T, 6, 0, offsetof(mavlink_arlobot_dist_sensor_data_t, ping_distance) }, \
         { "ir_distance", NULL, MAVLINK_TYPE_UINT16_T, 3, 12, offsetof(mavlink_arlobot_dist_sensor_data_t, ir_distance) }, \
         } \
}


/**
 * @brief Pack a arlobot_dist_sensor_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ping_distance 6 PING))) ultrasonic distance sensor value (cm)
 * @param ir_distance Infrared distance sensor value (cm.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_dist_sensor_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint16_t *ping_distance, const uint16_t *ir_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN];

	_mav_put_uint16_t_array(buf, 0, ping_distance, 6);
	_mav_put_uint16_t_array(buf, 12, ir_distance, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#else
	mavlink_arlobot_dist_sensor_data_t packet;

	mav_array_memcpy(packet.ping_distance, ping_distance, sizeof(uint16_t)*6);
	mav_array_memcpy(packet.ir_distance, ir_distance, sizeof(uint16_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif
}

/**
 * @brief Pack a arlobot_dist_sensor_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ping_distance 6 PING))) ultrasonic distance sensor value (cm)
 * @param ir_distance Infrared distance sensor value (cm.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_dist_sensor_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint16_t *ping_distance,const uint16_t *ir_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN];

	_mav_put_uint16_t_array(buf, 0, ping_distance, 6);
	_mav_put_uint16_t_array(buf, 12, ir_distance, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#else
	mavlink_arlobot_dist_sensor_data_t packet;

	mav_array_memcpy(packet.ping_distance, ping_distance, sizeof(uint16_t)*6);
	mav_array_memcpy(packet.ir_distance, ir_distance, sizeof(uint16_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif
}

/**
 * @brief Encode a arlobot_dist_sensor_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_dist_sensor_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_dist_sensor_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arlobot_dist_sensor_data_t* arlobot_dist_sensor_data)
{
	return mavlink_msg_arlobot_dist_sensor_data_pack(system_id, component_id, msg, arlobot_dist_sensor_data->ping_distance, arlobot_dist_sensor_data->ir_distance);
}

/**
 * @brief Encode a arlobot_dist_sensor_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_dist_sensor_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_dist_sensor_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arlobot_dist_sensor_data_t* arlobot_dist_sensor_data)
{
	return mavlink_msg_arlobot_dist_sensor_data_pack_chan(system_id, component_id, chan, msg, arlobot_dist_sensor_data->ping_distance, arlobot_dist_sensor_data->ir_distance);
}

/**
 * @brief Send a arlobot_dist_sensor_data message
 * @param chan MAVLink channel to send the message
 *
 * @param ping_distance 6 PING))) ultrasonic distance sensor value (cm)
 * @param ir_distance Infrared distance sensor value (cm.)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arlobot_dist_sensor_data_send(mavlink_channel_t chan, const uint16_t *ping_distance, const uint16_t *ir_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN];

	_mav_put_uint16_t_array(buf, 0, ping_distance, 6);
	_mav_put_uint16_t_array(buf, 12, ir_distance, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif
#else
	mavlink_arlobot_dist_sensor_data_t packet;

	mav_array_memcpy(packet.ping_distance, ping_distance, sizeof(uint16_t)*6);
	mav_array_memcpy(packet.ir_distance, ir_distance, sizeof(uint16_t)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arlobot_dist_sensor_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *ping_distance, const uint16_t *ir_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;

	_mav_put_uint16_t_array(buf, 0, ping_distance, 6);
	_mav_put_uint16_t_array(buf, 12, ir_distance, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif
#else
	mavlink_arlobot_dist_sensor_data_t *packet = (mavlink_arlobot_dist_sensor_data_t *)msgbuf;

	mav_array_memcpy(packet->ping_distance, ping_distance, sizeof(uint16_t)*6);
	mav_array_memcpy(packet->ir_distance, ir_distance, sizeof(uint16_t)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARLOBOT_DIST_SENSOR_DATA UNPACKING


/**
 * @brief Get field ping_distance from arlobot_dist_sensor_data message
 *
 * @return 6 PING))) ultrasonic distance sensor value (cm)
 */
static inline uint16_t mavlink_msg_arlobot_dist_sensor_data_get_ping_distance(const mavlink_message_t* msg, uint16_t *ping_distance)
{
	return _MAV_RETURN_uint16_t_array(msg, ping_distance, 6,  0);
}

/**
 * @brief Get field ir_distance from arlobot_dist_sensor_data message
 *
 * @return Infrared distance sensor value (cm.)
 */
static inline uint16_t mavlink_msg_arlobot_dist_sensor_data_get_ir_distance(const mavlink_message_t* msg, uint16_t *ir_distance)
{
	return _MAV_RETURN_uint16_t_array(msg, ir_distance, 3,  12);
}

/**
 * @brief Decode a arlobot_dist_sensor_data message into a struct
 *
 * @param msg The message to decode
 * @param arlobot_dist_sensor_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_arlobot_dist_sensor_data_decode(const mavlink_message_t* msg, mavlink_arlobot_dist_sensor_data_t* arlobot_dist_sensor_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_arlobot_dist_sensor_data_get_ping_distance(msg, arlobot_dist_sensor_data->ping_distance);
	mavlink_msg_arlobot_dist_sensor_data_get_ir_distance(msg, arlobot_dist_sensor_data->ir_distance);
#else
	memcpy(arlobot_dist_sensor_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARLOBOT_DIST_SENSOR_DATA_LEN);
#endif
}

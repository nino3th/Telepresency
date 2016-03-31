// MESSAGE ARLOBOT_ODOMETRY_DATA PACKING

#define MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA 224

typedef struct __mavlink_arlobot_odometry_data_t
{
 float x; ///< X position (meter)
 float y; ///< Y position (meter)
 float heading; ///< Heading (radian)
 float v; ///< Translational velocity (meter/sec.)
 float omega; ///< Angular velocity (rad/sec.)
} mavlink_arlobot_odometry_data_t;

#define MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN 20
#define MAVLINK_MSG_ID_224_LEN 20

#define MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_CRC 62
#define MAVLINK_MSG_ID_224_CRC 62



#define MAVLINK_MESSAGE_INFO_ARLOBOT_ODOMETRY_DATA { \
	"ARLOBOT_ODOMETRY_DATA", \
	5, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_arlobot_odometry_data_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_arlobot_odometry_data_t, y) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_arlobot_odometry_data_t, heading) }, \
         { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_arlobot_odometry_data_t, v) }, \
         { "omega", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_arlobot_odometry_data_t, omega) }, \
         } \
}


/**
 * @brief Pack a arlobot_odometry_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x X position (meter)
 * @param y Y position (meter)
 * @param heading Heading (radian)
 * @param v Translational velocity (meter/sec.)
 * @param omega Angular velocity (rad/sec.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_odometry_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float x, float y, float heading, float v, float omega)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, heading);
	_mav_put_float(buf, 12, v);
	_mav_put_float(buf, 16, omega);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#else
	mavlink_arlobot_odometry_data_t packet;
	packet.x = x;
	packet.y = y;
	packet.heading = heading;
	packet.v = v;
	packet.omega = omega;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif
}

/**
 * @brief Pack a arlobot_odometry_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x X position (meter)
 * @param y Y position (meter)
 * @param heading Heading (radian)
 * @param v Translational velocity (meter/sec.)
 * @param omega Angular velocity (rad/sec.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_odometry_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float x,float y,float heading,float v,float omega)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, heading);
	_mav_put_float(buf, 12, v);
	_mav_put_float(buf, 16, omega);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#else
	mavlink_arlobot_odometry_data_t packet;
	packet.x = x;
	packet.y = y;
	packet.heading = heading;
	packet.v = v;
	packet.omega = omega;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif
}

/**
 * @brief Encode a arlobot_odometry_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_odometry_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_odometry_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arlobot_odometry_data_t* arlobot_odometry_data)
{
	return mavlink_msg_arlobot_odometry_data_pack(system_id, component_id, msg, arlobot_odometry_data->x, arlobot_odometry_data->y, arlobot_odometry_data->heading, arlobot_odometry_data->v, arlobot_odometry_data->omega);
}

/**
 * @brief Encode a arlobot_odometry_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_odometry_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_odometry_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arlobot_odometry_data_t* arlobot_odometry_data)
{
	return mavlink_msg_arlobot_odometry_data_pack_chan(system_id, component_id, chan, msg, arlobot_odometry_data->x, arlobot_odometry_data->y, arlobot_odometry_data->heading, arlobot_odometry_data->v, arlobot_odometry_data->omega);
}

/**
 * @brief Send a arlobot_odometry_data message
 * @param chan MAVLink channel to send the message
 *
 * @param x X position (meter)
 * @param y Y position (meter)
 * @param heading Heading (radian)
 * @param v Translational velocity (meter/sec.)
 * @param omega Angular velocity (rad/sec.)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arlobot_odometry_data_send(mavlink_channel_t chan, float x, float y, float heading, float v, float omega)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, heading);
	_mav_put_float(buf, 12, v);
	_mav_put_float(buf, 16, omega);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif
#else
	mavlink_arlobot_odometry_data_t packet;
	packet.x = x;
	packet.y = y;
	packet.heading = heading;
	packet.v = v;
	packet.omega = omega;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arlobot_odometry_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float heading, float v, float omega)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, heading);
	_mav_put_float(buf, 12, v);
	_mav_put_float(buf, 16, omega);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif
#else
	mavlink_arlobot_odometry_data_t *packet = (mavlink_arlobot_odometry_data_t *)msgbuf;
	packet->x = x;
	packet->y = y;
	packet->heading = heading;
	packet->v = v;
	packet->omega = omega;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARLOBOT_ODOMETRY_DATA UNPACKING


/**
 * @brief Get field x from arlobot_odometry_data message
 *
 * @return X position (meter)
 */
static inline float mavlink_msg_arlobot_odometry_data_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from arlobot_odometry_data message
 *
 * @return Y position (meter)
 */
static inline float mavlink_msg_arlobot_odometry_data_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field heading from arlobot_odometry_data message
 *
 * @return Heading (radian)
 */
static inline float mavlink_msg_arlobot_odometry_data_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field v from arlobot_odometry_data message
 *
 * @return Translational velocity (meter/sec.)
 */
static inline float mavlink_msg_arlobot_odometry_data_get_v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field omega from arlobot_odometry_data message
 *
 * @return Angular velocity (rad/sec.)
 */
static inline float mavlink_msg_arlobot_odometry_data_get_omega(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a arlobot_odometry_data message into a struct
 *
 * @param msg The message to decode
 * @param arlobot_odometry_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_arlobot_odometry_data_decode(const mavlink_message_t* msg, mavlink_arlobot_odometry_data_t* arlobot_odometry_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	arlobot_odometry_data->x = mavlink_msg_arlobot_odometry_data_get_x(msg);
	arlobot_odometry_data->y = mavlink_msg_arlobot_odometry_data_get_y(msg);
	arlobot_odometry_data->heading = mavlink_msg_arlobot_odometry_data_get_heading(msg);
	arlobot_odometry_data->v = mavlink_msg_arlobot_odometry_data_get_v(msg);
	arlobot_odometry_data->omega = mavlink_msg_arlobot_odometry_data_get_omega(msg);
#else
	memcpy(arlobot_odometry_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARLOBOT_ODOMETRY_DATA_LEN);
#endif
}

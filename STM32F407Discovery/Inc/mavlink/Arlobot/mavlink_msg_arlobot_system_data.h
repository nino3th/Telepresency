// MESSAGE ARLOBOT_SYSTEM_DATA PACKING

#define MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA 226

typedef struct __mavlink_arlobot_system_data_t
{
 int32_t enc_vel_limit; ///< Encoder velocity limit, determined by distance sensors
 uint16_t state; ///< Arlobot system state. See ARLOBOT_STATE enum.
} mavlink_arlobot_system_data_t;

#define MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN 6
#define MAVLINK_MSG_ID_226_LEN 6

#define MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_CRC 214
#define MAVLINK_MSG_ID_226_CRC 214



#define MAVLINK_MESSAGE_INFO_ARLOBOT_SYSTEM_DATA { \
	"ARLOBOT_SYSTEM_DATA", \
	2, \
	{  { "enc_vel_limit", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_arlobot_system_data_t, enc_vel_limit) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_arlobot_system_data_t, state) }, \
         } \
}


/**
 * @brief Pack a arlobot_system_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state Arlobot system state. See ARLOBOT_STATE enum.
 * @param enc_vel_limit Encoder velocity limit, determined by distance sensors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_system_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t state, int32_t enc_vel_limit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN];
	_mav_put_int32_t(buf, 0, enc_vel_limit);
	_mav_put_uint16_t(buf, 4, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#else
	mavlink_arlobot_system_data_t packet;
	packet.enc_vel_limit = enc_vel_limit;
	packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif
}

/**
 * @brief Pack a arlobot_system_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state Arlobot system state. See ARLOBOT_STATE enum.
 * @param enc_vel_limit Encoder velocity limit, determined by distance sensors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_system_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t state,int32_t enc_vel_limit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN];
	_mav_put_int32_t(buf, 0, enc_vel_limit);
	_mav_put_uint16_t(buf, 4, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#else
	mavlink_arlobot_system_data_t packet;
	packet.enc_vel_limit = enc_vel_limit;
	packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif
}

/**
 * @brief Encode a arlobot_system_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_system_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_system_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arlobot_system_data_t* arlobot_system_data)
{
	return mavlink_msg_arlobot_system_data_pack(system_id, component_id, msg, arlobot_system_data->state, arlobot_system_data->enc_vel_limit);
}

/**
 * @brief Encode a arlobot_system_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_system_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_system_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arlobot_system_data_t* arlobot_system_data)
{
	return mavlink_msg_arlobot_system_data_pack_chan(system_id, component_id, chan, msg, arlobot_system_data->state, arlobot_system_data->enc_vel_limit);
}

/**
 * @brief Send a arlobot_system_data message
 * @param chan MAVLink channel to send the message
 *
 * @param state Arlobot system state. See ARLOBOT_STATE enum.
 * @param enc_vel_limit Encoder velocity limit, determined by distance sensors
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arlobot_system_data_send(mavlink_channel_t chan, uint16_t state, int32_t enc_vel_limit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN];
	_mav_put_int32_t(buf, 0, enc_vel_limit);
	_mav_put_uint16_t(buf, 4, state);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif
#else
	mavlink_arlobot_system_data_t packet;
	packet.enc_vel_limit = enc_vel_limit;
	packet.state = state;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arlobot_system_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t state, int32_t enc_vel_limit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, enc_vel_limit);
	_mav_put_uint16_t(buf, 4, state);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif
#else
	mavlink_arlobot_system_data_t *packet = (mavlink_arlobot_system_data_t *)msgbuf;
	packet->enc_vel_limit = enc_vel_limit;
	packet->state = state;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARLOBOT_SYSTEM_DATA UNPACKING


/**
 * @brief Get field state from arlobot_system_data message
 *
 * @return Arlobot system state. See ARLOBOT_STATE enum.
 */
static inline uint16_t mavlink_msg_arlobot_system_data_get_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field enc_vel_limit from arlobot_system_data message
 *
 * @return Encoder velocity limit, determined by distance sensors
 */
static inline int32_t mavlink_msg_arlobot_system_data_get_enc_vel_limit(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Decode a arlobot_system_data message into a struct
 *
 * @param msg The message to decode
 * @param arlobot_system_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_arlobot_system_data_decode(const mavlink_message_t* msg, mavlink_arlobot_system_data_t* arlobot_system_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	arlobot_system_data->enc_vel_limit = mavlink_msg_arlobot_system_data_get_enc_vel_limit(msg);
	arlobot_system_data->state = mavlink_msg_arlobot_system_data_get_state(msg);
#else
	memcpy(arlobot_system_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARLOBOT_SYSTEM_DATA_LEN);
#endif
}

// MESSAGE ARLOBOT_ENCODER_DATA PACKING

#define MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA 225

typedef struct __mavlink_arlobot_encoder_data_t
{
 int32_t enc_vel_l; ///< Left Encoder Velocity (ticks/sec.)
 int32_t enc_vel_r; ///< Right Encoder Velocity (ticks/sec.)
 int32_t enc_ticks_l; ///< Left Encoder Ticks Counter
 int32_t enc_ticks_r; ///< Right Encoder Ticks Counter
 int32_t cmd_enc_vel_l; ///< Command Left Encoder Velocity (ticks/sec.)
 int32_t cmd_enc_vel_r; ///< Command Right Encoder Velocity (ticks/sec.)
} mavlink_arlobot_encoder_data_t;

#define MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN 24
#define MAVLINK_MSG_ID_225_LEN 24

#define MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_CRC 48
#define MAVLINK_MSG_ID_225_CRC 48



#define MAVLINK_MESSAGE_INFO_ARLOBOT_ENCODER_DATA { \
	"ARLOBOT_ENCODER_DATA", \
	6, \
	{  { "enc_vel_l", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_arlobot_encoder_data_t, enc_vel_l) }, \
         { "enc_vel_r", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_arlobot_encoder_data_t, enc_vel_r) }, \
         { "enc_ticks_l", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_arlobot_encoder_data_t, enc_ticks_l) }, \
         { "enc_ticks_r", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_arlobot_encoder_data_t, enc_ticks_r) }, \
         { "cmd_enc_vel_l", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_arlobot_encoder_data_t, cmd_enc_vel_l) }, \
         { "cmd_enc_vel_r", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_arlobot_encoder_data_t, cmd_enc_vel_r) }, \
         } \
}


/**
 * @brief Pack a arlobot_encoder_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enc_vel_l Left Encoder Velocity (ticks/sec.)
 * @param enc_vel_r Right Encoder Velocity (ticks/sec.)
 * @param enc_ticks_l Left Encoder Ticks Counter
 * @param enc_ticks_r Right Encoder Ticks Counter
 * @param cmd_enc_vel_l Command Left Encoder Velocity (ticks/sec.)
 * @param cmd_enc_vel_r Command Right Encoder Velocity (ticks/sec.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_encoder_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t enc_vel_l, int32_t enc_vel_r, int32_t enc_ticks_l, int32_t enc_ticks_r, int32_t cmd_enc_vel_l, int32_t cmd_enc_vel_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN];
	_mav_put_int32_t(buf, 0, enc_vel_l);
	_mav_put_int32_t(buf, 4, enc_vel_r);
	_mav_put_int32_t(buf, 8, enc_ticks_l);
	_mav_put_int32_t(buf, 12, enc_ticks_r);
	_mav_put_int32_t(buf, 16, cmd_enc_vel_l);
	_mav_put_int32_t(buf, 20, cmd_enc_vel_r);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#else
	mavlink_arlobot_encoder_data_t packet;
	packet.enc_vel_l = enc_vel_l;
	packet.enc_vel_r = enc_vel_r;
	packet.enc_ticks_l = enc_ticks_l;
	packet.enc_ticks_r = enc_ticks_r;
	packet.cmd_enc_vel_l = cmd_enc_vel_l;
	packet.cmd_enc_vel_r = cmd_enc_vel_r;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif
}

/**
 * @brief Pack a arlobot_encoder_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param enc_vel_l Left Encoder Velocity (ticks/sec.)
 * @param enc_vel_r Right Encoder Velocity (ticks/sec.)
 * @param enc_ticks_l Left Encoder Ticks Counter
 * @param enc_ticks_r Right Encoder Ticks Counter
 * @param cmd_enc_vel_l Command Left Encoder Velocity (ticks/sec.)
 * @param cmd_enc_vel_r Command Right Encoder Velocity (ticks/sec.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_encoder_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t enc_vel_l,int32_t enc_vel_r,int32_t enc_ticks_l,int32_t enc_ticks_r,int32_t cmd_enc_vel_l,int32_t cmd_enc_vel_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN];
	_mav_put_int32_t(buf, 0, enc_vel_l);
	_mav_put_int32_t(buf, 4, enc_vel_r);
	_mav_put_int32_t(buf, 8, enc_ticks_l);
	_mav_put_int32_t(buf, 12, enc_ticks_r);
	_mav_put_int32_t(buf, 16, cmd_enc_vel_l);
	_mav_put_int32_t(buf, 20, cmd_enc_vel_r);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#else
	mavlink_arlobot_encoder_data_t packet;
	packet.enc_vel_l = enc_vel_l;
	packet.enc_vel_r = enc_vel_r;
	packet.enc_ticks_l = enc_ticks_l;
	packet.enc_ticks_r = enc_ticks_r;
	packet.cmd_enc_vel_l = cmd_enc_vel_l;
	packet.cmd_enc_vel_r = cmd_enc_vel_r;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif
}

/**
 * @brief Encode a arlobot_encoder_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_encoder_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_encoder_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arlobot_encoder_data_t* arlobot_encoder_data)
{
	return mavlink_msg_arlobot_encoder_data_pack(system_id, component_id, msg, arlobot_encoder_data->enc_vel_l, arlobot_encoder_data->enc_vel_r, arlobot_encoder_data->enc_ticks_l, arlobot_encoder_data->enc_ticks_r, arlobot_encoder_data->cmd_enc_vel_l, arlobot_encoder_data->cmd_enc_vel_r);
}

/**
 * @brief Encode a arlobot_encoder_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_encoder_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_encoder_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arlobot_encoder_data_t* arlobot_encoder_data)
{
	return mavlink_msg_arlobot_encoder_data_pack_chan(system_id, component_id, chan, msg, arlobot_encoder_data->enc_vel_l, arlobot_encoder_data->enc_vel_r, arlobot_encoder_data->enc_ticks_l, arlobot_encoder_data->enc_ticks_r, arlobot_encoder_data->cmd_enc_vel_l, arlobot_encoder_data->cmd_enc_vel_r);
}

/**
 * @brief Send a arlobot_encoder_data message
 * @param chan MAVLink channel to send the message
 *
 * @param enc_vel_l Left Encoder Velocity (ticks/sec.)
 * @param enc_vel_r Right Encoder Velocity (ticks/sec.)
 * @param enc_ticks_l Left Encoder Ticks Counter
 * @param enc_ticks_r Right Encoder Ticks Counter
 * @param cmd_enc_vel_l Command Left Encoder Velocity (ticks/sec.)
 * @param cmd_enc_vel_r Command Right Encoder Velocity (ticks/sec.)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arlobot_encoder_data_send(mavlink_channel_t chan, int32_t enc_vel_l, int32_t enc_vel_r, int32_t enc_ticks_l, int32_t enc_ticks_r, int32_t cmd_enc_vel_l, int32_t cmd_enc_vel_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN];
	_mav_put_int32_t(buf, 0, enc_vel_l);
	_mav_put_int32_t(buf, 4, enc_vel_r);
	_mav_put_int32_t(buf, 8, enc_ticks_l);
	_mav_put_int32_t(buf, 12, enc_ticks_r);
	_mav_put_int32_t(buf, 16, cmd_enc_vel_l);
	_mav_put_int32_t(buf, 20, cmd_enc_vel_r);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif
#else
	mavlink_arlobot_encoder_data_t packet;
	packet.enc_vel_l = enc_vel_l;
	packet.enc_vel_r = enc_vel_r;
	packet.enc_ticks_l = enc_ticks_l;
	packet.enc_ticks_r = enc_ticks_r;
	packet.cmd_enc_vel_l = cmd_enc_vel_l;
	packet.cmd_enc_vel_r = cmd_enc_vel_r;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arlobot_encoder_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t enc_vel_l, int32_t enc_vel_r, int32_t enc_ticks_l, int32_t enc_ticks_r, int32_t cmd_enc_vel_l, int32_t cmd_enc_vel_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, enc_vel_l);
	_mav_put_int32_t(buf, 4, enc_vel_r);
	_mav_put_int32_t(buf, 8, enc_ticks_l);
	_mav_put_int32_t(buf, 12, enc_ticks_r);
	_mav_put_int32_t(buf, 16, cmd_enc_vel_l);
	_mav_put_int32_t(buf, 20, cmd_enc_vel_r);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif
#else
	mavlink_arlobot_encoder_data_t *packet = (mavlink_arlobot_encoder_data_t *)msgbuf;
	packet->enc_vel_l = enc_vel_l;
	packet->enc_vel_r = enc_vel_r;
	packet->enc_ticks_l = enc_ticks_l;
	packet->enc_ticks_r = enc_ticks_r;
	packet->cmd_enc_vel_l = cmd_enc_vel_l;
	packet->cmd_enc_vel_r = cmd_enc_vel_r;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARLOBOT_ENCODER_DATA UNPACKING


/**
 * @brief Get field enc_vel_l from arlobot_encoder_data message
 *
 * @return Left Encoder Velocity (ticks/sec.)
 */
static inline int32_t mavlink_msg_arlobot_encoder_data_get_enc_vel_l(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field enc_vel_r from arlobot_encoder_data message
 *
 * @return Right Encoder Velocity (ticks/sec.)
 */
static inline int32_t mavlink_msg_arlobot_encoder_data_get_enc_vel_r(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field enc_ticks_l from arlobot_encoder_data message
 *
 * @return Left Encoder Ticks Counter
 */
static inline int32_t mavlink_msg_arlobot_encoder_data_get_enc_ticks_l(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field enc_ticks_r from arlobot_encoder_data message
 *
 * @return Right Encoder Ticks Counter
 */
static inline int32_t mavlink_msg_arlobot_encoder_data_get_enc_ticks_r(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field cmd_enc_vel_l from arlobot_encoder_data message
 *
 * @return Command Left Encoder Velocity (ticks/sec.)
 */
static inline int32_t mavlink_msg_arlobot_encoder_data_get_cmd_enc_vel_l(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field cmd_enc_vel_r from arlobot_encoder_data message
 *
 * @return Command Right Encoder Velocity (ticks/sec.)
 */
static inline int32_t mavlink_msg_arlobot_encoder_data_get_cmd_enc_vel_r(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Decode a arlobot_encoder_data message into a struct
 *
 * @param msg The message to decode
 * @param arlobot_encoder_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_arlobot_encoder_data_decode(const mavlink_message_t* msg, mavlink_arlobot_encoder_data_t* arlobot_encoder_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	arlobot_encoder_data->enc_vel_l = mavlink_msg_arlobot_encoder_data_get_enc_vel_l(msg);
	arlobot_encoder_data->enc_vel_r = mavlink_msg_arlobot_encoder_data_get_enc_vel_r(msg);
	arlobot_encoder_data->enc_ticks_l = mavlink_msg_arlobot_encoder_data_get_enc_ticks_l(msg);
	arlobot_encoder_data->enc_ticks_r = mavlink_msg_arlobot_encoder_data_get_enc_ticks_r(msg);
	arlobot_encoder_data->cmd_enc_vel_l = mavlink_msg_arlobot_encoder_data_get_cmd_enc_vel_l(msg);
	arlobot_encoder_data->cmd_enc_vel_r = mavlink_msg_arlobot_encoder_data_get_cmd_enc_vel_r(msg);
#else
	memcpy(arlobot_encoder_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARLOBOT_ENCODER_DATA_LEN);
#endif
}

// MESSAGE ARLOBOT_COMMANDER PACKING

#define MAVLINK_MSG_ID_ARLOBOT_COMMANDER 223

typedef struct __mavlink_arlobot_commander_t
{
 float param1; ///< Parameter 1, as defined by ARLOBOT_CMD enum.
 float param2; ///< Parameter 2, as defined by ARLOBOT_CMD enum.
 float param3; ///< Parameter 3, as defined by ARLOBOT_CMD enum.
 float param4; ///< Parameter 4, as defined by ARLOBOT_CMD enum.
 float param5; ///< Parameter 5, as defined by ARLOBOT_CMD enum.
 uint16_t command; ///< Command ID, as defined by ARLOBOT_CMD enum.
} mavlink_arlobot_commander_t;

#define MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN 22
#define MAVLINK_MSG_ID_223_LEN 22

#define MAVLINK_MSG_ID_ARLOBOT_COMMANDER_CRC 90
#define MAVLINK_MSG_ID_223_CRC 90



#define MAVLINK_MESSAGE_INFO_ARLOBOT_COMMANDER { \
	"ARLOBOT_COMMANDER", \
	6, \
	{  { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_arlobot_commander_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_arlobot_commander_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_arlobot_commander_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_arlobot_commander_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_arlobot_commander_t, param5) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_arlobot_commander_t, command) }, \
         } \
}


/**
 * @brief Pack a arlobot_commander message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command Command ID, as defined by ARLOBOT_CMD enum.
 * @param param1 Parameter 1, as defined by ARLOBOT_CMD enum.
 * @param param2 Parameter 2, as defined by ARLOBOT_CMD enum.
 * @param param3 Parameter 3, as defined by ARLOBOT_CMD enum.
 * @param param4 Parameter 4, as defined by ARLOBOT_CMD enum.
 * @param param5 Parameter 5, as defined by ARLOBOT_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_commander_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t command, float param1, float param2, float param3, float param4, float param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN];
	_mav_put_float(buf, 0, param1);
	_mav_put_float(buf, 4, param2);
	_mav_put_float(buf, 8, param3);
	_mav_put_float(buf, 12, param4);
	_mav_put_float(buf, 16, param5);
	_mav_put_uint16_t(buf, 20, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#else
	mavlink_arlobot_commander_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_COMMANDER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif
}

/**
 * @brief Pack a arlobot_commander message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command Command ID, as defined by ARLOBOT_CMD enum.
 * @param param1 Parameter 1, as defined by ARLOBOT_CMD enum.
 * @param param2 Parameter 2, as defined by ARLOBOT_CMD enum.
 * @param param3 Parameter 3, as defined by ARLOBOT_CMD enum.
 * @param param4 Parameter 4, as defined by ARLOBOT_CMD enum.
 * @param param5 Parameter 5, as defined by ARLOBOT_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_commander_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t command,float param1,float param2,float param3,float param4,float param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN];
	_mav_put_float(buf, 0, param1);
	_mav_put_float(buf, 4, param2);
	_mav_put_float(buf, 8, param3);
	_mav_put_float(buf, 12, param4);
	_mav_put_float(buf, 16, param5);
	_mav_put_uint16_t(buf, 20, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#else
	mavlink_arlobot_commander_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_COMMANDER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif
}

/**
 * @brief Encode a arlobot_commander struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_commander C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_commander_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arlobot_commander_t* arlobot_commander)
{
	return mavlink_msg_arlobot_commander_pack(system_id, component_id, msg, arlobot_commander->command, arlobot_commander->param1, arlobot_commander->param2, arlobot_commander->param3, arlobot_commander->param4, arlobot_commander->param5);
}

/**
 * @brief Encode a arlobot_commander struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_commander C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_commander_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arlobot_commander_t* arlobot_commander)
{
	return mavlink_msg_arlobot_commander_pack_chan(system_id, component_id, chan, msg, arlobot_commander->command, arlobot_commander->param1, arlobot_commander->param2, arlobot_commander->param3, arlobot_commander->param4, arlobot_commander->param5);
}

/**
 * @brief Send a arlobot_commander message
 * @param chan MAVLink channel to send the message
 *
 * @param command Command ID, as defined by ARLOBOT_CMD enum.
 * @param param1 Parameter 1, as defined by ARLOBOT_CMD enum.
 * @param param2 Parameter 2, as defined by ARLOBOT_CMD enum.
 * @param param3 Parameter 3, as defined by ARLOBOT_CMD enum.
 * @param param4 Parameter 4, as defined by ARLOBOT_CMD enum.
 * @param param5 Parameter 5, as defined by ARLOBOT_CMD enum.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arlobot_commander_send(mavlink_channel_t chan, uint16_t command, float param1, float param2, float param3, float param4, float param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN];
	_mav_put_float(buf, 0, param1);
	_mav_put_float(buf, 4, param2);
	_mav_put_float(buf, 8, param3);
	_mav_put_float(buf, 12, param4);
	_mav_put_float(buf, 16, param5);
	_mav_put_uint16_t(buf, 20, command);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER, buf, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER, buf, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif
#else
	mavlink_arlobot_commander_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.command = command;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arlobot_commander_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t command, float param1, float param2, float param3, float param4, float param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, param1);
	_mav_put_float(buf, 4, param2);
	_mav_put_float(buf, 8, param3);
	_mav_put_float(buf, 12, param4);
	_mav_put_float(buf, 16, param5);
	_mav_put_uint16_t(buf, 20, command);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER, buf, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER, buf, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif
#else
	mavlink_arlobot_commander_t *packet = (mavlink_arlobot_commander_t *)msgbuf;
	packet->param1 = param1;
	packet->param2 = param2;
	packet->param3 = param3;
	packet->param4 = param4;
	packet->param5 = param5;
	packet->command = command;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_COMMANDER, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARLOBOT_COMMANDER UNPACKING


/**
 * @brief Get field command from arlobot_commander message
 *
 * @return Command ID, as defined by ARLOBOT_CMD enum.
 */
static inline uint16_t mavlink_msg_arlobot_commander_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field param1 from arlobot_commander message
 *
 * @return Parameter 1, as defined by ARLOBOT_CMD enum.
 */
static inline float mavlink_msg_arlobot_commander_get_param1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field param2 from arlobot_commander message
 *
 * @return Parameter 2, as defined by ARLOBOT_CMD enum.
 */
static inline float mavlink_msg_arlobot_commander_get_param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field param3 from arlobot_commander message
 *
 * @return Parameter 3, as defined by ARLOBOT_CMD enum.
 */
static inline float mavlink_msg_arlobot_commander_get_param3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field param4 from arlobot_commander message
 *
 * @return Parameter 4, as defined by ARLOBOT_CMD enum.
 */
static inline float mavlink_msg_arlobot_commander_get_param4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field param5 from arlobot_commander message
 *
 * @return Parameter 5, as defined by ARLOBOT_CMD enum.
 */
static inline float mavlink_msg_arlobot_commander_get_param5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a arlobot_commander message into a struct
 *
 * @param msg The message to decode
 * @param arlobot_commander C-struct to decode the message contents into
 */
static inline void mavlink_msg_arlobot_commander_decode(const mavlink_message_t* msg, mavlink_arlobot_commander_t* arlobot_commander)
{
#if MAVLINK_NEED_BYTE_SWAP
	arlobot_commander->param1 = mavlink_msg_arlobot_commander_get_param1(msg);
	arlobot_commander->param2 = mavlink_msg_arlobot_commander_get_param2(msg);
	arlobot_commander->param3 = mavlink_msg_arlobot_commander_get_param3(msg);
	arlobot_commander->param4 = mavlink_msg_arlobot_commander_get_param4(msg);
	arlobot_commander->param5 = mavlink_msg_arlobot_commander_get_param5(msg);
	arlobot_commander->command = mavlink_msg_arlobot_commander_get_command(msg);
#else
	memcpy(arlobot_commander, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARLOBOT_COMMANDER_LEN);
#endif
}

// MESSAGE ARLOBOT_DOCK_DATA PACKING

#define MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA 220

typedef struct __mavlink_arlobot_dock_data_t
{
 uint16_t irv[3]; ///< 3 IR receviers' count value. 0:center; 1:left; 2:right
 uint8_t irs[3]; ///< 3 IR receivers. 0:center; 1:left; 2:right
} mavlink_arlobot_dock_data_t;

#define MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN 9
#define MAVLINK_MSG_ID_220_LEN 9

#define MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_CRC 52
#define MAVLINK_MSG_ID_220_CRC 52

#define MAVLINK_MSG_ARLOBOT_DOCK_DATA_FIELD_IRV_LEN 3
#define MAVLINK_MSG_ARLOBOT_DOCK_DATA_FIELD_IRS_LEN 3

#define MAVLINK_MESSAGE_INFO_ARLOBOT_DOCK_DATA { \
	"ARLOBOT_DOCK_DATA", \
	2, \
	{  { "irv", NULL, MAVLINK_TYPE_UINT16_T, 3, 0, offsetof(mavlink_arlobot_dock_data_t, irv) }, \
         { "irs", NULL, MAVLINK_TYPE_UINT8_T, 3, 6, offsetof(mavlink_arlobot_dock_data_t, irs) }, \
         } \
}


/**
 * @brief Pack a arlobot_dock_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param irs 3 IR receivers. 0:center; 1:left; 2:right
 * @param irv 3 IR receviers' count value. 0:center; 1:left; 2:right
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_dock_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint8_t *irs, const uint16_t *irv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN];

	_mav_put_uint16_t_array(buf, 0, irv, 3);
	_mav_put_uint8_t_array(buf, 6, irs, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#else
	mavlink_arlobot_dock_data_t packet;

	mav_array_memcpy(packet.irv, irv, sizeof(uint16_t)*3);
	mav_array_memcpy(packet.irs, irs, sizeof(uint8_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif
}

/**
 * @brief Pack a arlobot_dock_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param irs 3 IR receivers. 0:center; 1:left; 2:right
 * @param irv 3 IR receviers' count value. 0:center; 1:left; 2:right
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_dock_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint8_t *irs,const uint16_t *irv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN];

	_mav_put_uint16_t_array(buf, 0, irv, 3);
	_mav_put_uint8_t_array(buf, 6, irs, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#else
	mavlink_arlobot_dock_data_t packet;

	mav_array_memcpy(packet.irv, irv, sizeof(uint16_t)*3);
	mav_array_memcpy(packet.irs, irs, sizeof(uint8_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif
}

/**
 * @brief Encode a arlobot_dock_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_dock_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_dock_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arlobot_dock_data_t* arlobot_dock_data)
{
	return mavlink_msg_arlobot_dock_data_pack(system_id, component_id, msg, arlobot_dock_data->irs, arlobot_dock_data->irv);
}

/**
 * @brief Encode a arlobot_dock_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_dock_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_dock_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arlobot_dock_data_t* arlobot_dock_data)
{
	return mavlink_msg_arlobot_dock_data_pack_chan(system_id, component_id, chan, msg, arlobot_dock_data->irs, arlobot_dock_data->irv);
}

/**
 * @brief Send a arlobot_dock_data message
 * @param chan MAVLink channel to send the message
 *
 * @param irs 3 IR receivers. 0:center; 1:left; 2:right
 * @param irv 3 IR receviers' count value. 0:center; 1:left; 2:right
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arlobot_dock_data_send(mavlink_channel_t chan, const uint8_t *irs, const uint16_t *irv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN];

	_mav_put_uint16_t_array(buf, 0, irv, 3);
	_mav_put_uint8_t_array(buf, 6, irs, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif
#else
	mavlink_arlobot_dock_data_t packet;

	mav_array_memcpy(packet.irv, irv, sizeof(uint16_t)*3);
	mav_array_memcpy(packet.irs, irs, sizeof(uint8_t)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arlobot_dock_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *irs, const uint16_t *irv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;

	_mav_put_uint16_t_array(buf, 0, irv, 3);
	_mav_put_uint8_t_array(buf, 6, irs, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif
#else
	mavlink_arlobot_dock_data_t *packet = (mavlink_arlobot_dock_data_t *)msgbuf;

	mav_array_memcpy(packet->irv, irv, sizeof(uint16_t)*3);
	mav_array_memcpy(packet->irs, irs, sizeof(uint8_t)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARLOBOT_DOCK_DATA UNPACKING


/**
 * @brief Get field irs from arlobot_dock_data message
 *
 * @return 3 IR receivers. 0:center; 1:left; 2:right
 */
static inline uint16_t mavlink_msg_arlobot_dock_data_get_irs(const mavlink_message_t* msg, uint8_t *irs)
{
	return _MAV_RETURN_uint8_t_array(msg, irs, 3,  6);
}

/**
 * @brief Get field irv from arlobot_dock_data message
 *
 * @return 3 IR receviers' count value. 0:center; 1:left; 2:right
 */
static inline uint16_t mavlink_msg_arlobot_dock_data_get_irv(const mavlink_message_t* msg, uint16_t *irv)
{
	return _MAV_RETURN_uint16_t_array(msg, irv, 3,  0);
}

/**
 * @brief Decode a arlobot_dock_data message into a struct
 *
 * @param msg The message to decode
 * @param arlobot_dock_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_arlobot_dock_data_decode(const mavlink_message_t* msg, mavlink_arlobot_dock_data_t* arlobot_dock_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_arlobot_dock_data_get_irv(msg, arlobot_dock_data->irv);
	mavlink_msg_arlobot_dock_data_get_irs(msg, arlobot_dock_data->irs);
#else
	memcpy(arlobot_dock_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARLOBOT_DOCK_DATA_LEN);
#endif
}

// MESSAGE ARLOBOT_MATERIAL_ORDER PACKING

#define MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER 222

typedef struct __mavlink_arlobot_material_order_t
{
 uint32_t list; ///< order list
} mavlink_arlobot_material_order_t;

#define MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN 4
#define MAVLINK_MSG_ID_222_LEN 4

#define MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_CRC 74
#define MAVLINK_MSG_ID_222_CRC 74



#define MAVLINK_MESSAGE_INFO_ARLOBOT_MATERIAL_ORDER { \
	"ARLOBOT_MATERIAL_ORDER", \
	1, \
	{  { "list", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_arlobot_material_order_t, list) }, \
         } \
}


/**
 * @brief Pack a arlobot_material_order message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param list order list
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_material_order_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN];
	_mav_put_uint32_t(buf, 0, list);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#else
	mavlink_arlobot_material_order_t packet;
	packet.list = list;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif
}

/**
 * @brief Pack a arlobot_material_order message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param list order list
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_material_order_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN];
	_mav_put_uint32_t(buf, 0, list);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#else
	mavlink_arlobot_material_order_t packet;
	packet.list = list;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif
}

/**
 * @brief Encode a arlobot_material_order struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_material_order C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_material_order_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arlobot_material_order_t* arlobot_material_order)
{
	return mavlink_msg_arlobot_material_order_pack(system_id, component_id, msg, arlobot_material_order->list);
}

/**
 * @brief Encode a arlobot_material_order struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_material_order C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_material_order_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arlobot_material_order_t* arlobot_material_order)
{
	return mavlink_msg_arlobot_material_order_pack_chan(system_id, component_id, chan, msg, arlobot_material_order->list);
}

/**
 * @brief Send a arlobot_material_order message
 * @param chan MAVLink channel to send the message
 *
 * @param list order list
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arlobot_material_order_send(mavlink_channel_t chan, uint32_t list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN];
	_mav_put_uint32_t(buf, 0, list);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER, buf, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER, buf, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif
#else
	mavlink_arlobot_material_order_t packet;
	packet.list = list;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arlobot_material_order_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, list);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER, buf, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER, buf, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif
#else
	mavlink_arlobot_material_order_t *packet = (mavlink_arlobot_material_order_t *)msgbuf;
	packet->list = list;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARLOBOT_MATERIAL_ORDER UNPACKING


/**
 * @brief Get field list from arlobot_material_order message
 *
 * @return order list
 */
static inline uint32_t mavlink_msg_arlobot_material_order_get_list(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a arlobot_material_order message into a struct
 *
 * @param msg The message to decode
 * @param arlobot_material_order C-struct to decode the message contents into
 */
static inline void mavlink_msg_arlobot_material_order_decode(const mavlink_message_t* msg, mavlink_arlobot_material_order_t* arlobot_material_order)
{
#if MAVLINK_NEED_BYTE_SWAP
	arlobot_material_order->list = mavlink_msg_arlobot_material_order_get_list(msg);
#else
	memcpy(arlobot_material_order, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARLOBOT_MATERIAL_ORDER_LEN);
#endif
}

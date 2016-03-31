// MESSAGE ARLOBOT_APM_DATA PACKING

#define MAVLINK_MSG_ID_ARLOBOT_APM_DATA 221

typedef struct __mavlink_arlobot_apm_data_t
{
 float heading; ///< Heading from gyro and/or compass, 0~360 degree, CCW+
 float gps_lat; ///< Latitude (WGS84), in degrees
 float gps_lon; ///< Longitude (WGS84), in degrees
 float gps_alt; ///< Altitude (WGS84), in meters (positive for up)
 float gps_hdop; ///< GPS HDOP horizontal dilution of position in meter. If unknown, set to: UINT16_MAX
 int8_t compass_use; ///< Compass use for heading
 int8_t gps_status; ///< GPS fix type or status. See ARLOBOT_GPS_STATUS enumeration.
 int8_t satellites_visible; ///< Number of satellites visible.
} mavlink_arlobot_apm_data_t;

#define MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN 23
#define MAVLINK_MSG_ID_221_LEN 23

#define MAVLINK_MSG_ID_ARLOBOT_APM_DATA_CRC 188
#define MAVLINK_MSG_ID_221_CRC 188



#define MAVLINK_MESSAGE_INFO_ARLOBOT_APM_DATA { \
	"ARLOBOT_APM_DATA", \
	8, \
	{  { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_arlobot_apm_data_t, heading) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_arlobot_apm_data_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_arlobot_apm_data_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_arlobot_apm_data_t, gps_alt) }, \
         { "gps_hdop", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_arlobot_apm_data_t, gps_hdop) }, \
         { "compass_use", NULL, MAVLINK_TYPE_INT8_T, 0, 20, offsetof(mavlink_arlobot_apm_data_t, compass_use) }, \
         { "gps_status", NULL, MAVLINK_TYPE_INT8_T, 0, 21, offsetof(mavlink_arlobot_apm_data_t, gps_status) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_INT8_T, 0, 22, offsetof(mavlink_arlobot_apm_data_t, satellites_visible) }, \
         } \
}


/**
 * @brief Pack a arlobot_apm_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param heading Heading from gyro and/or compass, 0~360 degree, CCW+
 * @param compass_use Compass use for heading
 * @param gps_lat Latitude (WGS84), in degrees
 * @param gps_lon Longitude (WGS84), in degrees
 * @param gps_alt Altitude (WGS84), in meters (positive for up)
 * @param gps_hdop GPS HDOP horizontal dilution of position in meter. If unknown, set to: UINT16_MAX
 * @param gps_status GPS fix type or status. See ARLOBOT_GPS_STATUS enumeration.
 * @param satellites_visible Number of satellites visible.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_apm_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float heading, int8_t compass_use, float gps_lat, float gps_lon, float gps_alt, float gps_hdop, int8_t gps_status, int8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN];
	_mav_put_float(buf, 0, heading);
	_mav_put_float(buf, 4, gps_lat);
	_mav_put_float(buf, 8, gps_lon);
	_mav_put_float(buf, 12, gps_alt);
	_mav_put_float(buf, 16, gps_hdop);
	_mav_put_int8_t(buf, 20, compass_use);
	_mav_put_int8_t(buf, 21, gps_status);
	_mav_put_int8_t(buf, 22, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#else
	mavlink_arlobot_apm_data_t packet;
	packet.heading = heading;
	packet.gps_lat = gps_lat;
	packet.gps_lon = gps_lon;
	packet.gps_alt = gps_alt;
	packet.gps_hdop = gps_hdop;
	packet.compass_use = compass_use;
	packet.gps_status = gps_status;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_APM_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif
}

/**
 * @brief Pack a arlobot_apm_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heading Heading from gyro and/or compass, 0~360 degree, CCW+
 * @param compass_use Compass use for heading
 * @param gps_lat Latitude (WGS84), in degrees
 * @param gps_lon Longitude (WGS84), in degrees
 * @param gps_alt Altitude (WGS84), in meters (positive for up)
 * @param gps_hdop GPS HDOP horizontal dilution of position in meter. If unknown, set to: UINT16_MAX
 * @param gps_status GPS fix type or status. See ARLOBOT_GPS_STATUS enumeration.
 * @param satellites_visible Number of satellites visible.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arlobot_apm_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float heading,int8_t compass_use,float gps_lat,float gps_lon,float gps_alt,float gps_hdop,int8_t gps_status,int8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN];
	_mav_put_float(buf, 0, heading);
	_mav_put_float(buf, 4, gps_lat);
	_mav_put_float(buf, 8, gps_lon);
	_mav_put_float(buf, 12, gps_alt);
	_mav_put_float(buf, 16, gps_hdop);
	_mav_put_int8_t(buf, 20, compass_use);
	_mav_put_int8_t(buf, 21, gps_status);
	_mav_put_int8_t(buf, 22, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#else
	mavlink_arlobot_apm_data_t packet;
	packet.heading = heading;
	packet.gps_lat = gps_lat;
	packet.gps_lon = gps_lon;
	packet.gps_alt = gps_alt;
	packet.gps_hdop = gps_hdop;
	packet.compass_use = compass_use;
	packet.gps_status = gps_status;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARLOBOT_APM_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif
}

/**
 * @brief Encode a arlobot_apm_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_apm_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_apm_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arlobot_apm_data_t* arlobot_apm_data)
{
	return mavlink_msg_arlobot_apm_data_pack(system_id, component_id, msg, arlobot_apm_data->heading, arlobot_apm_data->compass_use, arlobot_apm_data->gps_lat, arlobot_apm_data->gps_lon, arlobot_apm_data->gps_alt, arlobot_apm_data->gps_hdop, arlobot_apm_data->gps_status, arlobot_apm_data->satellites_visible);
}

/**
 * @brief Encode a arlobot_apm_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arlobot_apm_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arlobot_apm_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arlobot_apm_data_t* arlobot_apm_data)
{
	return mavlink_msg_arlobot_apm_data_pack_chan(system_id, component_id, chan, msg, arlobot_apm_data->heading, arlobot_apm_data->compass_use, arlobot_apm_data->gps_lat, arlobot_apm_data->gps_lon, arlobot_apm_data->gps_alt, arlobot_apm_data->gps_hdop, arlobot_apm_data->gps_status, arlobot_apm_data->satellites_visible);
}

/**
 * @brief Send a arlobot_apm_data message
 * @param chan MAVLink channel to send the message
 *
 * @param heading Heading from gyro and/or compass, 0~360 degree, CCW+
 * @param compass_use Compass use for heading
 * @param gps_lat Latitude (WGS84), in degrees
 * @param gps_lon Longitude (WGS84), in degrees
 * @param gps_alt Altitude (WGS84), in meters (positive for up)
 * @param gps_hdop GPS HDOP horizontal dilution of position in meter. If unknown, set to: UINT16_MAX
 * @param gps_status GPS fix type or status. See ARLOBOT_GPS_STATUS enumeration.
 * @param satellites_visible Number of satellites visible.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arlobot_apm_data_send(mavlink_channel_t chan, float heading, int8_t compass_use, float gps_lat, float gps_lon, float gps_alt, float gps_hdop, int8_t gps_status, int8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN];
	_mav_put_float(buf, 0, heading);
	_mav_put_float(buf, 4, gps_lat);
	_mav_put_float(buf, 8, gps_lon);
	_mav_put_float(buf, 12, gps_alt);
	_mav_put_float(buf, 16, gps_hdop);
	_mav_put_int8_t(buf, 20, compass_use);
	_mav_put_int8_t(buf, 21, gps_status);
	_mav_put_int8_t(buf, 22, satellites_visible);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif
#else
	mavlink_arlobot_apm_data_t packet;
	packet.heading = heading;
	packet.gps_lat = gps_lat;
	packet.gps_lon = gps_lon;
	packet.gps_alt = gps_alt;
	packet.gps_hdop = gps_hdop;
	packet.compass_use = compass_use;
	packet.gps_status = gps_status;
	packet.satellites_visible = satellites_visible;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arlobot_apm_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float heading, int8_t compass_use, float gps_lat, float gps_lon, float gps_alt, float gps_hdop, int8_t gps_status, int8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, heading);
	_mav_put_float(buf, 4, gps_lat);
	_mav_put_float(buf, 8, gps_lon);
	_mav_put_float(buf, 12, gps_alt);
	_mav_put_float(buf, 16, gps_hdop);
	_mav_put_int8_t(buf, 20, compass_use);
	_mav_put_int8_t(buf, 21, gps_status);
	_mav_put_int8_t(buf, 22, satellites_visible);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA, buf, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif
#else
	mavlink_arlobot_apm_data_t *packet = (mavlink_arlobot_apm_data_t *)msgbuf;
	packet->heading = heading;
	packet->gps_lat = gps_lat;
	packet->gps_lon = gps_lon;
	packet->gps_alt = gps_alt;
	packet->gps_hdop = gps_hdop;
	packet->compass_use = compass_use;
	packet->gps_status = gps_status;
	packet->satellites_visible = satellites_visible;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARLOBOT_APM_DATA, (const char *)packet, MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARLOBOT_APM_DATA UNPACKING


/**
 * @brief Get field heading from arlobot_apm_data message
 *
 * @return Heading from gyro and/or compass, 0~360 degree, CCW+
 */
static inline float mavlink_msg_arlobot_apm_data_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field compass_use from arlobot_apm_data message
 *
 * @return Compass use for heading
 */
static inline int8_t mavlink_msg_arlobot_apm_data_get_compass_use(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  20);
}

/**
 * @brief Get field gps_lat from arlobot_apm_data message
 *
 * @return Latitude (WGS84), in degrees
 */
static inline float mavlink_msg_arlobot_apm_data_get_gps_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field gps_lon from arlobot_apm_data message
 *
 * @return Longitude (WGS84), in degrees
 */
static inline float mavlink_msg_arlobot_apm_data_get_gps_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gps_alt from arlobot_apm_data message
 *
 * @return Altitude (WGS84), in meters (positive for up)
 */
static inline float mavlink_msg_arlobot_apm_data_get_gps_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gps_hdop from arlobot_apm_data message
 *
 * @return GPS HDOP horizontal dilution of position in meter. If unknown, set to: UINT16_MAX
 */
static inline float mavlink_msg_arlobot_apm_data_get_gps_hdop(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gps_status from arlobot_apm_data message
 *
 * @return GPS fix type or status. See ARLOBOT_GPS_STATUS enumeration.
 */
static inline int8_t mavlink_msg_arlobot_apm_data_get_gps_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  21);
}

/**
 * @brief Get field satellites_visible from arlobot_apm_data message
 *
 * @return Number of satellites visible.
 */
static inline int8_t mavlink_msg_arlobot_apm_data_get_satellites_visible(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  22);
}

/**
 * @brief Decode a arlobot_apm_data message into a struct
 *
 * @param msg The message to decode
 * @param arlobot_apm_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_arlobot_apm_data_decode(const mavlink_message_t* msg, mavlink_arlobot_apm_data_t* arlobot_apm_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	arlobot_apm_data->heading = mavlink_msg_arlobot_apm_data_get_heading(msg);
	arlobot_apm_data->gps_lat = mavlink_msg_arlobot_apm_data_get_gps_lat(msg);
	arlobot_apm_data->gps_lon = mavlink_msg_arlobot_apm_data_get_gps_lon(msg);
	arlobot_apm_data->gps_alt = mavlink_msg_arlobot_apm_data_get_gps_alt(msg);
	arlobot_apm_data->gps_hdop = mavlink_msg_arlobot_apm_data_get_gps_hdop(msg);
	arlobot_apm_data->compass_use = mavlink_msg_arlobot_apm_data_get_compass_use(msg);
	arlobot_apm_data->gps_status = mavlink_msg_arlobot_apm_data_get_gps_status(msg);
	arlobot_apm_data->satellites_visible = mavlink_msg_arlobot_apm_data_get_satellites_visible(msg);
#else
	memcpy(arlobot_apm_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARLOBOT_APM_DATA_LEN);
#endif
}

/** @file
 *	@brief MAVLink comm protocol testsuite generated from Arlobot.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ARLOBOT_TESTSUITE_H
#define ARLOBOT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_Arlobot(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_Arlobot(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_arlobot_dist_sensor_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_arlobot_dist_sensor_data_t packet_in = {
		{ 17235, 17236, 17237, 17238, 17239, 17240 },{ 17859, 17860, 17861 }
    };
	mavlink_arlobot_dist_sensor_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.ping_distance, packet_in.ping_distance, sizeof(uint16_t)*6);
        	mav_array_memcpy(packet1.ir_distance, packet_in.ir_distance, sizeof(uint16_t)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_dist_sensor_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_arlobot_dist_sensor_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_dist_sensor_data_pack(system_id, component_id, &msg , packet1.ping_distance , packet1.ir_distance );
	mavlink_msg_arlobot_dist_sensor_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_dist_sensor_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ping_distance , packet1.ir_distance );
	mavlink_msg_arlobot_dist_sensor_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_arlobot_dist_sensor_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_dist_sensor_data_send(MAVLINK_COMM_1 , packet1.ping_distance , packet1.ir_distance );
	mavlink_msg_arlobot_dist_sensor_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_arlobot_dock_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_arlobot_dock_data_t packet_in = {
		{ 17235, 17236, 17237 },{ 151, 152, 153 }
    };
	mavlink_arlobot_dock_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.irv, packet_in.irv, sizeof(uint16_t)*3);
        	mav_array_memcpy(packet1.irs, packet_in.irs, sizeof(uint8_t)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_dock_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_arlobot_dock_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_dock_data_pack(system_id, component_id, &msg , packet1.irs , packet1.irv );
	mavlink_msg_arlobot_dock_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_dock_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.irs , packet1.irv );
	mavlink_msg_arlobot_dock_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_arlobot_dock_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_dock_data_send(MAVLINK_COMM_1 , packet1.irs , packet1.irv );
	mavlink_msg_arlobot_dock_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_arlobot_apm_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_arlobot_apm_data_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,65,132,199
    };
	mavlink_arlobot_apm_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.heading = packet_in.heading;
        	packet1.gps_lat = packet_in.gps_lat;
        	packet1.gps_lon = packet_in.gps_lon;
        	packet1.gps_alt = packet_in.gps_alt;
        	packet1.gps_hdop = packet_in.gps_hdop;
        	packet1.compass_use = packet_in.compass_use;
        	packet1.gps_status = packet_in.gps_status;
        	packet1.satellites_visible = packet_in.satellites_visible;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_apm_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_arlobot_apm_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_apm_data_pack(system_id, component_id, &msg , packet1.heading , packet1.compass_use , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.gps_hdop , packet1.gps_status , packet1.satellites_visible );
	mavlink_msg_arlobot_apm_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_apm_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.heading , packet1.compass_use , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.gps_hdop , packet1.gps_status , packet1.satellites_visible );
	mavlink_msg_arlobot_apm_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_arlobot_apm_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_apm_data_send(MAVLINK_COMM_1 , packet1.heading , packet1.compass_use , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.gps_hdop , packet1.gps_status , packet1.satellites_visible );
	mavlink_msg_arlobot_apm_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_arlobot_material_order(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_arlobot_material_order_t packet_in = {
		963497464
    };
	mavlink_arlobot_material_order_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.list = packet_in.list;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_material_order_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_arlobot_material_order_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_material_order_pack(system_id, component_id, &msg , packet1.list );
	mavlink_msg_arlobot_material_order_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_material_order_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.list );
	mavlink_msg_arlobot_material_order_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_arlobot_material_order_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_material_order_send(MAVLINK_COMM_1 , packet1.list );
	mavlink_msg_arlobot_material_order_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_arlobot_commander(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_arlobot_commander_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,18275
    };
	mavlink_arlobot_commander_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.param1 = packet_in.param1;
        	packet1.param2 = packet_in.param2;
        	packet1.param3 = packet_in.param3;
        	packet1.param4 = packet_in.param4;
        	packet1.param5 = packet_in.param5;
        	packet1.command = packet_in.command;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_commander_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_arlobot_commander_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_commander_pack(system_id, component_id, &msg , packet1.command , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 );
	mavlink_msg_arlobot_commander_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_commander_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 );
	mavlink_msg_arlobot_commander_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_arlobot_commander_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_commander_send(MAVLINK_COMM_1 , packet1.command , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 );
	mavlink_msg_arlobot_commander_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_arlobot_odometry_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_arlobot_odometry_data_t packet_in = {
		17.0,45.0,73.0,101.0,129.0
    };
	mavlink_arlobot_odometry_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.heading = packet_in.heading;
        	packet1.v = packet_in.v;
        	packet1.omega = packet_in.omega;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_odometry_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_arlobot_odometry_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_odometry_data_pack(system_id, component_id, &msg , packet1.x , packet1.y , packet1.heading , packet1.v , packet1.omega );
	mavlink_msg_arlobot_odometry_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_odometry_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.x , packet1.y , packet1.heading , packet1.v , packet1.omega );
	mavlink_msg_arlobot_odometry_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_arlobot_odometry_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_odometry_data_send(MAVLINK_COMM_1 , packet1.x , packet1.y , packet1.heading , packet1.v , packet1.omega );
	mavlink_msg_arlobot_odometry_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_arlobot_encoder_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_arlobot_encoder_data_t packet_in = {
		963497464,963497672,963497880,963498088,963498296,963498504
    };
	mavlink_arlobot_encoder_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.enc_vel_l = packet_in.enc_vel_l;
        	packet1.enc_vel_r = packet_in.enc_vel_r;
        	packet1.enc_ticks_l = packet_in.enc_ticks_l;
        	packet1.enc_ticks_r = packet_in.enc_ticks_r;
        	packet1.cmd_enc_vel_l = packet_in.cmd_enc_vel_l;
        	packet1.cmd_enc_vel_r = packet_in.cmd_enc_vel_r;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_encoder_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_arlobot_encoder_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_encoder_data_pack(system_id, component_id, &msg , packet1.enc_vel_l , packet1.enc_vel_r , packet1.enc_ticks_l , packet1.enc_ticks_r , packet1.cmd_enc_vel_l , packet1.cmd_enc_vel_r );
	mavlink_msg_arlobot_encoder_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_encoder_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.enc_vel_l , packet1.enc_vel_r , packet1.enc_ticks_l , packet1.enc_ticks_r , packet1.cmd_enc_vel_l , packet1.cmd_enc_vel_r );
	mavlink_msg_arlobot_encoder_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_arlobot_encoder_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_encoder_data_send(MAVLINK_COMM_1 , packet1.enc_vel_l , packet1.enc_vel_r , packet1.enc_ticks_l , packet1.enc_ticks_r , packet1.cmd_enc_vel_l , packet1.cmd_enc_vel_r );
	mavlink_msg_arlobot_encoder_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_arlobot_system_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_arlobot_system_data_t packet_in = {
		963497464,17443
    };
	mavlink_arlobot_system_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.enc_vel_limit = packet_in.enc_vel_limit;
        	packet1.state = packet_in.state;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_system_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_arlobot_system_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_system_data_pack(system_id, component_id, &msg , packet1.state , packet1.enc_vel_limit );
	mavlink_msg_arlobot_system_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_system_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.state , packet1.enc_vel_limit );
	mavlink_msg_arlobot_system_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_arlobot_system_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_arlobot_system_data_send(MAVLINK_COMM_1 , packet1.state , packet1.enc_vel_limit );
	mavlink_msg_arlobot_system_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_Arlobot(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_arlobot_dist_sensor_data(system_id, component_id, last_msg);
	mavlink_test_arlobot_dock_data(system_id, component_id, last_msg);
	mavlink_test_arlobot_apm_data(system_id, component_id, last_msg);
	mavlink_test_arlobot_material_order(system_id, component_id, last_msg);
	mavlink_test_arlobot_commander(system_id, component_id, last_msg);
	mavlink_test_arlobot_odometry_data(system_id, component_id, last_msg);
	mavlink_test_arlobot_encoder_data(system_id, component_id, last_msg);
	mavlink_test_arlobot_system_data(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ARLOBOT_TESTSUITE_H

/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <pthread.h>
#include "autopilot_interface.h"

using namespace GeographicLib;

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
                MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

    printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

void
set_takeoff_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask = (0x1000 | 0b110111000011);

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x   = x;
    sp.y   = y;
    sp.z   = z;

    printf("TAKEOFF POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

void
set_land_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask = (0x2000 | 0b110111000111);

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x   = x;
    sp.y   = y;
    sp.z   = z;

    printf("LAND POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}

/*
void
explore_adjust_attitude( message)
{

}
*/


void
Autopilot_Interface::
update_UAVs_state(DSRC_message_t message)
{
    // 自身的状态放在 read_messages(); 函数里更新

    int autopilot_id;

    multicopter_state_t  UAV_state;

    memset(&UAV_state, 0, sizeof(UAV_state));
    memcpy(&UAV_state, message.payload, sizeof(UAV_state));

    autopilot_id = UAV_state.autopilot_id;

    // id error
    if(autopilot_id < 0 || autopilot_id >= UAV_NUM)
        return;

    UAVs[autopilot_id] = UAV_state;

    // Latitude
/*    UAVs[autopilot_id].lat = get_lat(msg);

    // Longitude
    UAVs[autopilot_id].lon = get_lon(msg);

    // Altitude above ground
    UAVs[autopilot_id].relative_alt = get_relative_alt(msg);

    // fused GPS and accelerometers
    // Ground X Speed (Latitude, positive north) mm/s
    UAVs[autopilot_id].vx = get_vx(msg);

    // Ground Y Speed (Longitude, positive east) mm/s
    UAVs[autopilot_id].vy = get_vy(msg);

    // Ground Z Speed (Altitude, positive down)  mm/s
    UAVs[autopilot_id].vz = get_vz(msg);

    // Current heading in degrees, in compass units (0..360, 0=north)
    UAVs[autopilot_id].heading = get_heading(msg);

    // aeronautical frame, NED / north-east-down convention
    // X Acceleration    mm/s^2
    UAVs[autopilot_id].ax = get_ax(msg);

    // Y Acceleration    mm/s^2
    UAVs[autopilot_id].ay = get_ay(msg);

    UAVs[autopilot_id].az = get_az(msg);
*/
}

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

    exit_flag = false;

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

    socket_fd = -1;
    receiving_msg_type = -1;
    receiving_command = None_cmd;

	serial_port = serial_port_; // serial port management object


    receiving_status = 0;
    transmiting_status = 0;

    receive_tid = 0;
    transmit_tid = 0;

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
    current_local_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                    mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

                case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
                {
                    //printf("MAVLINK_MSG_ID_EXTENDED_SYS_STATUS\n");
                    mavlink_msg_extended_sys_state_decode(&message, &(current_messages.extended_sys_status));
                    current_messages.time_stamps.extended_sys_status = get_time_usec();
                    this_timestamps.extended_sys_status = current_messages.time_stamps.extended_sys_status;
                    break;
                }

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
                    this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;

                    UAVs[autopilot_id].lat = current_messages.global_position_int.lat;
                    UAVs[autopilot_id].lon = current_messages.global_position_int.lon;
                    UAVs[autopilot_id].relative_alt = current_messages.global_position_int.relative_alt;

                    UAVs[autopilot_id].vx = current_messages.global_position_int.vx;
                    UAVs[autopilot_id].vy = current_messages.global_position_int.vy;
                    UAVs[autopilot_id].vz = current_messages.global_position_int.vz;

                    UAVs[autopilot_id].heading = current_messages.global_position_int.hdg;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;

                    UAVs[autopilot_id].ax = (int16_t)(current_messages.highres_imu.xacc/100);
                    UAVs[autopilot_id].ay = (int16_t)(current_messages.highres_imu.yacc/100);
                    UAVs[autopilot_id].az = (int16_t)(current_messages.highres_imu.zacc/100);
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   receive user message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
receive_user_messages()
{

    DSRC_message_t message;

    //----------------------------------------------------------
    // receive message
    //----------------------------------------------------------

    // debug
    printf("Waiting for message ...\n");

    int len = recv(socket_fd, recv_buf, sizeof(recv_buf), 0);

    if(len == 0)
    {
        printf("connect colsed!\n");
        return;
    }
    else if(len < 0)
    {
        printf("recv failed with error: %d\n", errno);
        exit(EXIT_FAILURE);
    }
    else
    {

        explore_setpoint_message_t  explore_message;
        explore_adjust_message_t    adjust_message;

        memset(&message, 0, sizeof(message));
        memcpy(&message, recv_buf, sizeof(message));

        uint8_t msg_type = message.message_ID;
        int height;

        printf("messageID: %d\n", msg_type);
        switch(msg_type)
        {
        case Vehicle_UAV_msg:

            memset(&current_vehicle_UAV_message, 0, sizeof(current_vehicle_UAV_message));
            memcpy(&current_vehicle_UAV_message, message.payload, sizeof(current_vehicle_UAV_message));
            current_global_setpoint.lat = current_vehicle_UAV_message.lat;
            current_global_setpoint.lon = current_vehicle_UAV_message.lon;
            current_global_setpoint.relative_alt = current_vehicle_UAV_message.relative_alt;

            receiving_command = Vehicle_to_UAV_cmd;

            break;
        case platoon_takeoff_msg:

            height = message.payload[1];

            if(height > 30 || height < 0)
                return;
            // 默认起飞高度 2m
            if(height == 0)
                height = 2;

            current_local_setpoint.x = initial_local_position.x;
            current_local_setpoint.y = initial_local_position.y;
            current_local_setpoint.z = height;

            receiving_command = Platoon_cmd;
            receiving_msg_type = platoon_takeoff_msg;

            break;
        case platoon_start_msg:

            receiving_msg_type = platoon_start_msg;

            break;
        case platoon_pause_msg:

            receiving_msg_type = platoon_pause_msg;

            break;
        case platoon_land_msg:

            receiving_command = None_cmd;
            receiving_msg_type = platoon_land_msg;

            break;
        case explore_setpoint_msg:

            memset(&explore_message, 0, sizeof(explore_message));
            memcpy(&explore_message, message.payload, sizeof(explore_message));

            current_global_setpoint.lat = explore_message.lat;
            current_global_setpoint.lon = explore_message.lon;
            current_global_setpoint.relative_alt = explore_message.relative_alt;

            receiving_command = Explore_cmd;
            receiving_msg_type = explore_setpoint_msg;

            break;
        case explore_adjust_msg:

            memset(&adjust_message, 0, sizeof(adjust_message));
            memcpy(&adjust_message, message.payload, sizeof(adjust_message));

            current_local_position.x += adjust_message.offset_x;
            current_local_position.y += adjust_message.offset_y;
            current_local_position.z -= adjust_message.offset_z;

            current_local_position.yaw += adjust_message.offset_yaw;

            receiving_msg_type = explore_adjust_msg;
            break;
        case explore_return_msg:

            receiving_command = None_cmd;
            receiving_msg_type = explore_return_msg;

            break;
        case UAV_state_msg:

            update_UAVs_state(message);

            break;
        case square_trajectory_msg:

            receiving_command = SquareTrack_cmd;

            break;
        case square_puase_msg:

            receiving_msg_type = square_puase_msg;

            break;
        case square_continue_msg:

            receiving_msg_type = square_continue_msg;

            break;
        }

    }

}


// ------------------------------------------------------------------------------
//   transmit user message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
transmit_user_messages()
{

    DSRC_message_t transmit_message;
    multicopter_state_t msg;
    //----------------------------------------------------------
    // transmit message
    //----------------------------------------------------------

    transmit_message.message_ID = UAV_state_msg;

    msg.system_id = system_id;
    msg.autopilot_id = autopilot_id;
    msg.companion_id = companion_id;

    msg.base_mode = current_messages.heartbeat.base_mode;
    msg.landed_state = current_messages.extended_sys_status.landed_state;

    msg.lat = UAVs[autopilot_id].lat;
    msg.lon = UAVs[autopilot_id].lon;
    msg.relative_alt = UAVs[autopilot_id].relative_alt;

    msg.heading = UAVs[autopilot_id].heading;

    msg.vx = UAVs[autopilot_id].vx;
    msg.vy = UAVs[autopilot_id].vy;
    msg.vz = UAVs[autopilot_id].vz;

    msg.ax = UAVs[autopilot_id].ax;
    msg.ay = UAVs[autopilot_id].ay;
    msg.az = UAVs[autopilot_id].az;

    memset(&transmit_message, 0, sizeof(transmit_message));
    memcpy(transmit_message.payload, &msg, sizeof(msg));

    if(send(socket_fd, &transmit_message, sizeof(transmit_message), 0) < 0)
    {
        printf("send failed with error: %d\n", errno);
        exit(EXIT_FAILURE);
    }

}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
    mavlink_set_position_target_local_ned_t sp = current_local_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}

	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
/*	while ( not ( current_messages.time_stamps.local_position_ned &&
                  current_messages.time_stamps.attitude   &&
                  current_messages.time_stamps.global_position_int )  )*/
    while ( not ( current_messages.time_stamps.local_position_ned &&
                      current_messages.time_stamps.attitude  )  )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
    initial_local_position.x        = local_data.local_position_ned.x;
    initial_local_position.y        = local_data.local_position_ned.y;
    initial_local_position.z        = local_data.local_position_ned.z;
    initial_local_position.vx       = local_data.local_position_ned.vx;
    initial_local_position.vy       = local_data.local_position_ned.vy;
    initial_local_position.vz       = local_data.local_position_ned.vz;
    initial_local_position.yaw      = local_data.attitude.yaw;
    initial_local_position.yaw_rate = local_data.attitude.yawspeed;

    initial_global_position.lat     = local_data.global_position_int.lat;
    initial_global_position.lon     = local_data.global_position_int.lon;
    initial_global_position.relative_alt = local_data.global_position_int.relative_alt;

    printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_local_position.x, initial_local_position.y, initial_local_position.z);
    printf("INITIAL POSITION YAW = %.4f \n", initial_local_position.yaw);
    printf("INITIAL GLOBAL POSITION ALA = [ %d , %d , %d ] \n", initial_global_position.lat, initial_global_position.lon, initial_global_position.relative_alt);
	printf("\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz


    socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TCP_PORT);
    addr.sin_addr.s_addr = inet_addr(server_ip);

    socklen_t len = sizeof(addr);
    // receive message
    if(connect(socket_fd, (struct sockaddr *)&addr, len) < 0)
    {
        fprintf(stderr, "connect failed with error: %d\n", errno);
        exit(EXIT_FAILURE);
    }
    // debug
    printf("Socket connect succeed\n");

    result = pthread_create( &receive_tid, NULL, &start_autopilot_interface_receive_thread, this );
    if ( result ) throw result;
    printf("\n");

    // wait for it to be started
    while ( not receiving_status )
        usleep(100000); // 10Hz


    result = pthread_create( &transmit_tid, NULL, &start_autopilot_interface_transmit_thread, this );
    if ( result ) throw result;
    printf("\n");

    // wait for it to be started
    while ( not transmiting_status )
        usleep(100000); // 10Hz

	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;
    exit_flag = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

    // wait for exit
    pthread_join(receive_tid ,NULL);
    pthread_join(transmit_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

    // 关闭socket
    if(close(socket_fd) == -1)
    {
          fprintf(stderr, "close socket failed\n");
          exit(EXIT_FAILURE);
    }

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}

/*
 * receive user message
 *
 */
void
Autopilot_Interface::
start_receive_thread()
{
    // debug
    printf("Start receive thread\n");

    if ( not receiving_status == false)
    {
        fprintf(stderr,"Receive thread already running\n");
        return;
    }
    else
    {
        receive_thread();
        return;
    }
}

/*
 * transmit user message
 *
 */
void
Autopilot_Interface::
start_transmit_thread()
{
    // debug
    printf("Start transmit thread\n");

    if ( not transmiting_status == false)
    {
        fprintf(stderr,"Transmit thread already running\n");
        return;
    }
    else
    {
        transmit_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}

// ----------------------------------------------------------------------------------
// Arm or Disarm Helper Functions
// ----------------------------------------------------------------------------------
void
Autopilot_Interface::
arm_disarm_autopilot(int cmd)
{
    // Define the system type (see mavlink_types.h for list of possible types)
    mavlink_command_long_t command_long;
    command_long.param1 = cmd;  // 1 or 0 to arm or disarm
    command_long.param2 = 0;
    command_long.param3 = 0;
    command_long.param4 = 0;
    command_long.param5 = 0;
    command_long.param6 = 0;
    command_long.param7 = 0;
    command_long.target_system = system_id; // - not really sure why this has to be at a valu of 1
    command_long.target_component = autopilot_id;
    command_long.command = MAV_CMD_COMPONENT_ARM_DISARM; //indicates what type of command
    command_long.confirmation = 1;

    // Initialize the required buffers
    mavlink_message_t message;
    // mavlink_command_long_t msg;
//    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t mavlink_msg_cm_long;

    /* Pack the arm message */
    mavlink_msg_cm_long = mavlink_msg_command_long_encode(system_id, companion_id, &message, &command_long);
    /*Send the message (.write sends as bytes)*/
    int len = write_message(message);

    // check the write
    if(len <= 0)
        fprintf(stderr,"WARNING:could not send MAV_CMD_COMPONENT_ARM_DISARM \n");
    return;
}


// ----------------------------------------------------------------------------------
// Takeoff Helper Functions
// ----------------------------------------------------------------------------------
void
Autopilot_Interface::
takeoff_autopilot(float alt)
{

    // Define the system type (see mavlink_types.h for list of possible types)
    mavlink_command_long_t command_long;
    command_long.param1 = 0;  // 1 or 0 to arm or disarm
    command_long.param2 = 0;
    command_long.param3 = 0;
    command_long.param4 = 0;
    command_long.param5 = initial_global_position.lat;
    command_long.param6 = initial_global_position.lon;
    command_long.param7 = alt;
    command_long.target_system = system_id;// - not really sure why this has to be at a valu of 1
    command_long.target_component = autopilot_id;
    command_long.command = MAV_CMD_NAV_TAKEOFF;//indicates what type of command
    command_long.confirmation = 1;

    // Initialize the required buffers
    mavlink_message_t message;
    //mavlink_command_long_t msg;
//    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t mavlink_msg_cm_long;

    /* Pack the arm message */
    mavlink_msg_cm_long = mavlink_msg_command_long_encode(system_id, companion_id, &message, &command_long);
    /*Send the message (.write sends as bytes)*/
    int len = write_message(message);

    // check the write
    if(len <= 0)
        fprintf(stderr,"WARNING:could not send MAV_CMD_COMPONENT_ARM_DISARM \n");
    return;
}

// ----------------------------------------------------------------------------------
// Land  Helper Functions
// ----------------------------------------------------------------------------------
void
Autopilot_Interface::
land_autopilot(void)
{

    // Define the system type (see mavlink_types.h for list of possible types)
    mavlink_command_long_t command_long;
    command_long.param1 = 0;  // 1 or 0 to arm or disarm
    command_long.param2 = 0;
    command_long.param3 = 0;
    command_long.param4 = 0;
    command_long.param5 = 0;
    command_long.param6 = 0;
    command_long.param7 = 0;
    command_long.target_system = system_id;// - not really sure why this has to be at a valu of 1
    command_long.target_component = autopilot_id;
    command_long.command = MAV_CMD_NAV_LAND;//indicates what type of command
    command_long.confirmation = 1;

    // Initialize the required buffers
    mavlink_message_t message;
    //mavlink_command_long_t msg;
//    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t mavlink_msg_cm_long;

    /* Pack the land message */
    mavlink_msg_cm_long = mavlink_msg_command_long_encode(system_id, companion_id, &message, &command_long);

    disable_offboard_control();
    usleep(100);

    /*Send the message (.write sends as bytes)*/
    int len = write_message(message);

    /*Send the message (.write sends as bytes)*/
    write_message(message);

    // check the write
    if(len <= 0)
        fprintf(stderr,"WARNING:could not send MAV_CMD_COMPONENT_ARM_DISARM \n");
    return;
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
    current_local_setpoint = sp;

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
        write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}

/*
 * thread receiving explore position for explore task
 *
 */
void
Autopilot_Interface::
receive_thread()
{

    receiving_status = true;

    while ( ! time_to_exit )
    {
        receive_user_messages();
        usleep(100000); // reaceive batches at 10Hz
    }

    receiving_status = false;

    return;

}

/*
 * thread transmit
 *
 */
void
Autopilot_Interface::
transmit_thread()
{

    transmiting_status = true;

    while ( ! time_to_exit )
    {
        transmit_user_messages();
        usleep(100000); // transmit batches at 10Hz
    }

    transmiting_status = false;

    return;

}


// 还需要修改
void
Autopilot_Interface::
platoon_formation_controller()
{
    float deltT = 0.1;   // 更新速度设置为10Hz
    float kv = 1, kp = 1;
    float output_x, output_y;

    if(autopilot_id != 0)
    {
        output_x = kv*(UAVs[autopilot_id-1].lon - UAVs[autopilot_id].lon) +
                    kp*(UAVs[autopilot_id-1].vx - UAVs[autopilot_id].vx);
        output_y = kv*(UAVs[autopilot_id-1].lat - UAVs[autopilot_id].lat) +
                    kp*(UAVs[autopilot_id-1].vy - UAVs[autopilot_id].vy);
    }

    current_local_position.vx = current_local_position.vx + output_x * deltT;
    current_local_position.vy = current_local_position.vy + output_y * deltT;

    current_local_position.x = current_local_position.x + current_local_position.vx * deltT;
    current_local_position.y = current_local_position.y + current_local_position.vy * deltT;
}


void
Autopilot_Interface::
transfer_globalSP_to_localSP()
{

    double lat1 = double(initial_global_position.lat / 1E7);
    double lat2 = double(current_global_setpoint.lat / 1E7);
    double lon1 = double(initial_global_position.lon / 1E7);
    double lon2 = double(current_global_setpoint.lon / 1E7);

    double x;
    double y;
    double z;

    const Geodesic & geod = Geodesic::WGS84();

    geod.Inverse(lat1, lon1, lat2, lon1, x);
    geod.Inverse(lat1, lon1, lat1, lon2, y);
    z = double(initial_global_position.relative_alt - current_global_setpoint.relative_alt) / 10000;

    printf("set offset point = [x: %f, y: %f, z: %f]\n", x, y, z);

    if(lat2 < lat1) x = -x;
    if(lon2 < lon1) y = -y;

    current_local_position.x = x;
    current_local_position.y = y;
    current_local_position.z = z;

}

void
Autopilot_Interface::
wait_UAV_arrive(WaitType wait)
{
    const float position_error = 0.1;

    float x_error;
    float y_error;
    float z_error;
    uint8_t landed_state;

    switch(wait)
    {
    case takeoff:
        z_error = abs(current_messages.local_position_ned.z - current_local_setpoint.z);
        printf("z_error: %f\n", z_error);

        landed_state = current_messages.extended_sys_status.landed_state & 	MAV_LANDED_STATE_ON_GROUND;
        printf("land state： %d, ", landed_state);

        // loop until UAV arriv the set point
        while( z_error > position_error )
        {
            usleep(500000);  // sleep 500ms
            z_error = abs(current_messages.local_position_ned.z - current_local_setpoint.z);

            landed_state = current_messages.extended_sys_status.landed_state & 	MAV_LANDED_STATE_ON_GROUND;
            printf("land state： %d, ", landed_state);

            printf("z_error: %f, ", z_error);
            mavlink_local_position_ned_t pos = current_messages.local_position_ned;
            printf("CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", pos.x, pos.y, pos.z);

        }
        printf("take off complete\n");
        break;

    case setposition:
        x_error = abs(current_messages.local_position_ned.x - current_local_setpoint.x);
        y_error = abs(current_messages.local_position_ned.y - current_local_setpoint.y);
        z_error = abs(current_messages.local_position_ned.z - current_local_setpoint.z);

        printf("x_error: %f, ", x_error);

        printf("y_error: %f, ", y_error);

        printf("z_error: %f, ", z_error);

        while(  x_error > position_error || y_error > position_error || z_error > position_error )
        {
            usleep(500000); // sleep 500ms
            x_error = abs(current_messages.local_position_ned.x - current_local_setpoint.x);
            y_error = abs(current_messages.local_position_ned.y - current_local_setpoint.y);
            z_error = abs(current_messages.local_position_ned.z - current_local_setpoint.z);

            mavlink_local_position_ned_t pos = current_messages.local_position_ned;
            printf("CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", pos.x, pos.y, pos.z);
        }
        printf("arrive setpoint\n");
        break;

    case land:
        landed_state = current_messages.extended_sys_status.landed_state & 	MAV_LANDED_STATE_ON_GROUND;
        printf("land state： %d, ", landed_state);

        while( landed_state != MAV_LANDED_STATE_ON_GROUND )
        {
            usleep(500000);  // sleep 500ms
            landed_state = current_messages.extended_sys_status.landed_state & 	MAV_LANDED_STATE_ON_GROUND;
            printf("land state： %d, ", landed_state);

            mavlink_local_position_ned_t pos = current_messages.local_position_ned;
            printf("CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", pos.x, pos.y, pos.z);
        }
        printf("land\n");
        break;
    }
}

void
Autopilot_Interface::
collect_picture()
{
    sleep(3);
}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

/******* lower level read and write by UART ******/

void*
start_autopilot_interface_read_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_read_thread();

    // done!
    return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_write_thread();

    // done!
    return NULL;
}


/******* upper level receive and transmite *******/


void*
start_autopilot_interface_receive_thread(void *args)
{

    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    autopilot_interface->start_receive_thread();

    return NULL;

}

void*
start_autopilot_interface_transmit_thread(void *args)
{

    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    autopilot_interface->start_transmit_thread();

    return NULL;

}

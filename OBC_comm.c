/*
 * OBC_comm.c
 *
 *  Created on: 26-Apr-2015
 *      Author: nikhil
 */

#ifndef OBC_COMM_C_
#define OBC_COMM_C_

/**
 * @Warning	EDIT THIS FILE WHEN YOU ARE 200% SURE OF WHAT YOU ARE DOING!
 * @note	For more information on Mavlink please visit http://qgroundcontrol.org/mavlink/start
 * @brief	This file contain function related to intercommunication between Slave Processor and On-Board Computer
 */


#include "ch.h"
#include "hal.h"
#include "Setup.h"

#define USE_SD2_FOR_TELEMETRY	0				//IF 1  SD2 is used for sending telemetry output to GCS
												//if 0 SD2 is used for sending debug messages(you will need set this value to 0 and
												//uncomment the code inside debug function in Setup.c

//=================ADDITION FROM MAVLINK====================

#include "mavlink_types.h"

#ifndef MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#endif

static mavlink_system_t mavlink_system;

static void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
    	sdPut(&SDU1, ch );
    }
#if(USE_SD2_FOR_TELEMETRY == 1)
    if (chan == MAVLINK_COMM_1)
    {
//    	uart1_transmit(ch);
    	sdPut(&SD2, ch );
    }
#endif
}

#include "OS_PORT/ext/mavlink/v1.0/common/mavlink.h"

//==========================================================

#include "intercomm.h"
#include "main.h"
#include "Arducopter/params.h"

const float MILLIG_TO_MS2 = 9.80665f / 1000.0f;
const float MS2_TO_MILLIG = 1000.0f/9.80665f;
const float RAD_TO_MILLIRAD = 1000.0f;




/**
 * variables defined by atulya
 */



//Sends Hearthbeat to OBC
static void send_heart_beat(mavlink_channel_t chan){

	mavlink_msg_heartbeat_send(chan,
			2,
			0,
		sys_state.base_mode,
		MAV_MODE_FLAG_STABILIZE_ENABLED,
		sys_state.system_status);
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION!
 */
static void send_scaled_imu(mavlink_channel_t chan){

	mavlink_msg_scaled_imu_send(chan,
			  millis(),
			  (int16_t)(sens_imu.accel.x*MS2_TO_MILLIG),
			  (int16_t)(sens_imu.accel.y*MS2_TO_MILLIG),
			  (int16_t)(sens_imu.accel.z*MS2_TO_MILLIG),
			  (int16_t)(sens_imu.gyro.x*RAD_TO_MILLIRAD),
			  (int16_t)(sens_imu.gyro.y*RAD_TO_MILLIRAD),
			  (int16_t)(sens_imu.gyro.z*RAD_TO_MILLIRAD),
			  (int16_t)0,
			  (int16_t)0,
			  (int16_t)0);
}

//Sends attitude data to OBC
static void send_attitude(mavlink_channel_t chan){

	mavlink_msg_attitude_send(
			chan,
			millis(),
			sens_imu.attitude.x,
			sens_imu.attitude.y,
			sens_imu.attitude.z,
			sens_imu.gyro.x,
			sens_imu.gyro.y,
			sens_imu.gyro.z);
}

static void send_status(mavlink_channel_t chan){
	mavlink_msg_sys_status_send(
			chan,
			sys_state.onboard_control_sensors_present,
			sys_state.onboard_control_sensors_enabled,
			sys_state.onboard_control_sensors_health,
			sys_state.system_load,
			0,
			-1,
			-1,
			0,
			0,
			0,
			0,
			0,
			0);
}

static void send_debug_msg(mavlink_channel_t chan) {
	mavlink_msg_debug_vect_send(
			chan,
			debug_vec.name,
			millis(),
			debug_vec.vector.x,
			debug_vec.vector.y,
			debug_vec.vector.z
			);

}

//Sends gps-baro position data to OBC
static void send_gps(mavlink_channel_t chan){

	float psi = ahrs.attitude.z;
	if(ahrs.attitude.z < 0)
		psi = ahrs.attitude.z + 2*M_PI_F;

	mavlink_msg_global_position_int_send(
			chan,
			millis(),
			sens_gps.lat,						//sending out raw gps data as received from LLP
			sens_gps.lng,
			sens_gps.alt*CM_TO_MM,
			sens_gps.alt*CM_TO_MM,
			sens_gps.vel.x,
			sens_gps.vel.y,
			sens_gps.vel.z,
			(uint16_t)(psi*5729.57));
}

//Sends rc data to OBC
static void send_rc_in(mavlink_channel_t chan){
    mavlink_msg_rc_channels_raw_send(
    	chan,
        millis(),
        0, // port
        rc_in[0],								//sending out raw RC input data as received from LLP
		rc_in[1],
		rc_in[2],
		rc_in[3],
		rc_in[4],
		rc_in[5],
		rc_in[6],
		0,
        -1);
}

static void send_vfr_hud(mavlink_channel_t chan)
{
	mavlink_msg_vfr_hud_send(
			chan,
			0.0f,
			sys_state.ground_speed,
			sys_state.heading,
			sys_state.throttle,
			inav.position.z/100.0,
			inav.velocity.z/100.0
			);
}

static void send_override_outputs(mavlink_channel_t chan)
{
	mavlink_msg_servo_output_raw_send(
			chan,
			millis(),
			1,
			ic_rc_or_data.ic_rc.rc1,
			ic_rc_or_data.ic_rc.rc2,
			ic_rc_or_data.ic_rc.rc3,
			ic_rc_or_data.ic_rc.rc4,
			ic_rc_or_data.ic_rc.rc5,
			ic_rc_or_data.ic_rc.rc5,
			ic_rc_or_data.ic_rc.rc6,
			ic_rc_or_data.ic_rc.rc7
			);
}

static void send_local_position_ned(mavlink_channel_t chan)
{
	mavlink_msg_local_position_ned_send(
			chan,
			millis(),
			inav.position.x,
			inav.position.y,
			inav.position.z,
			inav.velocity.x,
			inav.velocity.y,
			inav.velocity.z);
}

static void send_nav_controller_output(mavlink_channel_t chan)
{
	//normal operation
//	mavlink_msg_nav_controller_output_send(
//			chan,
//			pos_control.roll_target,
//			pos_control.pitch_target,
//			sys_state.nav_bearing,
//			sys_state.target_bearing,
//			sys_state.wp_dist,
//			sys_state.alt_error,
//			sys_state.aspeed_error,
//			sys_state.xtrack_error
//			);
	//tuning position and vel
	mavlink_msg_nav_controller_output_send(
			chan,
			pos_control.pos_target.y,
			pos_control.pos_target.x,
			sys_state.nav_bearing,
			sys_state.target_bearing,
			sys_state.wp_dist,
			pos_control.pos_target.z,
			pos_control.vel_target.z,
			pos_control.vel_target.x
			);
	//tuning inertial_nav gains
#if(USE_BARO_NOT_SONAR == 1)
	float alt = sens_baro.depth;
#else
	float alt = sens_sonar.depth;
#endif
//	mavlink_msg_nav_controller_output_send(
//			chan,
//			inav.local_x_cm,
//			inav.local_y_cm,
//			sys_state.nav_bearing,
//			sys_state.target_bearing,
//			sys_state.wp_dist,
//			alt,
//			pos_control.vel_target.z,
//			pos_control.vel_target.x
//			);
	(void)alt;
}

//TODO: In release version of the code remove the sim_state and hil_state and replace them with something else
static void send_hil_state(mavlink_channel_t chan)
{
	mavlink_msg_hil_state_send(
			chan,
			millis(),
			wp_nav.waypoint.x,
			wp_nav.waypoint.y,
			wp_nav.waypoint.z,
			pos_control.vel_desired.x,
			pos_control.vel_desired.y,
			pos_control.vel_desired.z,
			pos_control.pos_target.x,
			pos_control.pos_target.y,
			pos_control.pos_target.z,
			pos_control.vel_target.x,
			pos_control.vel_target.y,
			pos_control.vel_target.z,
			pos_control.accel_target_filter_x.output,
			pos_control.accel_target_filter_y.output,
			pos_control.accel_target.z
			);
}

/**
 * changes made by atulya
 */
static void send_sim_state(mavlink_channel_t chan)
{
	//tuning inertial_nav gains
#if(USE_BARO_NOT_SONAR == 1)
	float alt = sens_baro.depth;
#else
	float alt = sens_sonar.depth;
#endif
	mavlink_msg_sim_state_send(
			chan,
			ic_rc_or_data.ic_rc.rc1,
			ic_rc_or_data.ic_rc.rc2,
			ic_rc_or_data.ic_rc.rc3,
			ic_rc_or_data.ic_rc.rc4,
			debug_vec.vector.x,
			sens_cv.flag_active,
			ahrs.attitude.z,
			ahrs.accel_ef.x*100,
			ahrs.accel_ef.y*100,
			-(ahrs.accel_ef.z*100 + GRAVITY_CMSS),
			debug_vec2.x,
			debug_vec2.y,
			debug_vec2.z,
			inav.local_x_cm,
			inav.local_y_cm,
			alt,
			0,
			0,
			inav.velocity.x,
			inav.velocity.y,
			inav.velocity.z
			);
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION!
 */

static WORKING_AREA(mavlinkSendThread, 8192);
static msg_t mavlinkSend(void *arg) {

  (void)arg;
  chRegSetThreadName("mavlinkSend");

  uint16_t hbt_cnt = 0;

  while (TRUE) {

	  //The 1Hz messages
	  if(hbt_cnt > 99)
	  {
		  palTogglePad(GPIOE, 1);
		  send_heart_beat(MAVLINK_COMM_0);
		  send_status(MAVLINK_COMM_0);
		  hbt_cnt = 0;
	  }

	  //Each of the following cases is run at 12.5Hz
	  //Messages which are present twice get published at 25Hz
	  uint8_t temp_iter = hbt_cnt%8;
	  switch(temp_iter) {
		  case 0:
			  send_attitude(MAVLINK_COMM_0);
			  send_scaled_imu(MAVLINK_COMM_0);
			  break;
		  case 1:
			  send_gps(MAVLINK_COMM_0);

			  break;
		  case 2:
			  send_scaled_imu(MAVLINK_COMM_0);
			  send_nav_controller_output(MAVLINK_COMM_0);
			  break;
		  case 3:
			  send_debug_msg(MAVLINK_COMM_0);
			  send_rc_in(MAVLINK_COMM_0);
			  break;
		  case 4:
			  send_attitude(MAVLINK_COMM_0);
			  send_scaled_imu(MAVLINK_COMM_0);
			  break;
		  case 5:
			  send_override_outputs(MAVLINK_COMM_0);
			  break;
		  case 6:
			  send_scaled_imu(MAVLINK_COMM_0);
			  send_nav_controller_output(MAVLINK_COMM_0);
			  break;
		  case 7:
			  send_vfr_hud(MAVLINK_COMM_0);
			  break;
		  default:
			  break;
	  }

#if(LOGGING_MODE == 1)

	//TODO change the sending rates after the debugging stage
	  send_local_position_ned(MAVLINK_COMM_0);
	  send_sim_state(MAVLINK_COMM_0);
	  send_hil_state(MAVLINK_COMM_0);

	  chThdSleep(US2ST(9500));
	  hbt_cnt++;
#endif

  }
  return 0;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION!
 */

static WORKING_AREA(mavlinkSendTelemetryThread, 8192);
static msg_t mavlinkTelemetrySend(void *arg) {

  (void)arg;
  chRegSetThreadName("mavlinkTelemetrySend");

  uint16_t telemetry_iter_count = 0;


  while (TRUE) {

	  //The 1 Hz messages
	  if(telemetry_iter_count > 24)
	  {
		  send_heart_beat(MAVLINK_COMM_1);
		  send_status(MAVLINK_COMM_1);
		  telemetry_iter_count = 0;
	  }

	  //The 12.5 Hz messages
	  if(telemetry_iter_count %2 == 1)
	  {
		  send_local_position_ned(MAVLINK_COMM_1);
		  send_nav_controller_output(MAVLINK_COMM_1);
	  }

	  //The 5 Hz messages
	  uint8_t temp_iter = telemetry_iter_count%5;
	  switch(temp_iter) {
		  case 0:
			  send_attitude(MAVLINK_COMM_1);
			  break;
		  case 1:
			  send_gps(MAVLINK_COMM_1);
			  send_vfr_hud(MAVLINK_COMM_1);
			  break;
		  case 2:
			  send_scaled_imu(MAVLINK_COMM_1);
			  send_debug_msg(MAVLINK_COMM_1);
			  break;
		  case 3:
			  send_rc_in(MAVLINK_COMM_1);
			  break;
		  case 4:
			  send_override_outputs(MAVLINK_COMM_1);
			  break;
		  default:
			  break;
	  }

	  telemetry_iter_count++;
	  //sleep for 40ms
	  chThdSleep(US2ST(40000));

  }
  return 0;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION!
 */
uint16_t comm_get_available(mavlink_channel_t chan){
	int16_t bytes = 0;
	   switch(chan) {
		case MAVLINK_COMM_0:
			bytes = chQSpaceI(&(&SDU1)->iqueue);
			break;
#if(USE_SD2_FOR_TELEMETRY == 1)
		case MAVLINK_COMM_1:
			bytes = chQSpaceI(&(&SD2)->iqueue);
			break;
#endif
		default:
			break;
	   }
	   return bytes;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION!
 */
uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    uint8_t data = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		data = sdGetTimeout(&SDU1, 2);
		break;
#if(USE_SD2_FOR_TELEMETRY == 1)
	case MAVLINK_COMM_1:
		data = sdGetTimeout(&SD2, 2);
		break;
#endif
	default:
		break;
	}
    return data;
}

//Function is necessory to work properly with mavros. If you define your param, then you can send it via this function
void send_params(mavlink_channel_t chan){
	mavlink_msg_param_value_send(
					 chan,
					 sysid_thismav,
					 SYSID,
					 MAVLINK_TYPE_UINT8_T,
					 PARAM_COUNT,
					 sysid_thismav_index);

	mavlink_msg_param_value_send(
					 chan,
					 sysid_sw_type,
					 SW_TYPE,
					 MAVLINK_TYPE_UINT8_T,
					 PARAM_COUNT,
					 sysid_sw_type_index);

	mavlink_msg_param_value_send(
					 chan,
					 sysid_mygcs,
					 MY_GCS,
					 MAVLINK_TYPE_UINT8_T,
					 PARAM_COUNT,
					 sysid_mygcs_index);
	FWparamQSend(chan);
}

//Function to handle incoming data from mavro. If you wish to add custom
//message, then add an id for corresponding message within switch to handle it
void handleMessage(mavlink_message_t* msg, mavlink_channel_t chan)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
    	send_params(chan);
    	break;
    }

    /* When a request os made for a specific parameter */
      case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
      {
    	  mavlink_param_request_read_t cmd;
          mavlink_msg_param_request_read_decode(msg, &cmd);

          /* local name buffer to enforce null-terminated string */
          char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
          strncpy(name, cmd.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
          /* enforce null termination */
          name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
          /* attempt to find parameter, set and send it */
          resendParamMavLink(chan, name, cmd.param_index);
          break;
      }

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
    	mavlink_msg_mission_count_send(chan, 1, 0, 0);
    	break;
    }
    case MAVLINK_MSG_ID_PARAM_SET:
    {
    	mavlink_param_set_t param_set;
    	mavlink_msg_param_set_decode(msg, &param_set);

    	/* local name buffer to enforce null-terminated string */
		char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
		strncpy(name, param_set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
		/* enforce null termination */
		name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
		/* attempt to find parameter, set and send it */
		FWupdateParamMavLink(chan, name, param_set.param_value);
		break;
    }
    /**
     * addition by atulya
     *
     * This msg Receives vision position estimates from OBC
     */
    case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
    {
    	mavlink_vision_position_estimate_t vision_position_inp;
    	mavlink_msg_vision_position_estimate_decode(msg, &vision_position_inp);

    	sens_cv.position.x = 100*vision_position_inp.x;				//M to CM
    	sens_cv.position.y = 100*vision_position_inp.y;
    	sens_cv.position.z = 100*vision_position_inp.z;
    	sens_cv.roll = vision_position_inp.roll;
    	sens_cv.pitch = vision_position_inp.pitch;
    	sens_cv.yaw = vision_position_inp.yaw;
    	sens_cv.stamp = millis();
    	sens_cv.obc_stamp = vision_position_inp.usec;

    	if(fabs(vision_position_inp.roll - 1) < EPSILON)				// signal sent to ensure CV is active
    		sens_cv.flag_active = 1;
    	else
    		sens_cv.flag_active = 0;

		sens_sonar.obc_stamp = vision_position_inp.usec;
		sens_sonar.stamp = sens_cv.stamp;
		sens_sonar.depth = -100*vision_position_inp.z;

    	break;
    }
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
    {
    	mavlink_set_position_target_local_ned_t local_position_target_ned;
    	mavlink_msg_set_position_target_local_ned_decode(msg, &local_position_target_ned);

		wp_nav.local_target_yaw = local_position_target_ned.yaw;

    	wp_nav.local_target_hbf.x = 100*local_position_target_ned.x;			//treat input as relative targets
    	wp_nav.local_target_hbf.y = 100*local_position_target_ned.y;			//treat input as relative targets
    	wp_nav.local_target_hbf.z =  (-100)*local_position_target_ned.z;		//treat input as relative targets

    	wp_nav.flag_waypoint_received = 1;
    	debug("Received waypoint [%.3f, %.3f, %.3f]", local_position_target_ned.x, local_position_target_ned.y, local_position_target_ned.z);

    	break;
    }

    default:
    	break;

    }
}

//Function to receive and update data from mavros
void OBC_update(void )
{
    mavlink_message_t msg_ch0, msg_ch1;
    mavlink_status_t status_ch0, status_ch1;
    status_ch0.packet_rx_drop_count = 0;
    status_ch1.packet_rx_drop_count = 0;
    uint16_t i;

    // Deal with channel 0
    uint16_t nbytes_ch0 = comm_get_available(MAVLINK_COMM_0);
    i  = 0;
    for (i = 0; i<nbytes_ch0; i++)
    {
        uint8_t c = comm_receive_ch(MAVLINK_COMM_0);

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg_ch0, &status_ch0))
        {
            handleMessage(&msg_ch0, MAVLINK_COMM_0);
        }
    }
#if(USE_SD2_FOR_TELEMETRY == 1)
    // Deal with channel 1
    uint16_t nbytes_ch1 = comm_get_available(MAVLINK_COMM_1);
    i = 0;
    for (i = 0; i<nbytes_ch1; i++)
    {
        uint8_t c = comm_receive_ch(MAVLINK_COMM_1);

        if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg_ch1, &status_ch1))
        {
            handleMessage(&msg_ch1, MAVLINK_COMM_1);
        }
    }
#endif
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION!
 */
static WORKING_AREA(mavlinkReceiveThread, 8192);
static msg_t mavlinkReceive(void *arg) {

  (void)arg;
  chRegSetThreadName("mavlinkReceive");

  while (TRUE) {

	  OBC_update();

	  delay(10);
  }
  return 0;
}

//Function to initiate OBC communication
void OBC_comm_init(void){

	mavlink_system.sysid = SYSID;

	chThdCreateStatic(mavlinkReceiveThread, sizeof(mavlinkReceiveThread), NORMALPRIO, mavlinkReceive, NULL);
	chThdCreateStatic(mavlinkSendThread, sizeof(mavlinkSendThread), NORMALPRIO, mavlinkSend, NULL);

#if(USE_SD2_FOR_TELEMETRY == 1)
	chThdCreateStatic(mavlinkSendTelemetryThread, sizeof(mavlinkSendTelemetryThread), LOWPRIO, mavlinkTelemetrySend, NULL);
#endif

}


#endif /* ODROID_COMM_C_ */

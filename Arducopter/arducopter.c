/*
 * arducopter.c
 *
 *  Created on: 28-Jan-2016
 *      Author: atulya
 */

#include "main.h"

AHRS ahrs;
Inertial_nav_data inav;
Position_Controller pos_control;
WP_Nav wp_nav;
SystemState sys_state;

void init_arducopter()
{
	// initialize states for controller
	initSystemState();
	initializePosController();
	resetController();
	initializeWPNav();
	initINAV();
	sys_state.system_status = MAV_STATE_STANDBY;
	dmc = FALSE;
}

void run_arducopter(uint32_t t_now)
{
	//start time of loop in 10us
	uint32_t start = chTimeNow();

	//check whether new data has arrived on imu
	if(sens_imu.stamp > inav.last_imu_stamp)
	{
		if(isIMUGlitching() == 0)
		{
			updateAHRS();
			updateINAV(sens_imu.stamp - inav.last_imu_stamp);
		}
		else
		{
			return;
		}
	}

	//CONDITION FOR RUNNING LOITER to be run only when the switch is pressed on for the HLP code
	if(isSlaveActive() == 1)
	{
		loiter_run(t_now);
		sys_state.system_status = MAV_STATE_ACTIVE;
	}
	else
	{
		sys_state.system_status = MAV_STATE_STANDBY;
		resetWaypoint();
		resetController();
	}

	checkArmingStatus(0.01f);
	if(sys_state.flag_arming == 1)
	{
		//wait for 1 second. Assuming data LLP reset for BARO is over within a second
//		delay(1000);//TODO
		resetINAV();						//need to reset the baro when arming
	}

	//calculate system load upto now
	uint32_t stop = chTimeNow();									//stop time in 10us(10 of microseconds)
	float duration = (stop-start)/0.1f;								//duration is in us
	sys_state.system_load = (uint16_t)(duration/10000.0*1000);		//load of the main code 0%:0 100%: 1000 (1 cycle is taken as 10ms)

	//update system state variables
	updateSystemState();

}

void initSystemState(void)
{
	sys_state.onboard_control_sensors_present = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
			MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE | MAV_SYS_STATUS_SENSOR_GPS |
			MAV_SYS_STATUS_SENSOR_LASER_POSITION | MAV_SYS_STATUS_SENSOR_VISION_POSITION |
			MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
			MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
			MAV_SYS_STATUS_AHRS;

	sys_state.onboard_control_sensors_enabled = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
			MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
			MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
			MAV_SYS_STATUS_AHRS;

	if(USE_BARO_NOT_SONAR == 1)
		sys_state.onboard_control_sensors_enabled = sys_state.onboard_control_sensors_enabled | MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
	else
		sys_state.onboard_control_sensors_enabled = sys_state.onboard_control_sensors_enabled | MAV_SYS_STATUS_SENSOR_LASER_POSITION;

	if(USE_GPS_NOT_CV == 1)
		sys_state.onboard_control_sensors_enabled = sys_state.onboard_control_sensors_enabled | MAV_SYS_STATUS_SENSOR_GPS;
	else
		sys_state.onboard_control_sensors_enabled = sys_state.onboard_control_sensors_enabled | MAV_SYS_STATUS_SENSOR_VISION_POSITION;

	sys_state.onboard_control_sensors_health = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
			MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
			MAV_SYS_STATUS_AHRS;
	sys_state.base_mode = MAV_MODE_GUIDED_DISARMED;
	sys_state.system_status = MAV_STATE_UNINIT;

	initializeLPF(&sys_state.channel7_filter, STICK_MIN, 0.8);

	strncpy(debug_vec.name, "uninit", 10);
	debug_vec.vector.x = 1.2;
	debug_vec.vector.y = -0.5;
	debug_vec.vector.z = 3.1;
}

void updateSystemState(void)
{
	uint32_t now = millis();

	Vector2f velxy;
	velxy = (Vector2f){inav.velocity.x, inav.velocity.y};

	//update the state for control
#if (USE_GPS_NOT_CV == 1)
	sys_state.onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
#else
	if(pos_control._flags.xy_control_to_pilot)
		sys_state.onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
	else
		sys_state.onboard_control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
#endif

#if (USE_BARO_NOT_SONAR == 1)
	sys_state.onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
#else
	sys_state.onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;

#endif

	//vfr_hud
	sys_state.ground_speed = normVec2f(velxy)/100.0f;
	sys_state.heading = (int16_t)(ahrs.attitude.z*RAD_TO_DEG);
	sys_state.throttle = (uint16_t)(pos_control.throttle_out - THROTTLE_MIN)/(THROTTLE_MAX - THROTTLE_MIN);

	//nav_controller_output
	sys_state.nav_bearing = (int16_t)(wp_nav.waypoint_yaw*RAD_TO_DEG);
	sys_state.target_bearing = 0.0f;
	Vector2f dist_error;
	dist_error = (Vector2f){pos_control.pos_target.x - inav.position.x, pos_control.pos_target.y - inav.position.y};
	sys_state.wp_dist = 0.0f;
	sys_state.alt_error = (pos_control.pos_target.z - inav.position.z)/100.0f;
	sys_state.xtrack_error = normVec2f(dist_error)/100.0f;
	sys_state.aspeed_error = 0.0f;

	//if AHRS has timed out update the status for it
	if((now - ahrs.stamp) > AP_INTERTIALNAV_GPS_TIMEOUT_MS)
		sys_state.onboard_control_sensors_health &= (~MAV_SYS_STATUS_AHRS);
	else
		sys_state.onboard_control_sensors_health |= MAV_SYS_STATUS_AHRS;

}

inline void checkArmingStatus(float dt)
{
	//CONDITION FOR MOTORS BEING ARMED(Note that these values may need to recalibrated in case remote is changed)
	if(rc_in[2] < (STICK_MIN + 80) && rc_in[3] > (STICK_MAX - 80)) {
		if(sys_state.arming_count < ARMING_TIME)			//pressed continuously for 1 sec @100Hz
			sys_state.arming_count += dt;
	}
	else {
		if(sys_state.arming_count > 0.0f)
			sys_state.arming_count -= dt;
	}

	//================check for disarming================
	if(rc_in[2] < (STICK_MIN + 80) && rc_in[3] < (STICK_MIN + 80)) {
		if(sys_state.disarming_count < ARMING_TIME)			//pressed continuously for 1 sec @100Hz
			sys_state.disarming_count += dt;
	}
	else {
		if(sys_state.disarming_count > 0.0f)
			sys_state.disarming_count -= dt;
	}

	//====assign flags based on the current status of the code=========
	if(sys_state.flag_armed == 0 && sys_state.arming_count >= ARMING_TIME)
	{
		sys_state.flag_armed = 1;
		sys_state.flag_arming = 1;
		sys_state.base_mode = MAV_MODE_GUIDED_ARMED;

	}
	else
		sys_state.flag_arming = 0;

	if(sys_state.flag_armed == 1 && sys_state.disarming_count >= ARMING_TIME)
	{
		sys_state.flag_armed = 0;
		sys_state.base_mode = MAV_MODE_GUIDED_DISARMED;

	}
}

inline uint8_t isSlaveActive(void)
{
	float chnl7_out = applyLPF(&sys_state.channel7_filter, rc_in[6], 0.01);
	debug_vec.vector.x = chnl7_out;
	//(Note that these values may need to recalibrated in case remote is changed)
	if(chnl7_out > (2157 + 917)/2)
		return 1;
	else
		return 0;
}

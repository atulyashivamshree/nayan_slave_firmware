/*
 * main.h
 *
 *  Created on: Aug 2, 2014
 *      Author: nikhil
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "ch.h"
#include "hal.h"
#include "Setup.h"
#include "intercomm.h"
#include "autopilot_math.h"
#include "inertial_nav.h"
#include "position_controller.h"
#include "wp_nav.h"
#include "OS_PORT/ext/mavlink/v1.0/common/mavlink.h"

#define ARMING_TIME 1.75f

/**
 * @brief stores the raw imu variables acceleration and the angular velocity
 */
typedef struct
{
	uint32_t stamp; /**> timestamp of the instant data was obtained #stamp.*/
	Vector3f accel_calib; /**> acceleration as measured by the accelerometer and calibrated #accel_calib.*/
	Vector3f gyro_calib; /**> angular velocity as measured in the body frame by imu #angular_velocity.*/
	Vector3f attitude; /**> attitude as measured in the body frame by imu #angular_velocity.*/
}Sensor_IMU;


/**
 * @brief stores the gps latitude and longitude data along with the fused altitude
 */
typedef struct
{
	uint32_t stamp;
	int32_t lat;
	int32_t lng;
	float alt;
}Sensor_GPS;

/**
 * @brief struct to acquire data from an external position sensor such as a vision based system
 */
typedef struct
{
	uint32_t stamp;
	uint64_t obc_stamp;
	Vector3f position;
	float yaw;
	uint8_t flag_active;
}Sensor_ExtPos;

typedef struct
{
	uint32_t stamp;
	uint64_t obc_stamp;
	float depth;
}Sensor_Depth;
/**
 * @brief stores the current attitude and the trigonometric values for future use
 */
typedef struct
{
	uint32_t stamp;
	Vector3f attitude;

	int32_t lat_home;
	int32_t lng_home;
	float alt_home;

	float sin_phi, cos_phi;
	float sin_theta, cos_theta;
	float sin_psi, cos_psi;

	Vector3f accel_ef;
}AHRS;

typedef struct
{
	uint32_t stamp;
	uint8_t base_mode;
	uint8_t mav_state;
	uint8_t flag_armed;
	uint8_t flag_arming;
	float arming_count;
	float disarming_count;
	LowPassFilter channel7_filter;

	//system_status
	uint8_t system_status;
	uint32_t onboard_control_sensors_present;
	uint32_t onboard_control_sensors_enabled;
	uint32_t onboard_control_sensors_health;
	uint16_t system_load;

	//vfr_hud
	float ground_speed;
	int16_t heading;
	uint16_t throttle;

	//nav_controller_output
	int16_t nav_bearing;
	int16_t target_bearing;
	uint16_t wp_dist;
	float alt_error;
	float aspeed_error;
	float xtrack_error;

}SystemState;

typedef struct
{
	char name[10];
	Vector3f vector;
}DebugVec;

extern Sensor_IMU sens_imu; /**< struct holding current imu variables #sens_imu.*/
extern Sensor_GPS sens_gps;
extern Sensor_Depth sens_baro;
extern Sensor_ExtPos sens_cv;
extern Sensor_Depth sens_sonar;

extern AHRS ahrs;
extern Inertial_nav_data inav; /**< data structure storing the inertial navigation crucial data #inav. */
extern Position_Controller pos_control;
extern WP_Nav wp_nav;
extern SystemState sys_state;

extern Vector3f velocity;
extern uint16_t rc_in[7];

//variables for debugging sent through sim_state
extern DebugVec debug_vec;

#endif /* MAIN_H_ */

/*
 * arducopter.h
 *
 *  Created on: 28-Jan-2016
 *      Author: atulya
 */

/**
 * @date 28-Jan-2016
 * @author Atulya Shivam Shree
 * @file arducopter.h
 * @brief Implements arducopter based INS and cascaded PID control
 */


#ifndef ARDUCOPTER_ARDUCOPTER_H
#define ARDUCOPTER_ARDUCOPTER_H

#include "autopilot_math.h"
#include "inertial_nav.h"
#include "position_controller.h"
#include "wp_nav.h"
#include "OS_PORT/ext/mavlink/v1.0/common/mavlink.h"

#define ARMING_TIME 1.5f			// time taken during arming of the system

/**
 * @brief stores the current attitude and trigonometric values for future use
 */
typedef struct
{
	uint32_t stamp; 				/**< current time stamp*/
	Vector3f attitude;				/**< IMU roll, pitch, yaw */

	int32_t lat_home;				/**< Latitude in deg*1e7 of home*/
	int32_t lng_home;				/**< Longitude in deg*1e7 of home*/

	float sin_phi, cos_phi;
	float sin_theta, cos_theta;
	float sin_psi, cos_psi;

	Vector3f accel_ef;
} AHRS;

/**
 * @brief stores variables which depict the current system state
 */
typedef struct
{
	//variables for mavlink_msg_heartbeat #0
	uint32_t stamp;					/**< current time stamp*/
	uint8_t base_mode;				/**< base mode for HEARTBEAT>base_mode */
	uint8_t mav_state;				/**< current mav state forHEARTBEAT */
	uint8_t system_status;			/**< indicates ACTIVE or STANDBY for mavlink HeartBeat message*/
	uint8_t flag_armed;				/**< flag to indicate system is armed*/
	uint8_t flag_arming;			/**< flag to indicate armed is complete */
	float arming_count;				/**< couting the time during which sticks are held for arming*/
	float disarming_count;			/**< couting the time during which sticks are held for arming*/

	//variables for mavlink_msg_sys_status #1
	uint32_t onboard_control_sensors_present;
	uint32_t onboard_control_sensors_enabled;
	uint32_t onboard_control_sensors_health;
	uint16_t system_load;

	//variables for mavlink_msg_vfr_hud #74
	float ground_speed;
	int16_t heading;
	uint16_t throttle;

	//variables for mavlink_msg_nav_controller_output #62
	int16_t nav_bearing;
	int16_t target_bearing;
	uint16_t wp_dist;
	float alt_error;
	float aspeed_error;
	float xtrack_error;

	LowPassFilter channel7_filter;	/**< an LPF on channel 7 out*/

} SystemState;

/**
 * @brief struct to hold a debug vector and publishing it
 */
typedef struct
{
	char name[10];
	Vector3f vector;
}DebugVec;


/**< data structure storing the ahrs data*/
extern AHRS ahrs;

/**< data structure storing the inertial navigation crucial data #inav. */
extern Inertial_nav_data inav;

/**< data structure storing the variables for position control */
extern Position_Controller pos_control;

/**< data structure for calling the position controller according to waypoint
 * following code */
extern WP_Nav wp_nav;

/**< data structure for holding all debug variables */
extern SystemState sys_state;

/**< data structure for checking debug values. This variable is published in
 * the funciton send_sim_state() inside OBC_comm.c */
extern Vector3f debug_vec2;

/**< another data structure for relaying debug data via telemetry or OBC */
extern DebugVec debug_vec;

/**
 * @brief initializes the main functions of arducopter based loiter code
 */
void initializeArducopter(void);

/**
 * @brief call to execute the main logic of controller based on Arducopter
 * @param t_now current timestamp in milliseconds from FCU clock
 */
void runArducopter(uint32_t t_now);

/**
 * @brief initializes all the variables pertaining to current system state
 */
void initializeSystemState(void);

/**
 * @brief updates the variables in the struct SystemState
 */
void updateSystemState(void);

/**
 * @brief checks and updates the arming/disarming state of the system
 *
 * if Throttle is at min and yaw at max positive for 1.5 sec system is armed
 * if Throttle is at min and yaw at min negative for 1.5 sec system is armed
 *
 * @param dt time interval at which the function is called
 */
void checkArmingStatus(float dt);

/**
 * @brief checks whether the Slave controller is active by comparing the value
 *  of RC channel7
 */
uint8_t isSlaveActive(void);

#endif /* ARDUCOPTER_ARDUCOPTER_H */

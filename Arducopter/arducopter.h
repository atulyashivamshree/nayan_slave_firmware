/*
 * arducopter.h
 *
 *  Created on: 28-Jan-2016
 *      Author: atulya
 */

#ifndef ARDUCOPTER_H_
#define ARDUCOPTER_H_

#include "autopilot_math.h"
#include "inertial_nav.h"
#include "position_controller.h"
#include "wp_nav.h"
#include "OS_PORT/ext/mavlink/v1.0/common/mavlink.h"

#define ARMING_TIME 1.5f

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

extern AHRS ahrs;
extern Inertial_nav_data inav; /**< data structure storing the inertial navigation crucial data #inav. */
extern Position_Controller pos_control;
extern WP_Nav wp_nav;
extern SystemState sys_state;

extern Vector3f debug_vec2;

/*
 * @brief initializes the main functions of arducopter based loiter code
 */
void init_arducopter(void);

void run_arducopter(uint32_t t_now);

void initSystemState(void);

void updateSystemState(void);

void checkArmingStatus(float dt);

uint8_t isSlaveActive(void);

#endif /* ARDUCOPTER_H_ */

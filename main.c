#include <stdlib.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "main.h"
#include "Setup.h"
#include "intercomm.h"
#include "odroid_comm.h"

/**
 * This variable contains velocity in centimeter per second
 */
Vector3f velocity;

/**
 * This variable contains radio control input values.
 */
uint16_t rc_in[7];

Sensor_IMU sens_imu;
uint32_t last_imu_stamp;
Sensor_GPS sens_gps;
uint32_t last_gps_stamp;
Sensor_Depth sens_baro;
uint32_t last_baro_stamp;
Sensor_ExtPos sens_cv;
Sensor_Depth sens_sonar;

AHRS ahrs;
Inertial_nav_data inav;
Position_Controller pos_control;
WP_Nav wp_nav;
SystemState sys_state;
DebugVec debug_vec;

int count_arming = 0;
uint8_t FLAG_ARMING = 0;

inline void checkArmingStatus(void)
{
	//CONDITION FOR MOTORS BEING ARMED(Note that these values may need to recalibrated in case remote is changed)
	if(rc_in[2] < (THROTTLE_MIN + 80) && rc_in[3] > (STICK_MAX - 80))
	{
		if(count_arming < ARMING_COUNT)			//pressed continuously for 1 sec @100Hz
			count_arming++;
	}
	else
	{
		if(count_arming > 0)
			count_arming--;
	}
}

int main(void)
{
	halInit();
	chSysInit();

	start_sys();

	delay(1000);

	odroid_comm_init();

	initSystemState();
	initializePosController();
	resetController();
	initializeWPNav();
	initINAV();
	sys_state.system_status = MAV_STATE_STANDBY;

	uint32_t start_arming, stop_arming;

	while(TRUE)
	{
		uint32_t start = chTimeNow();						//start time of loop in 10us

		if(sens_imu.stamp > last_imu_stamp)
		{
			if(isIMUGlitching() == 0)
			{
				updateAHRS();
				updateINAV(sens_imu.stamp - last_imu_stamp);
				last_imu_stamp = sens_imu.stamp;
			}
			else
			{
				delay(10);
				continue;
			}
		}

		//CONDITION FOR RUNNING LOITER to be run only when the switch is pressed on for the HLP code transfer
		//(Note that these values may need to recalibrated in case remote is changed)
		float chnl6_out = applyLPF(&wp_nav.channel6_filter, rc_in[6], 0.01);
		if(chnl6_out > (2000 + 917)/2)
		{
			loiter_run();
			sys_state.system_status = MAV_STATE_ACTIVE;
		}
		else
		{
			sys_state.system_status = MAV_STATE_STANDBY;
			resetWaypoint();
			resetController();
		}

		checkArmingStatus();

		if(count_arming == ARMING_COUNT && FLAG_ARMING == 0)
		{
			sys_state.base_mode = MAV_MODE_GUIDED_ARMED;
			FLAG_ARMING = 1;
			stop_arming = chTimeNow();
			float duration = (stop_arming-start_arming)/0.1f;
			debug("ARMING with count %d in time %f, resetting INAV", count_arming, duration);
			//wait for 1 second. Assuming data LLP reset for BARO is over within  second

			delay(1000);
			resetINAV();						//need to reset the baro when arming
		}
		else if( count_arming == 0)
		{
			FLAG_ARMING = 0;
			start_arming = chTimeNow();
		}

		uint32_t stop = chTimeNow();		//stop time in 10us
		float duration = (stop-start)/0.1f;
		sys_state.system_load = (uint16_t)(duration/10000.0*1000);				//load of the main code 0%:0 100%: 1000

		//update system state variables
		updateSystemState();

		delay(10);

	}
	return 0;
}

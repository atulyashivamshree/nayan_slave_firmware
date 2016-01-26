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
Sensor_GPS sens_gps;
Sensor_Depth sens_baro;
Sensor_ExtPos sens_cv;
Sensor_Depth sens_sonar;

AHRS ahrs;
Inertial_nav_data inav;
Position_Controller pos_control;
WP_Nav wp_nav;
SystemState sys_state;
DebugVec debug_vec;

int main(void)
{
	halInit();
	chSysInit();

	start_sys();

	delay(1000);

	odroid_comm_init();

	// initialize states for controller
	initSystemState();
	initializePosController();
	resetController();
	initializeWPNav();
	initINAV();
	sys_state.system_status = MAV_STATE_STANDBY;

	while(TRUE)
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
				delay(10);
				continue;
			}
		}

		//CONDITION FOR RUNNING LOITER to be run only when the switch is pressed on for the HLP code
		if(isSlaveActive() == TRUE)
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

		checkArmingStatus(0.01f);
		if(sys_state.flag_arming == 1)
		{
			//wait for 1 second. Assuming data LLP reset for BARO is over within a second
			delay(1000);
			resetINAV();						//need to reset the baro when arming
		}

		//calculate system load upto now
		uint32_t stop = chTimeNow();									//stop time in 10us(10 of microseconds)
		float duration = (stop-start)/0.1f;								//duration is in us
		sys_state.system_load = (uint16_t)(duration/10000.0*1000);		//load of the main code 0%:0 100%: 1000 (1 cycle is taken as 10ms)

		//update system state variables
		updateSystemState();

		delay(10);

	}
	return 0;
}

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

		debug_vec.vector.y = sys_state.channel7_filter.output;
		debug_vec.vector.z = sys_state.channel7_filter.alpha;

		checkArmingStatus(0.01f);
		if(sys_state.flag_arming == 1)
		{
			//wait for 1 second. Assuming data LLP reset for BARO is over within a second
			delay(1000);
			resetINAV();						//need to reset the baro when arming
		}

		//calculate system load upto now
		uint32_t stop = chTimeNow();			//stop time in 10us
		float duration = (stop-start)/0.1f;
		sys_state.system_load = (uint16_t)(duration/10000.0*1000);		//load of the main code 0%:0 100%: 1000

		//update system state variables
		updateSystemState();

		delay(10);

	}
	return 0;
}

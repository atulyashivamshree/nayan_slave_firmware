/**
 * @date 29-Sep-2015
 * @author Atulya Shivam Shree
 * @file inertial_nav.h
 * @brief Implements the basic inertial navigation system using a complementary filter
 */

#include "stdbool.h"
#include "APM_config.h"

#ifndef ARDUCOPTER_INERTIAL_NAV_H
#define ARDUCOPTER_INERTIAL_NAV_H

#define GRAVITY_MSS 					9.80665f
#define GRAVITY_CMSS 					100*GRAVITY_MSS
#define LATLON_TO_CM 					1.113195f

#define INERTIAL_NAV_DELTAT_MAX 		0.1

// #defines to control how often historical accel based positions are saved
// so they can later be compared to laggy gps readings
#define AP_INTERTIALNAV_GPS_TIMEOUT_MS              300     // timeout after which position error from GPS will fall to zero

#define COUNT_Z_DELAY_BARO							2
#define COUNT_Z_DELAY_SONAR							25
#define COUNT_DELAY_EKF								3
		// assuming a 150 ms delay for the baro data if the AHRS is called at 100Hz
#define GPS_RADIUS_CM								400
#define BARO_RADIUS_CM								250
#define SONAR_RADIUS_CM								450
#define CV_RADIUS_CM								400
#define MAX_ACCEL_CHANGE							8		// 80000cmss * 0.01
#define MIN_ACCEL_MEASURED							1
#define MAX_BODY_ACCEL								1000	//10mss

//============CONTROL VARIABLES FOR DIFFERENT ACTIONS==============
// WARNING: state estimation using CV and SONAR are not stable as of now. Switch to CV and SONAR with caution
#define USE_BARO_NOT_SONAR							0		//0 for sonar 1 for baro
#define USE_GPS_NOT_CV								0		//0 for cv 1 for GPS
#define LOGGING_MODE								1		//0 for normal operation 1 for LOGGING controller data
//=================================================================

#if (USE_BARO_NOT_SONAR == 1)
	#define AP_HISTORIC_Z_SIZE						COUNT_Z_DELAY_BARO
	#define AP_INTERTIALNAV_TC_Z    				1.0f // default time constant for complementary filter's Z axis
	#define POSTARGET_MAX_ALTITUDE					1000
	#define POSTARGET_MIN_ALTITUDE					-1000
#else
	#define AP_HISTORIC_Z_SIZE						40
//	#define ALT_DELAY								COUNT_Z_DELAY_SONAR
	#define ALT_DELAY								COUNT_DELAY_EKF
	#define AP_INTERTIALNAV_TC_Z    				1.0f // default time constant for complementary filter's Z axis
	#define POSTARGET_MAX_ALTITUDE					250
	#define POSTARGET_MIN_ALTITUDE					50

#endif

#if (USE_GPS_NOT_CV == 1)
	#define AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS   10
	#define AP_HISTORIC_XY_SIZE							5
	#define AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS  4       // must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y
	#define AP_INTERTIALNAV_TC_XY   					2.0f 	// default time constant for complementary filter's X & Y axis
#else
	#define AP_HISTORIC_XY_SIZE							40
	#define XY_DELAY									COUNT_DELAY_EKF
	#define AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS   1
	#define AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS  2       // must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y
	#define AP_INTERTIALNAV_TC_XY   					1.0f 	// default time constant for complementary filter's X & Y axis
#endif

/**
 * @brief implements basic variables required for inertial navigation
 */
typedef struct
{
	float local_x_cm ;			/**< IGNORE added by atulya*/
	float local_y_cm ;			/**< IGNORE added by atulya*/

	Vector3f position_base; 		/**< base position as calculated by the inertial sensors only #position_base. */
	Vector3f position_correction; 	/**< position_correction + position_base = position #position_correction. */
	Vector3f velocity; 				/**< velocity estimate as calculated by the inertial sensors only #velocity. */
	Vector3f position_error; 		/**< position error = (position - (position_base + position_correction)) #position_error. */
	Vector3f position; 				/**< position of the vehicle as calculated by the inertial navigation system #position. */
	Vector3f accel_correction_hbf; 	//bias in accelerometer readings

	uint32_t last_update;

	/******************************
	 ******** Filter gains ********
	 *****************************/
	float k1_xy, k2_xy, k3_xy; 		/**< gain parameters for the complimentary filter of the inertial navigation */
	float k1_z, k2_z, k3_z; 		/**< gain parameters for the complimentary filter of the inertial navigation on z */

	/******************************
	 ******** GPS Variables *******
	 *****************************/
	uint32_t gps_last_update; 		/**< timestamp of the last update to the external postion tracking system #gps_last_update.*/
	uint32_t gps_last; 				/**< timestamp of the last update to the gps data #gps_last.*/
	uint8_t flag_gps_glitching;		/**< set to 1 if we have just recovered from a glitch in the GPS*/
	float lon_to_cm_scaling;        /**< conversion of longitude to centimeters*/
	int32_t last_good_lat;			/**< last good latitude from GPS*/
	int32_t last_good_lng;			/**< last good longitude from GPS*/
	uint32_t last_good_gps_update;	/**< timestamp of the last good GPS data */

	/******************************
	 ******** Baro Variables ******
	 *****************************/
	uint32_t baro_last_update;		/**< timestamp when the altitude was last updated using baro*/
	uint32_t baro_last;				/**< timestamp of last baro reading*/
	uint8_t flag_baro_glitching;	/**< set to 1 if we have just recovered from a glitch in the Baro*/
	float last_good_baro;		/**< last good sltitude readings from baro */
	uint32_t last_good_baro_update; /**< timestamp of last good baro altitude*/

	/******************************
	 *******Sonar Variables *******
	 *****************************/
	uint32_t sonar_last_update;		/**< timestamp when altitude was last updated using sonar */
	uint32_t sonar_last;			/**< timestamp of last sonar reading*/
	uint8_t flag_sonar_glitching;	/**< set to 1 if we have just recovered from a glitch in the sonar*/
	float last_good_sonar;			/**< last good altitude readings from sonar */
	uint32_t last_good_sonar_update;/**< timestamp of last good sonar altitude*/

	/******************************
	 ******** CV Variables ********
	 *****************************/
	uint32_t cv_last_update;		/**< timestamp when position wdas last updated using CV */
	uint32_t cv_last;				/**< timestamp of last sonar reading*/
	uint8_t flag_cv_glitching;		/**< set to 1 if we have just recovered from a glitch in CV*/
	Vector3f last_good_cv;			/**< last good position readings from CV */
	uint32_t last_good_cv_update;	/**< timestamp of last good CV position update*/

	/******************************
	 ******** IMU Variables *******
	 *****************************/
	Vector3f last_good_imu_accel;	/**< last good accelerometer readings*/
	uint32_t last_good_imu_update;	/**< timestamp when inav was last updated using accel*/
	uint32_t last_imu_stamp;		/**< timestamp of the last good imu readin*/

	float time_constant_xy;			/**< PARAM time constant for the gain parameters #time_constant_xy.*/
	float time_constant_z;			/**< PARAM time constant for the gain parameters #time_constant_z.*/

	/**********************************************
	 *  Variables to store historic estimates calculated from the IMU
	 ***********************************************/
	uint8_t historic_xy_counter;			/**< counter for pushing elements to queue after n iterations*/
	float historic_x[AP_HISTORIC_XY_SIZE];	/**< queue for storing estiamtes of x*/
	float historic_y[AP_HISTORIC_XY_SIZE];	/**< queue for storing estimates of y*/
	Queue_property historic_x_property;		/**< storing property of x queue */
	Queue_property historic_y_property;		/**< storing property of y queue */
	uint8_t xy_delay;

	/**********************************************
	 *  Variables to store historic estimates calculated from the IMU
	 ***********************************************/
	uint8_t historic_z_counter;				/**< counter for pushing elements to queue after n iterations*/
	float historic_z[AP_HISTORIC_Z_SIZE];	/**< queue for storing estimates of z */
	Queue_property historic_z_property;		/**< storing property of z queue */
	uint8_t z_delay;

}Inertial_nav_data;

/**
 * @brief update the INS system on the basis of new data
 * @param del_t time difference between two calls to the update function in ms
 */
void updateINAV(uint32_t del_t);

/**
 * @brief initialize the variables for inertial navigation
 */
void initINAV(void);

/**
 * @brief resets the inertial nav code. Usually required while arming
 */
void resetINAV(void);

/**
 * @brief update the variables of the AHRS on the basis of new imu readings
 */
void updateAHRS(void);

/**
 * @brief automatically initialize the home position depending on the current position
 */
void initializeGPSHome(void);

/**
 * @brief initializes the altitude readings either using sonar or baro
 */

void initializeAlt(void);

/**
 * @brief sets the position variables to home
 */
void setupHomePositionCV(void);

/**
 * @brief update the gains of the complementary filter used to update the inertial navigation data
 */
void updateINAVGains(void);

/**
 * @brief update the gains of the filter individually for each of the variable
 */
void updateINAVGainsIndividually(float kp1, float kp2, float kp3);

/**
 * @brief sets the desired postion in the inertial navigation system units in cm
 */
void setPositionXY(float x, float y);

/**
 * @brief checks whether the IMU data is glitching
 */
int isIMUGlitching(void);

#endif /* ARDUCOPTER_INERTIAL_NAV_H */

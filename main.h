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
#include "Arducopter/arducopter.h"

/**
 * @brief stores the raw imu variables acceleration and the angular velocity
 */
typedef struct
{
	uint32_t stamp; /* System Time of the instant at which data was obtained in  milliseconds */
	Vector3f accel; /* calibrated acceleration from IMU in meter per seconds in Body Frame*/
	Vector3f gyro; /* calibrated angular velocities from IMU in radian per seconds in Body Frame*/
	Vector3f attitude; /* attitude as measured in the body frame by imu in radians in Body Frame*/
}Sensor_IMU;


/**
 * @brief stores the gps latitude and longitude data along with the fused altitude from Barometer
 */
typedef struct
{
	uint32_t stamp;/* System Time of the instant at which data was obtained in  milliseconds */
	int32_t lat;/* Latitude * 1E07 from GPS */
	int32_t lng;/* Longitude * 1E07 from GPS */
	float alt;/* Relative Altitude from Barometer in cm*/
	Vector3f vel; /* Velocities from GPS in cm per sec in NED frame*/
}Sensor_GPS;
/**
 * @brief struct to acquire data from an external position sensor such as a vision based system
 * @note This is being updated at handleMessage() in OBC_comm.c
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

extern Sensor_IMU sens_imu; /**< struct holding current imu variables #sens_imu.*/
extern Sensor_GPS sens_gps;
extern Sensor_Depth sens_baro;
extern Sensor_ExtPos sens_cv;
extern Sensor_Depth sens_sonar;

extern uint16_t rc_in[7];

extern bool_t dmc;

extern uint16_t control_command[4];

#endif /* MAIN_H_ */

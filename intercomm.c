/*
 * intercomm.cpp
 *
 *  Created on: 16-Apr-2015
 *      Author: nikhil
 */

#ifndef INTERCOMM_CPP_
#define INTERCOMM_CPP_

/**
 * @Warning DO NOT EDIT THIS FILE!
 * This file contain function related to intercommunication between Master and Slave processor
 */


#include "Setup.h"
#include "intercomm.h"
#include "config/def.h"
#include "main.h"

ic_imu_u ic_imu_data;

ic_rc_or_u ic_rc_or_data;

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */

void spi_exchange_data(SPIDriver *spip, uint8_t *tx, uint8_t *rx, size_t size) {

	spiAcquireBus(spip);
	spiSelect(spip);
	spiExchange(spip, size, tx, rx);
	spiUnselect(spip);
	spiReleaseBus(spip);
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */
bool_t check_ahrs_sanity(float _a){

	if(_a > 5 || _a < -5){
		return FALSE;
	}
	return TRUE;

}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */
bool_t check_acc_sanity(float _a){

	if(_a > 16 || _a < -16){
		return FALSE;
	}
	return TRUE;

}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */
void update_ic_data(void){

	uint32_t stamp = millis();
	sens_imu.stamp = stamp;
	sens_imu.accel_calib.x = ic_imu_data.ic_imu.ax;
	sens_imu.accel_calib.y = ic_imu_data.ic_imu.ay;
	sens_imu.accel_calib.z = ic_imu_data.ic_imu.az;

	sens_imu.gyro_calib.x = ic_imu_data.ic_imu.gx;
	sens_imu.gyro_calib.y = ic_imu_data.ic_imu.gy;
	sens_imu.gyro_calib.z = ic_imu_data.ic_imu.gz;

	ahrs.stamp = stamp;
	ahrs.attitude.x = ic_imu_data.ic_imu.roll;
	ahrs.attitude.y = ic_imu_data.ic_imu.pitch;
	ahrs.attitude.z = ic_imu_data.ic_imu.yaw;

	Vector3f _position_gps;
	_position_gps.x = ic_imu_data.ic_imu.lat;
	_position_gps.y = ic_imu_data.ic_imu.lng;
	_position_gps.z = ic_imu_data.ic_imu.alt;

	if(_position_gps.x != sens_gps.lat || _position_gps.y != sens_gps.lng || _position_gps.z != sens_gps.alt)
	{
		sens_gps.stamp = stamp;
		sens_gps.lat = _position_gps.x;
		sens_gps.lng = _position_gps.y;
		sens_gps.alt = _position_gps.z;
	}

	velocity.x = ic_imu_data.ic_imu.vx;
	velocity.y = ic_imu_data.ic_imu.vy;
	velocity.z = ic_imu_data.ic_imu.vz;

	rc_in[0] = ic_imu_data.ic_imu.rc_in_1;
	rc_in[1] = ic_imu_data.ic_imu.rc_in_2;
	rc_in[2] = ic_imu_data.ic_imu.rc_in_3;
	rc_in[3] = ic_imu_data.ic_imu.rc_in_4;
	rc_in[4] = ic_imu_data.ic_imu.rc_in_5;
	rc_in[5] = ic_imu_data.ic_imu.rc_in_6;
	rc_in[6] = ic_imu_data.ic_imu.rc_in_7;
}

#endif /* INTERCOMM_CPP_ */

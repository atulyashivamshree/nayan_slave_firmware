/*
 * params.c
 *
 *  Created on: 24-Oct-2015
 *      Author: atulya
 */
/*
 * param.c
 *
 *  Created on: 29-Dec-2013
 *      Author: nikhil9
 */

#include "main.h"
//#include "OS_PORT/ext/mavlink/v1.0/common/mavlink.h"

const char sysid_thismav[16] = "SYSID_THISMAV";
const char sysid_sw_type[16] = "SYSID_SW_TYPE";
const char sysid_mygcs[16] = "SYSID_MYGCS";

const char inav_tc_xy[16] = "INAV_TC_XY";
const char inav_tc_z[16] = "INAV_TC_Z";

const char pos_xy_p[16] = "POS_XY_P";
const char vel_xy_p[16] = "VEL_XY_P";
const char vel_xy_i[16] = "VEL_XY_I";
const char vel_xy_imax[16] = "VEL_XY_IMAX";

const char pos_z_p[16] = "POS_Z_P";
const char vel_z_p[16] = "VEL_Z_P";
const char accel_z_p[16] = "THR_ACCEL_P";
const char accel_z_i[16] = "THR_ACCEL_I";
const char accel_z_imax[16] = "ACCEL_Z_IMAX";
const char accel_z_d[16] = "ACCEL_Z_D";
const char accel_z_filt_hz[16] = "ACCEL_Z_FILT_HZ";

const char throttle_hover[16] = "THR_MID";
const char alt_delay[16] = "ALT_DELAY";
const char xy_delay[16] = "XY_DELAY";

const char k1_xy[16] = "K1_XY";
const char k2_xy[16] = "K2_XY";
const char k3_xy[16] = "K3_XY";
const char k1_z[16] = "K1_Z";
const char k2_z[16] = "K2_Z";
const char k3_z[16] = "K3_Z";

const uint8_t sysid_thismav_index = 0;
const uint8_t sysid_sw_type_index = 1;
const uint8_t sysid_mygcs_index = 2;

const uint8_t inav_tc_xy_index = 3;
const uint8_t inav_tc_z_index = 4;

const uint8_t pos_xy_p_index = 5;
const uint8_t vel_xy_p_index = 6;
const uint8_t vel_xy_i_index = 7;
const uint8_t vel_xy_imax_index = 8;

const uint8_t pos_z_p_index = 9;
const uint8_t vel_z_p_index = 10;
const uint8_t accel_z_p_index = 11;
const uint8_t accel_z_i_index = 12;
const uint8_t accel_z_d_index = 13;
const uint8_t accel_z_imax_index = 14;
const uint8_t accel_z_filt_hz_index = 15;
const uint8_t throttle_hover_index = 16;
const uint8_t alt_delay_index = 17;
const uint8_t xy_delay_index = 18;

const uint8_t k1_xy_index = 19;
const uint8_t k2_xy_index = 20;
const uint8_t k3_xy_index = 21;
const uint8_t k1_z_index = 22;
const uint8_t k2_z_index = 23;
const uint8_t k3_z_index = 24;
//void updatePIDFromEEPROM(void);	TODO FUTURE SCOPE
//void writeDefaultParam(void);		TODO FUTURE SCOPE

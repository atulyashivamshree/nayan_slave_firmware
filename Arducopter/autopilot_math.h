/*
 * autopilot_math.h
 *
 *  Created on: 02-Oct-2015
 *      Author: atulya
 */

/**
 * @date 02-Oct-2015
 * @author Atulya Shivam Shree
 * @file autopilot_math.h
 * @brief Implements the basic math functions used for developing inertial
 * navigation and a position controller
 */

#include "math.h"
#include "stdlib.h"
#include "stdint.h"

#ifndef ARDUCOPTER_AUTOPILOT_MATH_H_
#define ARDUCOPTER_AUTOPILOT_MATH_H_

#define M_PI_F 3.141592653589793f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define CM_TO_MM 10
#define EPSILON 1e-3F

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

/**
 * @brief implements a 2 dimensional vector of type float
 */
typedef struct
{
	float x;
	float y;
}Vector2f;

/**
 * @brief implements a 3 dimensional vector of type float
 */
typedef struct
{
	float x;
	float y;
	float z;
}Vector3f;

/**
 * @brief contains the basic elements that define a queue
 */
typedef struct
{
	uint8_t first;			/**< stores the index of first element of queue*/
	uint8_t last;			/**< stores the index of last element of queue*/
	uint8_t size;			/**< stores the size of the buffer queue*/
	uint8_t counter;		/**< a counter for number of variables stored in the queue*/
	uint8_t is_full;		/**< a flag for storing whether the buffer is full*/
	uint8_t is_empty;		/**< a flag for storing whether the buffer is empty*/
}Queue_property;

/**
 * @brief contains the basic elements for an LPF
 */
typedef struct
{
	float cutoff_freq;		/**< cutoff frequency in hz */
	float sampling_freq;	/**< sampling frequency in hz */
	float alpha;			/**< an intermediary value for computation*/

	float output;			/**< system state on which lpf is applied */
}LowPassFilter;

/**
 * @brief a structure for holding states of a P controller
 */
typedef struct
{
	float kP;
}Controller_P;

/**
 * @brief structure for holding states of a Vector2f PI controller
 */
typedef struct
{
	float kP;					/**< Gain kp*/
	float kI;					/**< Gain kI*/
	float Imax;					/**< Maximum value integrator can take */
	float filt_hz;				/**< cutoff frequency for filter on input */
	uint8_t reset_filter;		/**< a flag to indicate that filter is to be reset*/

	float dt;					/**< sampling time of the controller*/
	Vector2f input;				/**< input in the form of a 2D vector*/
	Vector2f integrator;		/**< storing the value of integrator*/
	float filt_alpha;			/**< a constant term used in LPF*/

}Controller_PI_2D;

/**
 * @brief structure for holding states of a Vector3f PID controller
 */
typedef struct
{
	float kP;					/**< Gain kp*/
	float kI;					/**< Gain ki*/
	float kD;					/**< Gain kd*/
	float Imax;					/**< Max value of integrator*/
	float filt_hz;				/**< cutoff frequency for filter on input*/
	uint8_t reset_filter;		/**< flag to be used in case of a reset */

	float dt;					/**< sampling time of the controller*/
	float input;				/**< input value*/
	float integrator;			/**< storing the value of integrator*/
	float derivative;			/**< storing the value of derivative*/
	float filt_alpha;			/**< a constant term used in LPF*/

}Controller_PID;

// 2D vector length
static inline float pythagorous2(float a, float b) {
	return sqrt(a*a+b*b);
}
/**
 * @brief returns sqrt of value
 *
 * a varient of sqrt() that checks the input ranges and ensures a
 * valid value as output. If a negative number is given then 0 is
 * returned. The reasoning is that a negative number for sqrt() in our
 * code is usually caused by small numerical rounding errors, so the
 * real input should have been zero
 */

static inline float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

/**
 * @brief constrain a value within bounds
 */
static inline float constrain_float(float amt, float low, float high)
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**
 * @brief constrain a value within bounds
 */
static inline int constrain_int(int amt, int low, int high)
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**
 * @brief return the L2 norm of a 3D vector
 */
static inline float normVec3f(Vector3f inp)
{
	return sqrt(pow(inp.x,2)+ pow(inp.y, 2)+ pow(inp.z, 2));
}

/**
 * @brief return the L2 norm of a 2D vector
 */
static inline float normVec2f(Vector2f inp)
{
	return sqrt(pow(inp.x,2)+ pow(inp.y, 2));
}
/**
 * @brief sets a vector to [0, 0, 0]
 */
void initializeVector3fToZero(Vector3f* inp);

/**
 * @brief sets a vector to [0, 0]
 */
void initializeVector2fToZero(Vector2f* inp);

/**
 * @brief pops the first element of the queue and returns it
 * @param arr the array storing all the elements of the queue
 * @param q_property stores the current state of the queue
 */
void resetQueue(float arr[], Queue_property *q_property);

/**
 * @brief pops the first element of the queue and returns it
 * @param arr the array storing all the elements of the queue
 * @param q_property stores the current state of the queue
 */
float popQueue(float arr[], Queue_property *q_property);

/**
 * @brief pushes an element into the queue
 * @param data element to be pushed into the array
 * @param arr the array storing all the elements of the queue
 * @param q_property stores the current state of the queue
 */
void pushToQueue(float data, float arr[], Queue_property *q_property);

////////////----Low Pass Filter-------/////////////
// intialize the state and cutoff frequency of the lpf
void initializeLPF(LowPassFilter *lpf, float outp, float frequency);

// reset the state of the LPF to the current value
void resetLPF(LowPassFilter *lpf, float val);

// apply the low pass complimentary filter on the output
float applyLPF(LowPassFilter *lpf, float input, float dt);
////------------------------------------------////

////////////////CONTROLLER LIBRARIES/////////////////////

/////////////------------PI-----------////////////

void initializePI(Controller_PI_2D *pi, float kP, float kI, float imax, float filt_hz, float dt);
void setPIInput(Controller_PI_2D *pi, Vector2f input, float dt);
void resetPI_I(Controller_PI_2D *pi);
Vector2f getPI_P(Controller_PI_2D *pi);
Vector2f getPI_I(Controller_PI_2D *pi);
Vector2f getPI_I_shrink(Controller_PI_2D *pi);

////////////////////////PID///////////////////////

void initializePID(Controller_PID *pid, float kP, float kI, float kD, float imax, float filt_hz, float dt);
void setPIDInput_FilterAll(Controller_PID *pid, float input);
void setPIDInput_FilterD(Controller_PID *pid, float input);
void resetPID_I(Controller_PID *pid);
float getPID_P(Controller_PID *pid);
float getPID_I(Controller_PID *pid);
float getPID_D(Controller_PID *pid);

//////////////////////////////////////////////////

#endif /* ARDUCOPTER_AUTOPILOT_MATH_H_ */

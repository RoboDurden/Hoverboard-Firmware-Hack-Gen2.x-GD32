#include <stdint.h>
#include "../Inc/bldc.h"

// PID_Gemini	might work better with lower rank bldc_outputFilterPwm like 10

/*
I have given up on ChatGpt :-(
What a waste of time, noch ai could provide pid code that reduces error to 0 to reach target speed
in the time wasted on stupid ai, i could have coded my own speed control 
with my good old physics approach of a damped harmonic oscilator ten times :-(

yes, here the prompt for Gemini 2.5pro that did the job in one promt:
	
Please give me the c code for a PID controller that outputs an int16_t pwm value ranging from -BLDC_TIMER_MID_VALUE to +BLDC_TIMER_MID_VALUE for a bldc motor based on a int32_t revs32 speed value that will be revs/s*1024. 
The PID controller and the revs32 will be updated at a rate of PWM_FREQ = 12000 Hz. 
The code will run on a GD32F130, so no floating point engine.
The PID controller code should only work with  integers but no 128bit integers and no 64bit division.
An Init function should accept the usual float values like void PID_Init(PIDController *pid, float kp, float ki, float kd, int16_t min_pwm, int16_t max_pwm, float max_i) 
This shall be the update function being called at 12 kHz: int16_t PID_Update(PIDController *pid, int32_t setpoint, int32_t actual) 

*/

// Gemini 2.5Pro begin ---------------

#define INT64 int64_t

typedef struct {
    // Scaled PID gains for integer math
    int32_t kp_scaled;
    int32_t ki_scaled;
    int32_t kd_scaled;

    // State variables
    INT64 integral_term; // 64-bit to prevent overflow during accumulation
    int32_t prev_error;

    // Output limits
    int16_t min_output;
    int16_t max_output;

    // Anti-windup limit for the integral term (scaled)
    INT64 max_integral_scaled;
		int32_t start_integral;

} PIDController;


// --- Scaling Configuration ---
// To avoid floating-point math, we scale the PID gains and calculations by a
// factor of 2^PID_SCALE_BITS. This allows us to use fast integer arithmetic.
// A higher value increases precision for the gains but requires 64-bit
// intermediate variables to prevent overflow. 16 bits is a good balance.
#define PID_SCALE_BITS 16
#define PID_SCALE (1LL << PID_SCALE_BITS) // Use 1LL for 64-bit literal
#define DRIVER_FREQ 1000
/**
 * @param pid      Pointer to the PIDController structure to initialize.
 * @param kp       The proportional gain (e.g., 0.5).
 * @param ki       The integral gain (e.g., 10.0).
 * @param kd       The derivative gain (e.g., 0.001).
 * @param min_pwm  The minimum PWM output value (e.g., -BLDC_TIMER_MID_VALUE).
 * @param max_pwm  The maximum PWM output value (e.g., BLDC_TIMER_MID_VALUE).
 * @param max_i    The maximum absolute contribution of the integral term to the final output.
 */
void PID_Init(PIDController *pid, float kp, float ki, float kd,
              int16_t min_pwm, int16_t max_pwm, float max_i, int32_t start_i) 
{
    // Calculate and store scaled gains based on the PID equation in discrete time
    pid->kp_scaled = (int32_t)(kp * PID_SCALE);
    pid->ki_scaled = (int32_t)(ki * PID_SCALE / DRIVER_FREQ);
    pid->kd_scaled = (int32_t)(kd * PID_SCALE * DRIVER_FREQ);

    // Store output and anti-windup limits
    pid->min_output = min_pwm;
    pid->max_output = max_pwm;
    pid->max_integral_scaled = (INT64)(max_i * PID_SCALE);
		pid->start_integral = start_i;

    // Reset state variables
    pid->integral_term = 0;
    pid->prev_error = 0;
}

int32_t error;
int32_t delta_error;
uint32_t p_outH; uint32_t p_outL;
uint32_t i_outH; uint32_t i_outL;
uint32_t d_outH; uint32_t d_outL;
int32_t p_out32;
int32_t i_out32;
int32_t d_out32;
int32_t output,output32 ;
int16_t bErr;
extern uint32_t msTicks;
uint16_t iPID_DoI = 30;

/**
 * This function must be called at the frequency defined by PWM_FREQ (12 kHz).
 * It performs all calculations using integer arithmetic for maximum efficiency.
 *
 * @param pid       Pointer to the initialized PIDController structure.
 * @param setpoint  The desired value (target speed in revs/s * 1024).
 * @param actual    The measured value (current speed in revs/s * 1024).
 * @return          The computed and clamped PWM output value as an int16_t.
 */
int16_t PID_Update(PIDController *pid, int32_t setpoint, int32_t actual) {
    // Calculate current error
    error = setpoint - actual;
 //error = 0;

    // --- Proportional Term ---
    // P = Kp * error
    // Result is stored in a 64-bit integer to prevent overflow
    INT64 p_out = (INT64)pid->kp_scaled * error;


		if (	(!pid->start_integral) || ((error < pid->start_integral) && (error > -pid->start_integral)	)	)		// only add integral term if p-term is no longer storng
		{
			// --- Integral Term ---
			// I = I + Ki * error * dt
			pid->integral_term += (INT64)pid->ki_scaled * error;

			// Apply anti-windup: clamp the integral term to prevent it from growing too large
			if (pid->integral_term > pid->max_integral_scaled) {
					pid->integral_term = pid->max_integral_scaled;
			} else if (pid->integral_term < -pid->max_integral_scaled) {
					pid->integral_term = -pid->max_integral_scaled;
			}
		}
		else pid->integral_term = 0;
		
    INT64 i_out = pid->integral_term;

    // --- Derivative Term ---
    // D = Kd * (error - prev_error) / dt
		delta_error = error - pid->prev_error;
		//delta_error = ((msTicks/5)%200)-100;
    INT64 d_out = (INT64)pid->kd_scaled * delta_error;

    // --- Combine Terms & Scale Down ---
    // Sum of P, I, and D terms (the result is still scaled)
    INT64 output_scaled = p_out + i_out + d_out;

		p_outH = p_out>>32;
		p_outL = (uint32_t)p_out;	
		i_outH = i_out>>32;
		i_outL = (uint32_t)i_out;	
		d_outH = d_out>>32;
		d_outL = (uint32_t)d_out;	
		p_out32 = p_out >> PID_SCALE_BITS;
		i_out32 = i_out >> PID_SCALE_BITS;
		d_out32 = d_out >> PID_SCALE_BITS;
		output32 = p_out32 + i_out32 + d_out32;
    if (output32 > pid->max_output) 	output32 = pid->max_output;
    else if (output32 < pid->min_output)	output32 = pid->min_output;
		
		
    // Scale back down to the final output range using a fast bit-shift.
    // This is equivalent to (output_scaled / PID_SCALE) but avoids division.
    output = (int32_t)(output_scaled >> PID_SCALE_BITS);

    // --- Clamp Final Output ---
    // Enforce the min/max PWM limits specified during initialization.
    if (output > pid->max_output) 	output = pid->max_output;
    else if (output < pid->min_output)	output = pid->min_output;

    // --- Update State for Next Iteration ---
    // Store the current error for the next derivative calculation
    pid->prev_error = error;
//return 0;
    return (int16_t)output;
}

// Gemini 2.5Pro end ---------------



extern int32_t revs32;
extern int32_t torque32;
extern uint8_t iDrivingMode;
extern int32_t iOdom;

PIDController pid;	// PID controller instance


//PIDInit: uint16_t iDoEvery; float kp; float ki; float kd;	float minmax_pwm; float max_i; int32_t start_i; 
	// 	iDoEvery:	16 = only every 16th CalculateBLDC() = 1 kHz if PWM_FREQ=16kHz. Set it higher than 1 if PID reacts too fast
	//	kp: correction term proportional to the error
	//	ki: integrate over the error to reach target
	//	kd: positiv feedback if error increases, negative feedback if error decreases	(Robo understanding)
	//	minmax_pwm: 1.0 = +-maximum pwm value (1125 for 16 kHz) allowed
	// 	max_i: 	0.4 = only 40% of max pwm for the ki term
	//	start_i: 30 = only add ki integral term when error is within +-30 (steps for position control). 0= always add ki term
PIDInit aoPIDInit[3] = PIDINIT_a3o;
/*{
	//{16,	0.1f, 0.2f, 0.5f,	1.0, 1.0, 0},		// constant speed in revs*1024			manual tuned by robo
	{16,	0.024f, 0.18f, 0.55f,	1.0, 1.0, 0},		// constant speed in revs*1024		further fine tuned by remoteOptimizePID
	{16,	0.2f, 1.0f, 0.0005f,	1.0, 1.0, 0},		// max torque
	{1,		4.0f, 2.0f, 0.1f	,		0.5, 0.5, 30}			// position = iOdom		FILTER_SHIFT can/should be 9 !
		};
*/

uint16_t iDriverDoEvery = 16;	// only PID_Update every iDriverDoEvery Driver() calls
void DriverInit(uint8_t iDrivingModeNew) 	// Initialize controller (tune these values for your system)
{
	iDrivingMode = iDrivingModeNew;
	if (iDrivingMode==0)	return;
	PIDInit* p = &aoPIDInit[iDrivingMode-1];
	PID_Init(&pid, p->kp, p->ki, p->kd, -p->minmax_pwm*BLDC_TIMER_MID_VALUE, p->minmax_pwm*BLDC_TIMER_MID_VALUE, p->max_i*BLDC_TIMER_MID_VALUE, p->start_i);
	iDriverDoEvery = p->iDoEvery;
}

#ifdef REMOTE_OPTIMIZEPID
	static inline uint32_t uabs32(int32_t x) {
			int32_t mask = x >> 31;          // 0 if x >= 0, -1 if x < 0
			return (uint32_t)((x ^ mask) - mask);
	}
	uint32_t iOptimizeErrors = 0;
	uint32_t iOptimizeTotal = 0;
	int32_t iError32Test=1;
	uint16_t iOptimizeThreshold = 33;	// 100% error would 1024, 5% error = 51, 2% == 33
	uint16_t bError = 0;
	extern int32_t iRemoteMax;
#endif

int16_t	Driver(uint8_t iDrivingMode, int32_t input)		// pwm/speed/torque/position as input and returns pwm value to get a low pass filter in bldc.c
{
	#ifdef REMOTE_OPTIMIZEPID
		//iError32Test = (uabs32(pid.prev_error)<<10)/uabs32(input);
		uint16_t bErrorSet = 5000;
		if(iDrivingMode == 3)
		{
			iError32Test = uabs32(pid.prev_error);		// error is absolute displacement from target iOdom
			bErrorSet = 500;
		}
		else 
			iError32Test = uabs32((pid.prev_error*1024)/input);
		if (	iError32Test > iOptimizeThreshold)
		{		
			iOptimizeErrors++;		// 100% error would 1024, 5% error = 51
			bError = iRemoteMax+10;	//bErrorSet;
		}
		else bError = 0;
		iOptimizeTotal++;
	#endif

	switch (iDrivingMode)
	{
	case 0:	// interpret as simple pwm value
		return input;
	case 1:	// input will be taken as revs/s*1024 
		return PID_Update(&pid, input, revs32);
	case 2:	// input will be taken as Nm*1024 
		return PID_Update(&pid, input, torque32);
	case 3:	// input will be iOdom
		return PID_Update(&pid, input, iOdom);
	}

	return 0;	// error, unkown drive mode
}


#include <stdint.h>
#include "../Inc/bldc.h"


// PID_Gemini	might work better with lower rank bldc_outputFilterPwm like 10

/*
I have given up on ChatGpt :-(
What a waste of time, noch ai could provide pid code that reduces error to 0 to reach target speed
in the time wasted on stupid ai, i could have coded my own speed control 
with my good old physics approach of a damped harmonic oscilator ten times :-(

yes, here the prompt for Gemini 2.5pro that did the job in one promt:
	
Please give me the c code for a PID controller that outputs an int16_t pwm value ranging from -1250 to +1250 for a bldc motor based on a int32_t revs32 speed value that will be revs/s*1024. 
The PID controller and the revs32 will be updated at a rate of PWM_FREQ = 12000 Hz. 
The code will run on a GD32F130, so no floating point engine.
The PID controller code should only work with  integers but no 128bit integers and no 64bit division.
An Init function should accept the usual float values like void PID_Init(PIDController *pid, float kp, float ki, float kd, int16_t min_pwm, int16_t max_pwm, float max_i) 
This shall be the update function being called at 12 kHz: int16_t PID_Update(PIDController *pid, int32_t setpoint, int32_t actual) 

*/

// Gemini 2.5Pro begin ---------------

typedef struct {
    // Scaled PID gains for integer math
    int32_t kp_scaled;
    int32_t ki_scaled;
    int32_t kd_scaled;

    // State variables
    int64_t integral_term; // 64-bit to prevent overflow during accumulation
    int32_t prev_error;

    // Output limits
    int16_t min_output;
    int16_t max_output;

    // Anti-windup limit for the integral term (scaled)
    int64_t max_integral_scaled;

} PIDController;


// --- Scaling Configuration ---
// To avoid floating-point math, we scale the PID gains and calculations by a
// factor of 2^PID_SCALE_BITS. This allows us to use fast integer arithmetic.
// A higher value increases precision for the gains but requires 64-bit
// intermediate variables to prevent overflow. 16 bits is a good balance.
#define PID_SCALE_BITS 16
#define PID_SCALE (1LL << PID_SCALE_BITS) // Use 1LL for 64-bit literal

/**
 * @param pid      Pointer to the PIDController structure to initialize.
 * @param kp       The proportional gain (e.g., 0.5).
 * @param ki       The integral gain (e.g., 10.0).
 * @param kd       The derivative gain (e.g., 0.001).
 * @param min_pwm  The minimum PWM output value (e.g., -1250).
 * @param max_pwm  The maximum PWM output value (e.g., 1250).
 * @param max_i    The maximum absolute contribution of the integral term to the final output.
 */
void PID_Init(PIDController *pid, float kp, float ki, float kd,
              int16_t min_pwm, int16_t max_pwm, float max_i) {

    // Calculate and store scaled gains based on the PID equation in discrete time
    pid->kp_scaled = (int32_t)(kp * PID_SCALE);
    pid->ki_scaled = (int32_t)(ki * PID_SCALE / PWM_FREQ);
    pid->kd_scaled = (int32_t)(kd * PID_SCALE * PWM_FREQ);

    // Store output and anti-windup limits
    pid->min_output = min_pwm;
    pid->max_output = max_pwm;
    pid->max_integral_scaled = (int64_t)(max_i * PID_SCALE);

    // Reset state variables
    pid->integral_term = 0;
    pid->prev_error = 0;
}

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
    int32_t error = setpoint - actual;

    // --- Proportional Term ---
    // P = Kp * error
    // Result is stored in a 64-bit integer to prevent overflow
    int64_t p_out = (int64_t)pid->kp_scaled * error;

    // --- Integral Term ---
    // I = I + Ki * error * dt
    pid->integral_term += (int64_t)pid->ki_scaled * error;

    // Apply anti-windup: clamp the integral term to prevent it from growing too large
    if (pid->integral_term > pid->max_integral_scaled) {
        pid->integral_term = pid->max_integral_scaled;
    } else if (pid->integral_term < -pid->max_integral_scaled) {
        pid->integral_term = -pid->max_integral_scaled;
    }
    int64_t i_out = pid->integral_term;

    // --- Derivative Term ---
    // D = Kd * (error - prev_error) / dt
    int32_t delta_error = error - pid->prev_error;
    int64_t d_out = (int64_t)pid->kd_scaled * delta_error;

    // --- Combine Terms & Scale Down ---
    // Sum of P, I, and D terms (the result is still scaled)
    int64_t output_scaled = p_out + i_out + d_out;

    // Scale back down to the final output range using a fast bit-shift.
    // This is equivalent to (output_scaled / PID_SCALE) but avoids division.
    int32_t output = (int32_t)(output_scaled >> PID_SCALE_BITS);

    // --- Clamp Final Output ---
    // Enforce the min/max PWM limits specified during initialization.
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }

    // --- Update State for Next Iteration ---
    // Store the current error for the next derivative calculation
    pid->prev_error = error;

    return (int16_t)output;
}

// Gemini 2.5Pro end ---------------



extern int32_t revs32;
extern int32_t torque32;
extern uint8_t iDrivingMode;
extern int32_t iOdom;

PIDController pid;	// PID controller instance

void DriverInit(uint8_t iDrivingModeNew) 	// Initialize controller (tune these values for your system)
{
	iDrivingMode = iDrivingModeNew;
	switch (iDrivingMode)
	{
	case 1:	// input will be taken as revs/s 
		PID_Init(&pid, 0.2f, 1.0f, 0.0005f, -1250, 1250, 1250.0f);		
		return;
	case 2:	// input will be taken as NewtonMeter * 1024 
		PID_Init(&pid, 0.2f, 1.0f, 0.0005f, -1250, 1250, 1250.0f);		
		return;
	case 3:	// input will be iOdom position in hall steps = 4°
		PID_Init(&pid, 50.0f, 1.0f, 0.005f, -1250, 1250, 1250.0f);		
		return;
	}
}

int16_t	Driver(uint8_t iDrivingMode, int32_t input)		// pwm/speed/torque/position as input and returns pwm value to get a low pass filter in bldc.c
{
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

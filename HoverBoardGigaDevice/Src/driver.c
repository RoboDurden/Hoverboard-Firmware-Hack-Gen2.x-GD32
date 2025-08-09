#include <stdint.h>
#include "../Inc/bldc.h"

/*

// Deepseek begin -----------------------------
// prompt1: please give me the c code for a pid controller that outputs an int16_t pwm value for a bldc motor based on a uint32_t revs32 speed value. The pid controller and the revs32 will be updated at a rate of PWM_FREQ = 12000.
// prompt2: Yes, the speed should be int32_t to allow negative speeds. The code will run on a GD32F130, so no floating pint engine. Please give me PID controller code that only works with 16bit or 32bit integers.

#define SCALING_SHIFT  10      // 10 bits for fractional part (1024)
#define SCALING_FACTOR (1 << SCALING_SHIFT)

typedef struct {
    int32_t kp;           // Proportional gain (scaled)
    int32_t ki;           // Integral gain (scaled, includes DT)
    int32_t kd;           // Derivative gain (scaled, includes 1/DT)
    int32_t integral_sum; // Integral accumulator
    int32_t prev_error;   // Previous error value
    int16_t min_output;   // Minimum PWM output
    int16_t max_output;   // Maximum PWM output
    int32_t max_integral; // Anti-windup integral limit
} PIDController;

// Initialize PID controller with gains and limits
void PID_Init(PIDController *pid, float kp, float ki, float kd, int16_t min_pwm, int16_t max_pwm, float max_i) 
{
	// Convert gains to fixed-point representation
	pid->kp = (int32_t)(kp * SCALING_FACTOR + 0.5f);
	pid->ki = (int32_t)(ki * SCALING_FACTOR / PWM_FREQ + 0.5f);
	pid->kd = (int32_t)(kd * SCALING_FACTOR * PWM_FREQ + 0.5f);

	pid->integral_sum = 0;
	pid->prev_error = 0;
	pid->min_output = min_pwm;
	pid->max_output = max_pwm;
	pid->max_integral = (int32_t)(max_i + 0.5f);
}

// Update PID controller and compute PWM output
int16_t PID_Update(PIDController *pid, int32_t setpoint, int32_t actual) 
{
	// Calculate error (supports negative speeds)
	int32_t error = setpoint - actual;

	// Proportional term: (kp * error) / SCALING_FACTOR
	int32_t p_term = (int32_t)(((int64_t)pid->kp * error) >> SCALING_SHIFT);

	// Update integral sum with anti-windup
	pid->integral_sum += error;
	if (pid->integral_sum > pid->max_integral) {
			pid->integral_sum = pid->max_integral;
	} else if (pid->integral_sum < -pid->max_integral) {
			pid->integral_sum = -pid->max_integral;
	}

	// Integral term: (ki * integral_sum) / SCALING_FACTOR
	int32_t i_term = (int32_t)(((int64_t)pid->ki * pid->integral_sum) >> SCALING_SHIFT);

	// Derivative term: (kd * (error - prev_error)) / SCALING_FACTOR
	int32_t derivative = error - pid->prev_error;
	pid->prev_error = error;
	int32_t d_term = (int32_t)(((int64_t)pid->kd * derivative) >> SCALING_SHIFT);

	// Compute and clamp output
	int32_t output = p_term + i_term + d_term;

	if (output > pid->max_output) {
			output = pid->max_output;
	} else if (output < pid->min_output) {
			output = pid->min_output;
	}

	return (int16_t)output;
}

// --------EXAMPLE USAGE --------------------
//
// Called at 12kHz from timer interrupt
//	void PWM_Update_Handler() {
//    int32_t current_speed = GetMotorSpeed();  // Your sensor reading
//    int32_t target_speed = GetTargetSpeed();  // Your setpoint
//    
//    int16_t pwm = PID_Update(&speed_pid, target_speed, current_speed);
//    ApplyPWMToMotor(pwm);  // Your PWM output function
//	}

// Deepseek end -----------------------------

*/

// chatGpt 5 begin -------------------

#include <stdint.h>

#define Q_SCALE        (1 << 10)  // Q10 scaling
#define GAIN_SCALE     (1 << 10)  // match Q_SCALE so gain floats are "normal"

// PID structure
typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t min_pwm;
    int32_t max_pwm;
    int32_t max_i;
    int32_t decay;       // decay factor in Q15 (0..32767 = 0..0.9999)
    int32_t i_term;
    int32_t prev_error;
} PIDController;

// Init function: accepts normal float gains
void PID_Init(PIDController *pid, float kp, float ki, float kd,
              int16_t min_pwm, int16_t max_pwm, float max_i, float decay)
{
    pid->kp = (int32_t)(kp * GAIN_SCALE); // now direct scaling
    pid->ki = (int32_t)((ki / 12000.0f) * GAIN_SCALE);
    pid->kd = (int32_t)((kd * 12000.0f) * GAIN_SCALE);
    pid->min_pwm = min_pwm;
    pid->max_pwm = max_pwm;
    pid->max_i = (int32_t)(max_i);
    pid->decay = (int32_t)(decay * 32768.0f); // Q15 decay factor
    pid->i_term = 0;
    pid->prev_error = 0;
}

// Update function: called at 12 kHz
int16_t PID_Update(PIDController *pid, int32_t setpoint_q10, int32_t actual_q10)
{
    int32_t error = setpoint_q10 - actual_q10;

    // Proportional
    int32_t p_term = (pid->kp * error) / GAIN_SCALE;

    // Integral with decay
    pid->i_term = (pid->i_term * (32768 - pid->decay)) >> 15;
    pid->i_term += (pid->ki * error) / GAIN_SCALE;
    if (pid->i_term > pid->max_i)  pid->i_term = pid->max_i;
    if (pid->i_term < -pid->max_i) pid->i_term = -pid->max_i;

    // Derivative
    int32_t d_term = (pid->kd * (error - pid->prev_error)) / GAIN_SCALE;
    pid->prev_error = error;

    // Sum
    int32_t output = p_term + pid->i_term + d_term;

    // Clamp
    if (output > pid->max_pwm)  output = pid->max_pwm;
    if (output < pid->min_pwm)  output = pid->min_pwm;

    return (int16_t)output;
}


// chatGpt 5 end ---------------


extern int32_t revs32;
extern uint8_t iDrivingMode;

PIDController pid;	// PID controller instance


void DriverInit(uint8_t iDrivingModeNew) 	// Initialize controller (tune these values for your system)
{
	iDrivingMode = iDrivingModeNew;
	switch (iDrivingMode)
	{
	case 1:	// input will be taken as revs/s 
		// 					kp=0.5, ki=0.1, kd=0.01, PWM range: ±30000, max integral=10000
		//Deepseek PID_Init(&pid, 0.5f, 	0.1f, 	0.01f, -1250, 1250, 10000.0f);
		//Deepseek PID_Init(&pid, 1.0f, 	0.1f, 	0.01f, -1250, 1250, 10000.0f);
	
		PID_Init(&pid, 0.4f, 0.8f, 0.0f, -1250, 1250, 400.0f, 0.0005f);		// ChatGpt 5

		// Deepseek tuning Recommendations:
		//		Start with only proportional gain (set ki=0, kd=0)
		//		Increase kp until motor begins to oscillate, then reduce by 30%
		//		Slowly increase ki to eliminate steady-state error
		//		Add kd to reduce overshoot (start with 10-20% of kp value)
		//		Adjust anti-windup limit based on maximum expected error
		return;
	}
}

int32_t pwm = 1;

int16_t	Driver(uint8_t iDrivingMode, int32_t input)		// pwm/speed/torque/position as input and returns pwm value to get a low pass filter in bldc.c
{
		switch (iDrivingMode)
		{
		case 0:	// interpret as simple pwm value
			return input;
		case 1:	// input will be taken as revs/s*1024 
			//Deepseek return PID_Update(&pid, input, revs32);
			return PID_Update(&pid, (input*14)/8, revs32);	// ChatGpt code also needs a scaling of target by 14/8 :-(
		
		}
	return 0;	// error, unkown drive mode
}

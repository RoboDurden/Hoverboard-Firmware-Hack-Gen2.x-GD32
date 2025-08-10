#include <stdint.h>
#include "../Inc/bldc.h"


#define PID_Version 3		// 1..3


#if PID_Version == 1
												#define PID_Deepseek
#elif PID_Version == 2
												#define PID_ChatGPT5
#else
												#define PID_ChatGPT5_2	// can work with bldc_outputFilterPwm = bldc_inputFilterPwm
#endif	
/*

		I have given up on ChatGpt :-(
		What a waste of time, noch ai could provide pid code that reduces error to 0 to reach target speed
		in the time wasted on stupid ai, i could have coded my own speed control 
		with my good old physics approach of a damped harmonic oscilator ten times :-(

prompt for future tries:
	
Please give me the c code for a PID controller that outputs an int16_t pwm value ranging from -1250 to +1250 for a bldc motor based on a int32_t revs32 speed value that will be revs/s*1024. 
The PID controller and the revs32 will be updated at a rate of PWM_FREQ = 12000 Hz. 
The code will run on a GD32F130, so no floating point engine.
The PID controller code should only work with  integers but no 128bit integers and no 64bit division.
An Init function should accept the usual float values like void PID_Init(PIDController *pid, float kp, float ki, float kd, int16_t min_pwm, int16_t max_pwm, float max_i) 
This shall be the update function being called at 12 kHz: int16_t PID_Update(PIDController *pid, int32_t setpoint, int32_t actual) 

*/



#ifdef PID_Deepseek
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

#endif	// Deepseek end -----------------------------

#ifdef PID_ChatGPT5

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

#endif	// chatGpt 5 end ---------------


#ifdef PID_ChatGPT5_2


#define Q_SHIFT 8
#define Q_SCALE (1 << Q_SHIFT)

typedef struct {
    // Gains in Q16 (gain * 65536)
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;

    // Feedforward gain in Q8 PWM / Q8 speed units
    int32_t ff_q8;

    // State
    int32_t integral_q8;
    int32_t prev_error_q8;
    int16_t prev_output;

    // Limits
    int16_t min_pwm;
    int16_t max_pwm;
    int32_t max_i_pwm_q8;

    // Config
    int32_t decay_q15;   // decay factor in Q15
    int16_t accel_limit; // max PWM change per update
} PIDController;


// Init function — called once
void PID_Init(PIDController *pid,
              float kp, float ki, float kd,
              float ff_gain,
              int16_t min_pwm, int16_t max_pwm,
              float max_i_pwm,
              float decay,
              int16_t accel_limit)
{
    pid->kp_q16 = (int32_t)(kp * 65536.0f);
    pid->ki_q16 = (int32_t)(ki * 65536.0f / 12000.0f); // scaled per update
    pid->kd_q16 = (int32_t)(kd * 65536.0f * 12000.0f);
    pid->ff_q8  = (int32_t)(ff_gain * Q_SCALE);

    pid->integral_q8 = 0;
    pid->prev_error_q8 = 0;
    pid->prev_output = 0;

    pid->min_pwm = min_pwm;
    pid->max_pwm = max_pwm;
    pid->max_i_pwm_q8 = (int32_t)(max_i_pwm * Q_SCALE);

    pid->decay_q15 = (int32_t)(decay * 32768.0f);
    pid->accel_limit = accel_limit;
}


// Update function — called at 12 kHz
int16_t PID_Update(PIDController *pid, int32_t set_q8, int32_t act_q8)
{
    int32_t error_q8 = set_q8 - act_q8;
    int32_t delta_q8 = error_q8 - pid->prev_error_q8;

    // --- Feedforward ---
    int32_t ff_term = (pid->ff_q8 * set_q8) >> Q_SHIFT; // PWM units

    // --- Proportional term ---
    int32_t p_term = (pid->kp_q16 * error_q8) >> (16);

    // --- Derivative term ---
    int32_t d_term = (pid->kd_q16 * delta_q8) >> (16);

    // --- Integral term ---
    pid->integral_q8 -= (pid->integral_q8 * pid->decay_q15) >> 15;

    // Only integrate if not saturated in same direction
    int32_t out_no_i = ff_term + p_term + d_term;
    int saturated_pos = (out_no_i >= pid->max_pwm && error_q8 > 0);
    int saturated_neg = (out_no_i <= pid->min_pwm && error_q8 < 0);
    if (!saturated_pos && !saturated_neg) {
        int32_t step = error_q8;
        int32_t max_step = pid->max_i_pwm_q8 / 50;
        if (step > max_step) step = max_step;
        else if (step < -max_step) step = -max_step;
        pid->integral_q8 += step;
    }

    int32_t i_term_pwm = (pid->ki_q16 * pid->integral_q8) >> (16);
    if (i_term_pwm > pid->max_i_pwm_q8) i_term_pwm = pid->max_i_pwm_q8;
    else if (i_term_pwm < -pid->max_i_pwm_q8) i_term_pwm = -pid->max_i_pwm_q8;

    // --- Total ---
    int32_t out = ff_term + p_term + i_term_pwm + d_term;

    // Clamp
    if (out > pid->max_pwm) out = pid->max_pwm;
    else if (out < pid->min_pwm) out = pid->min_pwm;

    // --- Acceleration limiting ---
    int16_t diff = (int16_t)out - pid->prev_output;
    if (diff > pid->accel_limit) out = pid->prev_output + pid->accel_limit;
    else if (diff < -pid->accel_limit) out = pid->prev_output - pid->accel_limit;

    pid->prev_output = (int16_t)out;
    pid->prev_error_q8 = error_q8;

    return (int16_t)out;
}


#endif	// 2nd chatGpt 5 end ---------------


extern int32_t revs32;
extern uint8_t iDrivingMode;

PIDController pid;	// PID controller instance


void DriverInit(uint8_t iDrivingModeNew) 	// Initialize controller (tune these values for your system)
{
	iDrivingMode = iDrivingModeNew;
	switch (iDrivingMode)
	{
	case 1:	// input will be taken as revs/s 
		#ifdef PID_Deepseek
			// 					kp=0.5, ki=0.1, kd=0.01, PWM range: ±30000, max integral=10000
			Deepseek PID_Init(&pid, 0.5f, 	0.1f, 	0.01f, -1250, 1250, 10000.0f);
			// Deepseek tuning Recommendations:
			//		Start with only proportional gain (set ki=0, kd=0)
			//		Increase kp until motor begins to oscillate, then reduce by 30%
			//		Slowly increase ki to eliminate steady-state error
			//		Add kd to reduce overshoot (start with 10-20% of kp value)
			//		Adjust anti-windup limit based on maximum expected error
		#endif
		#ifdef PID_ChatGPT5
			PID_Init(&pid, 0.4f, 0.8f, 0.0f, -1250, 1250, 400.0f, 0.0005f);		// ChatGpt 5
		#endif
		#ifdef PID_ChatGPT5_2
			PID_Init(&pid, 0.05f, 0.4f, 0.0f, 0.25f, -1250, 1250, 500.0f, 0.0005f, 1);	
			/*	
			PID_Init(&pid,
					0.01f,    // kp — small, for correction only
					0.5f,     // ki — handles drift
					0.0f,     // kd — optional, start with 0
					0.3f,    // ff_gain — ~PWM per rev/s (Q8 scale)
					-1250, 1250,
					500.0f,   // max_i_pwm
					0.0005f,  // decay
					1        // accel_limit (PWM per update, 12kHz ? ~12.5ms to full scale)
			);
			*/
		#endif
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
			#ifdef PID_ChatGPT5
				return PID_Update(&pid, (input*14)/8, revs32);	// ChatGpt code also needs a scaling of target by 14/8 :-(		
			#else
				return PID_Update(&pid, input, revs32);
			#endif
		}
	return 0;	// error, unkown drive mode
}

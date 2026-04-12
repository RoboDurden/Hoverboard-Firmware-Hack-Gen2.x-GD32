//#include "../Inc/defines.h"
#include "../Inc/bldc.h"
#include "../Inc/foc.h"
#include <stdio.h>
#ifdef RTT_REMOTE
#include "SEGGER_RTT.h"
#endif

/*
// Internal constants
static const int16_t BLDC_TIMER_MID_VALUE = BLDC_TIMER_PERIOD / 2;   // = 1125
static const int16_t BLDC_TIMER_MIN_VALUE = 10;
static const uint16_t BLDC_TIMER_MAX_VALUE = BLDC_TIMER_PERIOD - 10; // = 2240
*/

// Global variables for voltage and current
float batteryVoltage = BAT_CELLS * 3.6;
float currentDC = 0.42;		// to see in serial log that pin is not defined
float realSpeed = 0.0;
int32_t revs32 = 0, revs32Latest = 0;
int32_t torque32 = 0;
int32_t revs32_reg = 0, torque32_reg = 0, realSpeed32_reg = 0;
int32_t revs32x = 0;
int32_t revs32Scale = (PWM_FREQ/15)<<REVS32_SHIFT;			// REVS32_SHIFT-bit fractional precision
int32_t revs32ScaleSlow = (PWM_FREQ/90)<<REVS32_SHIFT;	// REVS32_SHIFT-bit fractional precision

// Timeoutvariable set by timeout timer
extern FlagStatus timedOut;

// Variables to be set from the main routine
extern uint8_t iDrivingMode;
int32_t iBldcInput = 0;	// can be pwm/speed/torque/position
FlagStatus bldc_enable = RESET;

// ADC buffer to be filled by DMA
adc_buf_t adc_buffer;

// Internal calculation variables
uint8_t hall_a;
uint8_t hall_b;
uint8_t hall_c;
volatile uint8_t hall = 0;        // Global hall state

uint8_t pos;
uint8_t lastPos;

// FOC state
FOC_Angle foc_angle;
FOC_Current foc_current;
FOC_AlphaBeta foc_ab;
FOC_DQ foc_dq;
FOC_Controller foc_ctrl;
FOC_Observer foc_obs;
uint16_t foc_offset_y = 2000;  // calibrated at startup
uint16_t foc_offset_b = 2000;

// ISR-rate averaging for debugging
int32_t foc_id_sum = 0, foc_iq_sum = 0;
int32_t foc_iy_sum = 0, foc_ib_sum = 0;
uint16_t foc_avg_count = 0;
int16_t foc_id_avg = 0, foc_iq_avg = 0;
int16_t foc_iy_avg = 0, foc_ib_avg = 0;
int32_t foc_id_var_sum = 0, foc_iq_var_sum = 0;
int32_t foc_iy_var_sum = 0, foc_ib_var_sum = 0;

#ifdef FOC_ENABLED
// Runtime mode: 0=block commutation (startup), 1=FOC
uint8_t foc_mode = 0;
uint32_t foc_warmup_ticks = 0;  // counts up from 0, FOC allowed after warmup
// Speed thresholds for mode switching (in ISR ticks per hall sector)
// Lower ticks = higher speed. Hysteresis prevents oscillation.
#define FOC_ENGAGE_TICKS  200   // switch to FOC only above ~53 RPM
#define FOC_DISENGAGE_TICKS 400 // switch back to block below ~27 RPM
#define FOC_WARMUP_TICKS  48000 // 3 second warmup in block commutation
#endif
int32_t bldc_inputFilterPwm = 0;
int32_t bldc_outputFilterPwm = 0;
uint8_t iDrivingModeOverride = 0;
int32_t filter_reg;
uint8_t iFILTER_SHIFT = FILTER_SHIFT, iFILTER_SHIFTDo=FILTER_SHIFT;
uint16_t buzzerTimer = 0;	// also used to calculate battery voltage :-/
int16_t offsetcount = 0;
int16_t offsetdc = 2000;
uint32_t speedCounter = 0, speedCounterSlow=0;
uint32_t speedCounterLog = 0, speedCounterSlowLog = 0;

uint16_t iDriverDo = 1;
#ifdef BUZZER
FlagStatus buzzerToggle = RESET;
uint8_t buzzerFreq = 0;
uint8_t buzzerPattern = 0;
#endif

// robo23 odometer support
// from https://github.com/alex-makarov/hoverboard-firmware-hack-FOC/blob/master/Src/bldc.c
int32_t iOdom = 0;
int32_t iOdomLast = 0;
int16_t modulo(int16_t m, int16_t rest_classes)
{
  return (((m % rest_classes) + rest_classes) %rest_classes);
}
int16_t up_or_down(int16_t vorher, int16_t nachher)
{
  uint16_t up_down[6] = {0,-1,-2,0,2,1};
  return up_down[modulo(vorher-nachher, 6)];
}

// Commutation table
const uint8_t hall_to_pos[8] =
{
	// annotation: for example SA=0 means hall sensor pulls SA down to Ground
  0, // hall position [-] - No function (access from 1-6) 
  3, // hall position [1] (SA=1, SB=0, SC=0) -> PWM-position 3
  5, // hall position [2] (SA=0, SB=1, SC=0) -> PWM-position 5
  4, // hall position [3] (SA=1, SB=1, SC=0) -> PWM-position 4
  1, // hall position [4] (SA=0, SB=0, SC=1) -> PWM-position 1
  2, // hall position [5] (SA=1, SB=0, SC=1) -> PWM-position 2
  6, // hall position [6] (SA=0, SB=1, SC=1) -> PWM-position 6
  0, // hall position [-] - No function (access from 1-6) 
};





// Set motor enable
void SetEnable(FlagStatus setEnable)
{
	bldc_enable = setEnable;
}

/*
// Set pwm -1000 to 1000
void SetPWM(int16_t setPwm)
{
	//bldc_inputFilterPwm = CLAMP(1.125 * setPwm, -BLDC_TIMER_MID_VALUE, BLDC_TIMER_MID_VALUE); // thanks to WizzardDr, bldc.c: pwm_res = 72000000 / 2 / PWM_FREQ; == 2250 and not 2000
	
	bldc_inputFilterPwm =  BLDC_TIMER_MID_VALUE*(setPwm/1000.0);	// thanks to WizzardDr, bldc.c: pwm_res = 72000000 / 2 / PWM_FREQ; == 2250 and not 2000
	bldc_inputFilterPwm =  CLAMP(bldc_inputFilterPwm ,-BLDC_TIMER_MID_VALUE, BLDC_TIMER_MID_VALUE); 	
}
*/


void SetBldcInput(int32_t input)
{
	switch (iDrivingMode)
	{
	case 0:	// pwm
		iBldcInput =  BLDC_TIMER_MID_VALUE*(input/1000.0);	// thanks to WizzardDr, bldc.c: pwm_res = 72000000 / 2 / PWM_FREQ; == 2250 and not 2000
		iBldcInput =  CLAMP(iBldcInput,-BLDC_TIMER_MID_VALUE, BLDC_TIMER_MID_VALUE); 	
		return;
	case 1:	// speed in revs*1024 = revs<<10
		iBldcInput = input << (REVS32_SHIFT-10);//(REVS32_SHIFT-10);
		return;
	case 2:	// torque in Nm*1024 = torque<<10
		iBldcInput = input;
		return;
	case 3:	// iOdom position in hall steps = 4�
		iBldcInput = input;
		return;
	}
	iBldcInput = input;	// no restrictions for speed/torque/position ?
}		

void SetFilter(uint8_t iNew)
{
	if (iFILTER_SHIFT != iNew)
	{
		iFILTER_SHIFT = iNew;
		filter_reg = bldc_outputFilterPwm << iFILTER_SHIFT;	// scale filter state to match new low pass
	}
}



extern uint32_t steerCounter;								// Steer counter for setting update rate
extern uint32_t iBug;
extern uint32_t msTicks;
uint32_t iBuzzCounter = 0, iBuzzTime=0, iBuzzRate=0;

// Calculation-Routine for BLDC => calculates with 16kHz
void CalculateBLDC(void)
{
	int y = 0;     // yellow = phase A
	int b = 0;     // blue   = phase B
	int g = 0;     // green  = phase C

	
	#ifdef CURRENT_DC
		// Calibrate ADC offsets for the first 1000 cycles
		if (offsetcount < 1000)
		{  
			offsetcount++;
			offsetdc = (adc_buffer.current_dc + offsetdc) / 2;
			return;
		}
	#endif
	
	// Calculate battery voltage every 100 cycles
	#ifdef VBATT
		if (buzzerTimer % 100 == 0)
			batteryVoltage = batteryVoltage * 0.999 + ((float)adc_buffer.v_batt * ADC_BATTERY_VOLT) * 0.001;
	#endif

	/* not really needed, and low pass gets overwritten by DMA anyway
	#ifdef REMOTE_ADC
		adc_buffer.speed = adc_buffer.speed * 0.999 + adc_buffer.speed * 0.001;		// low pass
		adc_buffer.steer = adc_buffer.steer * 0.999 + adc_buffer.steer * 0.001;
	#endif
	*/
		
	
  	buzzerTimer++;	// also used to calculate battery voltage :-/
	if (msTicks > iBuzzTime)
	{
		iBuzzTime = msTicks + 1000;
		iBuzzRate = iBuzzCounter;
		iBuzzCounter = 0;
	}
	else iBuzzCounter++;
		
		
#ifdef BUZZER
	// Create square wave for buzzer
  if (buzzerFreq != 0 && (buzzerTimer / PWM_FREQ) % (buzzerPattern + 1) == 0)
	{
    if (buzzerTimer % (buzzerFreq*(PWM_FREQ/2000)) == 0)
		{
			buzzerToggle = buzzerToggle == RESET ? SET : RESET; // toggle variable
			digitalWrite(BUZZER,buzzerToggle);
		  //gpio_bit_write(BUZZER_PORT, BUZZER_PIN, buzzerToggle);
    }
  }
	else
	{
		digitalWrite(BUZZER,RESET);
		//gpio_bit_write(BUZZER_PORT, BUZZER_PIN, RESET);
  }
#endif
	
	// Calculate current DC
	#ifdef CURRENT_DC
		currentDC = ABS((adc_buffer.current_dc - offsetdc) * MOTOR_AMP_CONV_DC_AMP);
	#endif
	
  // Disable PWM when current limit is reached (current chopping), enable is not set or timeout is reached
	//DEBUG_LedSet((steerCounter%20) > 10,2);	// macro. iCol: 0=green, 1=organge, 2=red
	if (currentDC > DC_CUR_LIMIT  || bldc_enable == RESET  || timedOut == SET)	//		
	{
		iDrivingModeOverride = bldc_inputFilterPwm = iBldcInput = 0;
		SetFilter(14);	// soft brake	old: FILTER_SHIFT + 2
		if (ABS(bldc_outputFilterPwm)<100)
		{
			timer_automatic_output_disable(TIMER_BLDC);		
		}
		//DEBUG_LedSet((steerCounter%20) > 10,2);	// macro. iCol: 0=green, 1=organge, 2=red
  	}
	else
	{
		timer_automatic_output_enable(TIMER_BLDC);
		SetFilter(iFILTER_SHIFTDo);	// FILTER_SHIFT
		iDrivingModeOverride = iDrivingMode;
		//DEBUG_LedSet(hall_c == 0,0)
  	}

	//if (timedOut == SET)	DEBUG_LedSet((steerCounter%2) < 1,0)		
	
	// Read hall sensors (needed at startup as no hall interrupt has fired yet
	hall_a = digitalRead(HALL_A);
	hall_b = digitalRead(HALL_B);
	hall_c = digitalRead(HALL_C);
	hall = hall_a * 1 + hall_b * 2 + hall_c * 4;


	#ifdef TEST_HALL2LED
		#ifdef LED_ORANGE
			digitalWrite(LED_ORANGE,hall_b);
		#elif defined(UPPER_LED)
			digitalWrite(UPPER_LED,hall_b);
		#elif defined(LOWER_LED)
			digitalWrite(LOWER_LED,hall_b);
		#else
			if (hall_b)
			{
				#ifdef LED_GREEN
					digitalWrite(LED_GREEN,(steerCounter%2) < 1);
				#endif
				digitalWrite(LED_RED,(steerCounter%2) < 1);
			}
		#endif
		#ifdef LED_GREEN
			digitalWrite(LED_GREEN,hall_a);
		#endif
		digitalWrite(LED_RED,hall_c);
	#endif
	
	#ifdef PILOT_CALCULATE
		PilotCalculate();
	#endif

	// Determine current position based on hall sensors
	#ifdef REMOTE_AUTODETECT
		pos = AutodetectBldc(hall_to_pos[hall],buzzerTimer);	// AUTODETECT_Stage_Hall | AUTODETECT_Stage_HallOrder
		AutodetectScan(buzzerTimer);
	#else
		pos = hall_to_pos[hall];
	#endif
	// Update FOC angle estimation and phase currents
	foc_angle_update(&foc_angle, pos);
	#if defined(PHASE_CURRENT_Y) && defined(PHASE_CURRENT_B)
		// Ib offset compensation: startup calibration consistently reads
		// ~33 counts too high on PB1 (persistent across sessions). Correcting
		// here keeps the DC bias out of the Clarke/Park transforms.
		foc_current_update(&foc_current, adc_buffer.phase_current_y, adc_buffer.phase_current_b,
		                   foc_offset_y, foc_offset_b + 33);
		foc_clarke(&foc_current, &foc_ab);
		foc_park(&foc_ab, &foc_dq, foc_angle.electrical_angle);

		// Back-EMF observer: seed from halls once motor is spinning, then let it run free
		// Re-seed only if observer is way off from hall (catches drift)
		static uint8_t obs_seeded = 0;
		int16_t obs_hall_diff = (int16_t)(foc_angle.electrical_angle - foc_observer_angle(&foc_obs));
		if (!obs_seeded || obs_hall_diff > 5461 || obs_hall_diff < -5461) {  // > 30°
			if (foc_angle.sector_ticks > 0 && foc_angle.sector_ticks < 5000) {
				// hall velocity = ANGLE_60DEG (10923) per sector_ticks ISR cycles, in Q16
				int32_t hall_velocity = ((int32_t)10923 << 16) / (int32_t)foc_angle.sector_ticks;
				if (foc_angle.direction < 0) hall_velocity = -hall_velocity;
				foc_observer_set(&foc_obs, foc_angle.electrical_angle, hall_velocity);
				obs_seeded = 1;
			}
		}
		foc_observer_update(&foc_obs, foc_dq.d);

		// Accumulate for ISR-rate averaging (compute avg every 1000 cycles = ~62ms)
		foc_id_sum += foc_dq.d;
		foc_iq_sum += foc_dq.q;
		foc_iy_sum += foc_current.iy;
		foc_ib_sum += foc_current.ib;
		foc_avg_count++;
		if (foc_avg_count >= 1000) {
			foc_id_avg = foc_id_sum / 1000;
			foc_iq_avg = foc_iq_sum / 1000;
			foc_iy_avg = foc_iy_sum / 1000;
			foc_ib_avg = foc_ib_sum / 1000;
			// Variance pass (approximate: use avg from this batch)
			foc_id_var_sum = foc_iq_var_sum = foc_iy_var_sum = foc_ib_var_sum = 0;
			foc_id_sum = foc_iq_sum = foc_iy_sum = foc_ib_sum = 0;
			foc_avg_count = 0;
		}
	#endif

// Add this check before setting PWM:
	if (pos == 0) 	// 0b000 and 0b111 should never happen with the three hall sensors
	{
			// Invalid position - disable PWM
			timer_automatic_output_disable(TIMER_BLDC);
			//DEBUG_LedSet(SET,2);	// macro. iCol: 0=green, 1=organge, 2=red
			return;
	}

	extern uint16_t iDriverDoEvery;		// set in Driver:DriverInit()
	if (buzzerTimer%iDriverDoEvery==0)
		bldc_inputFilterPwm = Driver(iDrivingModeOverride,iBldcInput);		// interpret the input as pwm/speed/torque/position.

	// Calculate low-pass filter for pwm value
	filter_reg = filter_reg - (filter_reg >> iFILTER_SHIFT) + bldc_inputFilterPwm;
	bldc_outputFilterPwm = filter_reg >> iFILTER_SHIFT;

#ifdef FOC_ENABLED
	// FOC on/off is driven by wStateMaster bit 0 from the joystick controller
	// (see joystick's -f option). Block commutation runs when FOC is off.
	extern uint8_t wState;
	extern int32_t steer;
	foc_mode = foc_bldc_step(pos, (int16_t)bldc_outputFilterPwm, steer,
	                         (wState & 0x01) ? 1 : 0,
	                         &y, &b, &g);
#else
	// Block commutation only
	bldc_get_pwm(bldc_outputFilterPwm, pos, &y, &b, &g);
#endif

	// Step-response test mode: DISABLED — PI overshot and hit 2A limit.
	// Need more careful approach (ramp instead of step, or lower gains).
	// #define STEP_TEST_MODE
	#ifdef STEP_TEST_MODE
	{
		// Force sector 1 output (Y floats, B+=pwm, G-=pwm)
		static uint32_t test_tick = 0;
		int16_t iq_ref_test = (test_tick & 16000) ? 200 : 0;  // 0.5 Hz toggle (1s high, 1s low)
		test_tick++;

		// PI on Iq only (Vd=0)
		foc_ctrl.pi_q.kp = 153;
		foc_ctrl.pi_q.ki = 76;
		foc_ctrl.pi_q.limit = 500;  // modest voltage limit

		int16_t err = iq_ref_test - foc_dq.q;
		int16_t vq_out = foc_pi_update(&foc_ctrl.pi_q, err);

		// Apply Vq at a fixed angle pointing at sector 1 (ignore rotor angle)
		// For sector 1: Y=0, B=+vq, G=-vq (block-commutation-style output)
		int16_t y_out = 0;
		int16_t b_out = vq_out;
		int16_t g_out = -vq_out;

		ResetTimeout();
		timer_automatic_output_enable(TIMER_BLDC);

		// Disconnect Y phase (both FETs off) so PSU current = real phase current
		timer_channel_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, TIMER_CCX_DISABLE);
		timer_channel_complementary_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, TIMER_CCXN_DISABLE);
		timer_channel_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, TIMER_CCX_ENABLE);
		timer_channel_complementary_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, TIMER_CCXN_ENABLE);
		timer_channel_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, TIMER_CCX_ENABLE);
		timer_channel_complementary_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, TIMER_CCXN_ENABLE);

		timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, BLDC_TIMER_MID_VALUE);
		timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, CLAMP(b_out + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
		timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, CLAMP(g_out + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));

		// Fast logging (1 kHz = every 16 ISR cycles)
		#if defined(RTT_REMOTE)
		if ((test_tick & 15) == 0) {
			char s[48];
			sprintf(s, "%lu %d %d %d\r\n",
				(unsigned long)test_tick, iq_ref_test, foc_dq.q, vq_out);
			SEGGER_RTT_WriteString(0, s);
		}
		#endif
	}
	#else
	// Set PWM output (pwm_res/2 is the mean value, setvalue has to be between 10 and pwm_res-10)
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, CLAMP(g + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, CLAMP(b + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, CLAMP(y + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	#endif

	// RTT logging for back-EMF observer comparison (every ~3000 cycles)
	#if defined(RTT_REMOTE) && defined(PHASE_CURRENT_Y) && defined(PHASE_CURRENT_B)
	{
		static uint16_t rtt_log_count = 0;
		if (++rtt_log_count >= 3000) {
			rtt_log_count = 0;
			char s[120];
			uint16_t hall_deg = (uint32_t)foc_angle.electrical_angle * 360 / 65536;
			uint16_t obs_deg = (uint32_t)foc_observer_angle(&foc_obs) * 360 / 65536;
			int16_t diff_deg = (int16_t)((int32_t)obs_deg - (int32_t)hall_deg);
			if (diff_deg > 180) diff_deg -= 360;
			if (diff_deg < -180) diff_deg += 360;
			uint16_t off_deg = (uint32_t)foc_angle.angle_offset * 360 / 65536;
			extern int32_t steer;
			extern uint8_t wState;
			#ifdef FOC_ENABLED
			uint8_t m_val = foc_mode;
			#else
			uint8_t m_val = 0;
			#endif
			sprintf(s, "wS:0x%02X foc:%d m:%d dir:%+d Id:%+4d Iq:%+4d stk:%u off:%3u str:%+5ld\r\n",
				wState, (wState & 1) ? 1 : 0, m_val,
				(int)foc_angle.direction,
				foc_id_avg, foc_iq_avg,
				(unsigned)foc_angle.sector_ticks, off_deg, (long)steer);
			(void)hall_deg; (void)obs_deg; (void)diff_deg;
			SEGGER_RTT_WriteString(0, s);
		}
	}
	#endif

	// robo23
	iOdom = iOdom - up_or_down(lastPos, pos); // int32 will overflow at +-2.147.483.648

	if(speedCounterSlow < 4000) speedCounterSlow++;	// No speed after 250ms
	if (iOdomLast != iOdom)	// one hall step is 4°
	{
		//if (speedCounterSlow > 600)	// idea was to use the 24° step of realSpeed calculation for better revs32 at higher speeds. But doesn' work for some unkown reason
		if (speedCounterSlow > 10)	// 2.1.11 does not debounce hall inputs :-((((
		{
			revs32Latest = (iOdom-iOdomLast) * (revs32ScaleSlow / speedCounterSlow) ;		// warning, (iOdom-iOdomLast) might give wrong result when iOdom overflows
			// revs32ScaleSlow = (PWM_FREQ/90)<<REVS32_SHIFT;	// REVS32_SHIFT-bit fractional precision
			
			#if defined(CURRENT_DC) && defined(VBATT)
				//  torque = (fEff * V * I * 60) / (RPM * 2p)
				// rpm = 60* (PWM_FREQ/(speedCounterSlow/(iOdom-iOdomLast)) )/90;
				// iTorque = 0.8 * batteryVoltage * currentDC * (90/2*PI) *speedCounterSlow / (PWM_FREQ  *(iOdom-iOdomLast))
				// iTorque = 11.459 * batteryVoltage * currentDC *speedCounterSlow / (PWM_FREQ  *(iOdom-iOdomLast))
				int32_t torque32Now = ((((int32_t)(11.459 * batteryVoltage * currentDC))<<10) / (PWM_FREQ  *(iOdom-iOdomLast))) * speedCounterSlow;

				#define RANK_torque32 3 	// Calculate low-pass filter for pwm value
				torque32_reg = torque32_reg - (torque32_reg >> RANK_torque32) + torque32Now;
				torque32 = (speedCounterSlow < 1000) ? torque32_reg >> RANK_torque32 : torque32Now;
			#endif			
		}
		speedCounterSlowLog = speedCounterSlow;		// for logging with StmStudio
		speedCounterSlow = 0;
	}
	else if (speedCounterSlow >= 4000)	revs32Latest = revs32_reg = torque32 = torque32_reg = 0;

	#define RANK_revs32 4 	// Calculate low-pass filter for revs32 value
	revs32_reg = revs32_reg - (revs32_reg >> RANK_revs32) + revs32Latest ;		// warning, (iOdom-iOdomLast) might give wrong result when iOdom overflows
	revs32 = revs32_reg >> RANK_revs32;
		
	// Increments with 62.5us
	if(speedCounter < 8000) speedCounter++;	// No speed after 250ms
	
	#ifdef SPEED_AsRevsPerSec
	
		#define RANK_realSpeed32 10 	// Calculate low-pass filter for pwm value
		realSpeed32_reg = realSpeed32_reg - (realSpeed32_reg >> RANK_realSpeed32) + revs32;
	
		realSpeed	= (realSpeed32_reg >> RANK_realSpeed32) / 1024.0;
	#else
		// Every time position reaches value 1, one (electrical 360�) round = 24� mechanical angle is performed (rising edge)
		if (lastPos != 1 && pos == 1)
		{
			realSpeed = 1991.81f / (float)speedCounter; //[km/h]		// robo 2025: should get changed to rpm ?
			revs32x =  revs32Scale / speedCounter;
			if (lastPos == 2)	
				realSpeed *= -1;
			else
				revs32x *= -1;
			
			speedCounterLog = speedCounter;	// for logging with StmStudio
			speedCounter = 0;
		}
		else if (speedCounter >= 8000)	realSpeed = revs32x = 0;
	#endif
	
	//if (speedCounterSlowLog <= 600)	revs32 = revs32x;		// that doesn't work as motor begins to oscillate
	
	// Safe last position
	lastPos = pos;
	iOdomLast = iOdom;
}

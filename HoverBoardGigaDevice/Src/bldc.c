//#include "../Inc/defines.h"
#include "../Inc/bldc.h"
#include <stdio.h>

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
int32_t revs32 = 0;
int32_t torque32 = 0;
int32_t revs32_reg = 0, torque32_reg = 0, realSpeed32_reg = 0;
int32_t revs32x = 0;
#define REVS32_SHIFT 12
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
int32_t bldc_inputFilterPwm = 0;
int32_t bldc_outputFilterPwm = 0;
uint8_t iDrivingModeOverride = 0;
int32_t filter_reg;
uint8_t nFILTER_SHIFT = FILTER_SHIFT;
uint8_t iFILTER_SHIFT = FILTER_SHIFT;
uint16_t buzzerTimer = 0;	// also used to calculate battery voltage :-/
int16_t offsetcount = 0;
int16_t offsetdc = 2000;
uint32_t speedCounter = 0, speedCounterSlow=0;
uint32_t speedCounterLog = 0, speedCounterSlowLog = 0;

#ifdef BUZZER
FlagStatus buzzerToggle = RESET;
uint8_t buzzerFreq = 0;
uint8_t buzzerPattern = 0;
#endif

// robo23 odometer support
// from https://github.com/alex-makarov/hoverboard-firmware-hack-FOC/blob/master/Src/bldc.c
int32_t iOdom = 0;
//int32_t iOdomLast = 0;
int16_t modulo(int16_t m, int16_t rest_classes)
{
  return (((m % rest_classes) + rest_classes) %rest_classes);
}
// JW: Could be simplified by assuming max stepping up or down 1 step per bldc loop (16kHz => 62.5us).
// Rotating wheel more than 4° (1 step) per 62.5us is unrealistic for a hoverboard.
// 1 step in 62.5us for a 6.5 inch wheel hoverboard corresponds to the hoverboard moving at ~332km/h!
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
	iBug = PWM_FREQ;
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
		iDrivingModeOverride = bldc_inputFilterPwm = 0;
		SetFilter(FILTER_SHIFT + 2);	// soft brake
		if (ABS(bldc_outputFilterPwm)<100)
		{
			timer_automatic_output_disable(TIMER_BLDC);		
		}
		//DEBUG_LedSet((steerCounter%20) > 10,2);	// macro. iCol: 0=green, 1=organge, 2=red
  	}
	else
	{
		timer_automatic_output_enable(TIMER_BLDC);
		SetFilter(nFILTER_SHIFT);
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
// Add this check before setting PWM:
	if (pos == 0) 	// 0b000 and 0b111 should never happen with the three hall sensors
	{
			// Invalid position - disable PWM
			timer_automatic_output_disable(TIMER_BLDC);
			//DEBUG_LedSet(SET,2);	// macro. iCol: 0=green, 1=organge, 2=red
			return;
	}

	//if (buzzerTimer%20==0)	
	bldc_inputFilterPwm = Driver(iDrivingModeOverride,iBldcInput);		// interpret the input as pwm/speed/torque/position.
	
	// Calculate low-pass filter for pwm value
	filter_reg = filter_reg - (filter_reg >> iFILTER_SHIFT) + bldc_inputFilterPwm;
	bldc_outputFilterPwm = filter_reg >> iFILTER_SHIFT;
	
	//bldc_outputFilterPwm = bldc_inputFilterPwm;		// does not work for drivingMode 1, low-pass is needed :-(

	// Update PWM channels based on position y(ellow), b(lue), g(reen)
	bldc_get_pwm(bldc_outputFilterPwm, pos, &y, &b, &g);

	// Set PWM output (pwm_res/2 is the mean value, setvalue has to be between 10 and pwm_res-10)
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, CLAMP(g + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, CLAMP(b + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, CLAMP(y + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));

	// robo23
	int32_t iOdomDelta = up_or_down(lastPos, pos);
	iOdom = iOdom - iOdomDelta; // int32 will overflow at +-2.147.483.648 (4°=~0.005763m for a 6.5 inch wheel. overflow after ~12376km)
	
	if(speedCounterSlow < 4000) speedCounterSlow++;	// No speed after 250ms
	if (iOdomDelta != 0)	// one hall step is 4°
	{
		//if (speedCounterSlow > 600)	// idea was to use the 24� step of realSpeed calculation for better revs32 at higher speeds. But doesn' work for some unkown reason
		{
			int32_t revs32Now =  (-iOdomDelta) * (revs32ScaleSlow / speedCounterSlow);
			
			#define RANK_revs32 2 	// Calculate low-pass filter for pwm value
			revs32_reg = revs32_reg - (revs32_reg >> RANK_revs32) + revs32Now;

			revs32 = (speedCounterSlow < 1000) ? revs32_reg >> RANK_revs32 : revs32Now;
			
			#if defined(CURRENT_DC) && defined(VBATT)
				//  torque = (fEff * V * I * 60) / (RPM * 2p)
				// rpm = 60* (PWM_FREQ/(speedCounterSlow/(-iOdomDelta)) )/90;
				// iTorque = 0.8 * batteryVoltage * currentDC * (90/2*PI) *speedCounterSlow / (PWM_FREQ  *(-iOdomDelta))
				// iTorque = 11.459 * batteryVoltage * currentDC *speedCounterSlow / (PWM_FREQ  *(-iOdomDelta))
				int32_t torque32Now = ((((int32_t)(11.459 * batteryVoltage * currentDC))<<10) / (PWM_FREQ  *(-iOdomDelta))) * speedCounterSlow;

				#define RANK_torque32 3 	// Calculate low-pass filter for pwm value
				torque32_reg = torque32_reg - (torque32_reg >> RANK_torque32) + torque32Now;
				
				torque32 = (speedCounterSlow < 1000) ? torque32_reg >> RANK_torque32 : torque32Now;
			#endif			
		}
		speedCounterSlowLog = speedCounterSlow;		// for logging with StmStudio
		speedCounterSlow = 0;
	}
	else if (speedCounterSlow >= 4000)	revs32 = revs32_reg = torque32 = torque32_reg = 0;
		
	// Increments with 62.5us
	if(speedCounter < 8000) speedCounter++;	// No speed after 500ms
	
	#ifdef SPEED_AsRevsPerSec
	
		#define RANK_realSpeed32 10 	// Calculate low-pass filter for pwm value
		realSpeed32_reg = realSpeed32_reg - (realSpeed32_reg >> RANK_realSpeed32) + revs32;
	
		realSpeed	= (realSpeed32_reg >> RANK_realSpeed32) / 1024.0;
	#else
		// Every time position reaches value 1, one (electrical 360�) round = 24� mechanical angle is performed (rising edge)
		if (lastPos != 1 && pos == 1)
		{
			realSpeed = 1991.81f / (float)speedCounter; //[km/h]		// robo 2025: should get changed to rpm ?
			                                                        // JW: km/h calculation assumes 6.5 inch wheel. For 10 inch wheel, replace 1991.81f with 3064.32f. rpm or revs would be better since it is independent of wheel dimension.
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
	
	// Save last position
	lastPos = pos;
}


#include "../Inc/defines.h"
#include <stdio.h>

// Internal constants
static const int16_t BLDC_TIMER_MID_VALUE = BLDC_TIMER_PERIOD / 2;   // = 1125
static const int16_t BLDC_TIMER_MIN_VALUE = 10;
static const uint16_t BLDC_TIMER_MAX_VALUE = BLDC_TIMER_PERIOD - 10; // = 2240

// Global variables for voltage and current
float batteryVoltage = BAT_CELLS * 3.6;
float currentDC = 0.42;		// to see in serial log that pin is not defined
float realSpeed = 0.0;

// Timeoutvariable set by timeout timer
extern FlagStatus timedOut;

// Variables to be set from the main routine
int16_t bldc_inputFilterPwm = 0;
FlagStatus bldc_enable = RESET;

// ADC buffer to be filled by DMA
adc_buf_t adc_buffer;

// Internal calculation variables
uint8_t hall_a;
uint8_t hall_b;
uint8_t hall_c;
uint8_t hall;
uint8_t pos;
uint8_t lastPos;
int16_t bldc_outputFilterPwm = 0;
int32_t filter_reg;
uint16_t buzzerTimer = 0;	// also used to calculate battery voltage :-/
int16_t offsetcount = 0;
int16_t offsetdc = 2000;
uint32_t speedCounter = 0;

#ifdef BUZZER
FlagStatus buzzerToggle = RESET;
uint8_t buzzerFreq = 0;
uint8_t buzzerPattern = 0;
#endif

// robo23 odometer support
// from https://github.com/alex-makarov/hoverboard-firmware-hack-FOC/blob/master/Src/bldc.c
int32_t iOdom = 0;
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

//----------------------------------------------------------------------------
// Block PWM calculation based on position
//----------------------------------------------------------------------------
#ifndef PLATFORMIO
__INLINE 
#endif
void blockPWM(int pwm, int pwmPos, int *y, int *b, int *g)		// robo 2024/09/04: remove __INLINE for PlatformIO
{
  switch(pwmPos)
	{
    case 1:
      *y = 0;
      *b = pwm;
      *g = -pwm;
      break;
    case 2:
      *y = -pwm;
      *b = pwm;
      *g = 0;
      break;
    case 3:
      *y = -pwm;
      *b = 0;
      *g = pwm;
      break;
    case 4:
      *y = 0;
      *b = -pwm
		;
      *g = pwm;
      break;
    case 5:
      *y = pwm;
      *b = -pwm;
      *g = 0;
      break;
    case 6:
      *y = pwm;
      *b = 0;
      *g = -pwm;
      break;
    default:
      *y = 0;
      *b = 0;
      *g = 0;
  }
}

//----------------------------------------------------------------------------
// Set motor enable
//----------------------------------------------------------------------------
void SetEnable(FlagStatus setEnable)
{
	bldc_enable = setEnable;
}

//----------------------------------------------------------------------------
// Set pwm -1000 to 1000
//----------------------------------------------------------------------------
void SetPWM(int16_t setPwm)
{
	//bldc_inputFilterPwm = CLAMP(1.125 * setPwm, -BLDC_TIMER_MID_VALUE, BLDC_TIMER_MID_VALUE); // thanks to WizzardDr, bldc.c: pwm_res = 72000000 / 2 / PWM_FREQ; == 2250 and not 2000
	
	bldc_inputFilterPwm =  BLDC_TIMER_MID_VALUE*(setPwm/1000.0);	// thanks to WizzardDr, bldc.c: pwm_res = 72000000 / 2 / PWM_FREQ; == 2250 and not 2000
	bldc_inputFilterPwm =  CLAMP(bldc_inputFilterPwm ,-BLDC_TIMER_MID_VALUE, BLDC_TIMER_MID_VALUE); 	
}


extern uint32_t steerCounter;								// Steer counter for setting update rate

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
	
  buzzerTimer++;	// also used to calculate battery voltage :-/
#ifdef BUZZER
	// Create square wave for buzzer
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0)
	{
    if (buzzerTimer % buzzerFreq == 0)
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
	if (currentDC > DC_CUR_LIMIT || bldc_enable == RESET || timedOut == SET)
	{
		timer_automatic_output_disable(TIMER_BLDC);		
		//DEBUG_LedSet(SET,0)
  }
	else
	{
		timer_automatic_output_enable(TIMER_BLDC);
		//DEBUG_LedSet(hall_c == 0,0)
  }

	//if (timedOut == SET)	DEBUG_LedSet((steerCounter%2) < 1,0)		
	
	// Read hall sensors
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
				digitalWrite(LED_GREEN,(steerCounter%2) < 1);
				digitalWrite(LED_RED,(steerCounter%2) < 1);
			}
		#endif
		digitalWrite(LED_GREEN,hall_a);
		digitalWrite(LED_RED,hall_c);
	#endif

	// Determine current position based on hall sensors
	#ifdef REMOTE_AUTODETECT
		pos = AutodetectBldc(hall_to_pos[hall],buzzerTimer);
		AutodetectScan(buzzerTimer);
	#else
		pos = hall_to_pos[hall];
	#endif
	
		
	// Calculate low-pass filter for pwm value
	filter_reg = filter_reg - (filter_reg >> FILTER_SHIFT) + bldc_inputFilterPwm;
	bldc_outputFilterPwm = filter_reg >> FILTER_SHIFT;

	// Update PWM channels based on position y(ellow), b(lue), g(reen)
	blockPWM(bldc_outputFilterPwm, pos, &y, &b, &g);

	// Set PWM output (pwm_res/2 is the mean value, setvalue has to be between 10 and pwm_res-10)
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, CLAMP(g + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, CLAMP(b + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, CLAMP(y + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));

	// robo23
	iOdom = iOdom - up_or_down(lastPos, pos); // int32 will overflow at +-2.147.483.648
	
	// Increments with 62.5us
	if(speedCounter < 4000) speedCounter++;	// No speed after 250ms
	
	// Every time position reaches value 1, one round is performed (rising edge)
	if (lastPos != 1 && pos == 1)
	{
		realSpeed = 1991.81f / (float)speedCounter; //[km/h]
		if (lastPos == 2)	realSpeed *= -1;
		speedCounter = 0;
	}
	else if (speedCounter >= 4000)	realSpeed = 0;
	
	// Safe last position
	lastPos = pos;
}

/*
* This file is part of the hoverboard-firmware-hack-V2 project. The 
* firmware is used to hack the generation 2 board of the hoverboard.
* These new hoverboards have no mainboard anymore. They consist of 
* two Sensorboards which have their own BLDC-Bridge per Motor and an
* ARM Cortex-M3 processor GD32F130C8.
*
* Copyright (C) 2018 Florian Staeblein
* Copyright (C) 2018 Jakob Broemauer
* Copyright (C) 2018 Kai Liebich
* Copyright (C) 2018 Christoph Lehnert
* Copyright (C) 2024 Robo Durden
* Copyright (C) 2025 Hoverboard Havoc
*
* The program is based on the hoverboard project by Niklas Fauth. The 
* structure was tried to be as similar as possible, so that everyone 
* could find a better way through the code.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <stdio.h>

#include "../Inc/defines.h"
#include "../Inc/commutation.h"
#include <stdio.h>

// Internal constants

static const int16_t BLDC_TIMER_MID_VALUE = BLDC_TIMER_PERIOD / 2;   // = 1125


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
		pos = AutodetectBldc(get_sector(hall_a, hall_b, hall_c), buzzerTimer);
		AutodetectScan(buzzerTimer);
	#else
		pos = get_sector(hall_a, hall_b, hall_c);
	#endif
	

	// Calculate low-pass filter for pwm value
	filter_reg = filter_reg - (filter_reg >> FILTER_SHIFT) + bldc_inputFilterPwm;
	bldc_outputFilterPwm = filter_reg >> FILTER_SHIFT;

	uint32_t pulseA, pulseB, pulseC;
	get_pwm(bldc_outputFilterPwm, pos, &pulseA, &pulseB, &pulseC);
	// Update PWM channels based on position y(ellow), b(lue), g(reen)

	// Set PWM output (pwm_res/2 is the mean value, setvalue has to be between 10 and pwm_res-10)
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, pulseA);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, pulseB);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, pulseC);

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
	
	// Save last position
	lastPos = pos;
}

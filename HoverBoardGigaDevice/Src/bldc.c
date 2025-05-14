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

#include "../Inc/defines.h"
#include <stdio.h>

// Internal constants
static const uint8_t HALL_A_MASK = 0x4;
static const uint8_t HALL_B_MASK = 0x2;
static const uint8_t HALL_C_MASK = 0x1;

static const int16_t BLDC_TIMER_MID_VALUE = BLDC_TIMER_PERIOD / 2;   // = 1125
static const int16_t BLDC_TIMER_MIN_VALUE = 10;
static const uint16_t BLDC_TIMER_MAX_VALUE = BLDC_TIMER_PERIOD - 10; // = 2240
static const uint16_t PHASE_ADVANCE_AT_MAX_PWM = 6;
static const int16_t sine_q15[61] = {
       0, 572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126, 5690, 6252, 6813, 7371, 7927, 8481, 9032, 9580, 10126, 10668, 11207, 11743, 12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886, 16384, 16877, 17364, 17847, 18324, 18795, 19261, 19720, 20174, 20622, 21063, 21498, 21926, 22348, 22763, 23170, 23571, 23965, 24351, 24730, 25102, 25466, 25822, 26170, 26510, 26842, 27166, 27482, 27789, 28088, 28378
};

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


__INLINE uint8_t getHall()
{
    // Read hall sensors
    uint8_t hall_a = digitalRead(HALL_A);
    uint8_t hall_b = digitalRead(HALL_B);
    uint8_t hall_c = digitalRead(HALL_C);
    return hall_a * HALL_A_MASK + hall_b * HALL_B_MASK + hall_c * HALL_C_MASK;
}

// Uses Jantzen Lee convention https://www.youtube.com/watch?v=XCzfHDnt6G4
__INLINE uint8_t getPosition(uint8_t hall)
{
	switch (hall)
	{
		case 0b100:
			return 0;
		case 0b101:
			return 1;
		case 0b001:
			return 2;
		case 0b011:
			return 3;
		case 0b010:
			return 4;
		case 0b110:
			return 5;
		default:
			return 0; // Not sure what a good convention for invalid state is.  Maybe the MCU should panic?
					  // If we're field weakning, crashes can cause back EMF that blows stuff up. // https://www.youtube.com/watch?v=5eQyoVMz1dY
	}
}

__INLINE void svmPWM(int sector, int t0, int t1, int t2,
            uint32_t *phaseA, uint32_t *phaseB, uint32_t *phaseC)
{
    // exact integer half of the zero vector time
    uint32_t h = (uint32_t)t0 >> 1;

    switch (sector)
    {
    case 0:
        *phaseA = h + (uint32_t)t2;
        *phaseB = h;
        *phaseC = h + (uint32_t)t1 + (uint32_t)t2;
        break;
    case 1:
        *phaseA = h + (uint32_t)t1 + (uint32_t)t2;
        *phaseB = h;
        *phaseC = h + (uint32_t)t1;
        break;
    case 2:
        *phaseA = h + (uint32_t)t1 + (uint32_t)t2;
        *phaseB = h + (uint32_t)t2;
        *phaseC = h;
        break;
    case 3:
        *phaseA = h + (uint32_t)t1;
        *phaseB = h + (uint32_t)t1 + (uint32_t)t2;
        *phaseC = h;
        break;
    case 4:
        *phaseA = h;
        *phaseB = h + (uint32_t)t1 + (uint32_t)t2;
        *phaseC = h + (uint32_t)t2;
        break;
    case 5:
        *phaseA = h;
        *phaseB = h + (uint32_t)t2;
        *phaseC = h + (uint32_t)t1 + (uint32_t)t2;
        break;
    default:
        // fault condition: all off
        *phaseA = 0;
        *phaseB = 0;
        *phaseC = 0;
        break;
    }
}


__INLINE int32_t calculateT(int16_t angle) {
	// This needs some work
	int16_t sine = sine_q15[angle];
	int32_t tmp = bldc_outputFilterPwm;
	int32_t tmp2 = tmp * sine;
	int32_t tmp3 = tmp2 >> 15;
	int32_t tmp4 = tmp3 * BLDC_TIMER_MAX_VALUE;
	int32_t tmp5 = tmp4 / 1000;
	return tmp5;
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
	bldc_inputFilterPwm = CLAMP(setPwm, -1000, 1000);
}


extern uint32_t steerCounter;								// Steer counter for setting update rate


// Calculation-Routine for BLDC => calculates with 16kHz
void CalculateBLDC(void)
{
	uint32_t pulseA;
	uint32_t pulseB;
	uint32_t pulseC;
	
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

    hall = getHall();

	#ifdef TEST_HALL2LED
		#ifdef LED_ORANGE
			digitalWrite(LED_ORANGE, hall & HALL_B_MASK);
		#elif defined(UPPER_LED)
			digitalWrite(UPPER_LED, hall & HALL_B_MASK);
		#elif defined(LOWER_LED)
			digitalWrite(LOWER_LED, hall & HALL_B_MASK);
		#else
			if (hall & HALL_B_MASK)
			{
				digitalWrite(LED_GREEN,(steerCounter%2) < 1);
				digitalWrite(LED_RED,(steerCounter%2) < 1);
			}
		#endif
		digitalWrite(LED_GREEN, hall & HALL_A_MASK);
		digitalWrite(LED_RED, hall & HALL_C_MASK);
	#endif

	// Determine current position based on hall sensors
	#ifdef REMOTE_AUTODETECT
		pos = AutodetectBldc(hall_to_pos[hall],buzzerTimer);
		AutodetectScan(buzzerTimer);
	#else
		pos = getPosition(hall);
	#endif
	
		
	// Calculate low-pass filter for pwm value
	filter_reg = filter_reg - (filter_reg >> FILTER_SHIFT) + bldc_inputFilterPwm;
	bldc_outputFilterPwm = filter_reg >> FILTER_SHIFT;

	int angle = 30 + bldc_outputFilterPwm * PHASE_ADVANCE_AT_MAX_PWM / 1000;
	// This is based on Jantzen Lee's equations https://youtu.be/oHEVdXucSJs?t=510
	// ChatGPT thinks that I need to divide by a sine term.
	int32_t t1 = calculateT(60 - angle);
	int32_t t2 = calculateT(angle);
	int32_t t0 = BLDC_TIMER_MAX_VALUE - t1 - t2;

	svmPWM(pos, t0, t1, t2, &pulseA, &pulseB, &pulseC);

	// Set PWM output (pwm_res/2 is the mean value, setvalue has to be between 10 and pwm_res-10)
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, CLAMP(pulseA, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, CLAMP(pulseB, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, CLAMP(pulseC, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE));
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

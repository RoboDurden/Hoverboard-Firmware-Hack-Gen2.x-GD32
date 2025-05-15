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

#include "config.h"
#include "commutation.h"
#include "mathDefines.h"

// Internal constants
static const int16_t BLDC_TIMER_MID_VALUE = BLDC_TIMER_PERIOD / 2; // = 1125
static const int16_t BLDC_TIMER_MIN_VALUE = 10;
static const uint16_t BLDC_TIMER_MAX_VALUE = BLDC_TIMER_PERIOD - 10; // = 2240



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
void blockPWM(int pwm, int pwmPos, int *a, int *b, int *c)		// robo 2024/09/04: remove __INLINE for PlatformIO
{
  switch(pwmPos)
	{
    case 1:
      *b = 0;
      *a = pwm;
      *c = -pwm;
      break;
    case 2:
      *b = -pwm;
      *a = pwm;
      *c = 0;
      break;
    case 3:
      *b = -pwm;
      *a = 0;
      *c = pwm;
      break;
    case 4:
      *b = 0;
      *a = -pwm;
      *c = pwm;
      break;
    case 5:
      *b = pwm;
      *a = -pwm;
      *c = 0;
      break;
    case 6:
      *b = pwm;
      *a = 0;
      *c = -pwm;
      break;
    default:
      *b = 0;
      *a = 0;
      *c = 0;
  }
}

uint8_t get_sector(uint8_t hall_a, uint8_t hall_b, uint8_t hall_c)
{
  uint8_t hall = hall_a * 1 + hall_b * 2 + hall_c * 4;
  return hall_to_pos[hall];
}

void get_pwm(int pwm, uint8_t sector, uint32_t *pulse_a, uint32_t *pulse_b, uint32_t *pulse_c)
{
  int phaseA = 0; // yellow
  int phaseB = 0; // blue
  int phaseC = 0; // green

  // Update PWM channels based on position y(ellow), b(lue), g(reen)
  blockPWM(pwm, sector, &phaseA, &phaseB, &phaseC);
  // Set PWM output (pwm_res/2 is the mean value, setvalue has to be between 10 and pwm_res-10)
  *pulse_a = CLAMP(phaseA + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE);
  *pulse_b = CLAMP(phaseB + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE);
  *pulse_c = CLAMP(phaseC + BLDC_TIMER_MID_VALUE, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE);
}

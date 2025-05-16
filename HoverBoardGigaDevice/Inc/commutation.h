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

#ifndef COMMUTATION_H
#define COMMUTATION_H

#include <stdint.h>

#include "config.h" // Neded for tests

static const uint16_t BLDC_TIMER_MIN_VALUE = 10;
static const uint16_t BLDC_TIMER_MAX_VALUE = BLDC_TIMER_PERIOD - 10; // = 2240
/**
 * Max duty cycle is set to 1125 because of legacy implementation details in blockCommutation.c
 */
static const uint16_t DUTY_CYCLE_100_PERCENT = BLDC_TIMER_PERIOD / 2;   // = 1125

/**
 * @brief Determines the commutation sector based on Hall sensor inputs.
 *
 * This function interprets the states of the three Hall sensors and returns
 * the corresponding commutation sector for BLDC motor control.
 * Different papers and videos use different sector numbering conventions so we delegate this decision to the implementation.
 * The convention for the sector numbering must be compatible with the get_pwm implementation.
 * 
 * @param hall_a State of Hall sensor A (0 or 1).
 * @param hall_b State of Hall sensor B (0 or 1).
 * @param hall_c State of Hall sensor C (0 or 1).
 * @return Sector number (1-6) corresponding to the Hall sensor pattern.
 */
uint8_t get_sector(uint8_t hall_a, uint8_t hall_b, uint8_t hall_c);

/**
 * @brief Calculates PWM pulse values for each phase based on duty cycle and sector.
 *
 * This function computes the PWM pulse widths for phases A, B, and C, given the
 * desired duty cycle and commutation sector. The results are written to the
 * provided pointers.
 *
 * @param dutyCycle Desired PWM duty cycle (a value between -1125 and 1125 (DUTY_CYCLE_100_PERCENT) for legacy reasons).
 * @param sector    Current commutation sector provided by get_sector().
 * @param pulse_a   Pointer to store calculated PWM value for phase A.
 * @param pulse_b   Pointer to store calculated PWM value for phase B.
 * @param pulse_c   Pointer to store calculated PWM value for phase C.
 */
void get_pwm(int dutyCycle, uint8_t sector, uint32_t *pulse_a, uint32_t *pulse_b, uint32_t *pulse_c);

#endif
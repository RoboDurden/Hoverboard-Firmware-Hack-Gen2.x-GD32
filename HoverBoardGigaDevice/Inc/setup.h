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

#ifndef SETUP_H
#define SETUP_H

//#include "../Inc/configSelect.h"


#ifndef pinMode
	void pinModePull	(uint32_t pin, uint32_t mode, uint32_t pull);
	void pinMode(uint32_t pin, uint32_t mode);
#endif

void Clock_init(void);
void Clock_test(void);
void Interrupt_init(void);	// Initializes the interrupts
ErrStatus Watchdog_init(void);	// Initializes the watchdog
void TimeoutTimer_init(void);	// Initializes the timeout timer
void GPIO_init(void);	// Initializes the GPIOs
void PWM_init(void);	// Initializes the PWM
void ADC_init(void);	// Initializes the ADC

void USART0_Init(uint32_t iBaud);
void USART1_Init(uint32_t iBaud);
void USART2_Init(uint32_t iBaud);
void USART_MasterSlave_init(void);	// Initializes the usart master slave

//void USART_Steer_COM_init(void);	// Initializes the steer/bluetooth usart


void ConfigReset(void);
void ConfigWrite(void);
void ConfigRead(void);

#endif

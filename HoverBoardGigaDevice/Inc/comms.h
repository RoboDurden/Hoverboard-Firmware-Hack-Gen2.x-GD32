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

#ifndef COMMS_H
#define COMMS_H

//#include "gd32f1x0.h"
#include "../Inc/config.h"

//----------------------------------------------------------------------------
// Send buffer via USART
//----------------------------------------------------------------------------
void SendBuffer(uint32_t usart_periph, uint8_t buffer[], uint8_t length);

//----------------------------------------------------------------------------
// Calculate CRC
//----------------------------------------------------------------------------
uint16_t CalcCRC(uint8_t *ptr, int count);


// Description....: Clearing a page of microprocessor memory
// uint32_t address = Address of the page in Flash memory
void flashErase(uint32_t address);

// Description....: Reads 4 bytes from the microprocessor memory
// uint32_t address = Cell address;
uint32_t flashRead(uint32_t address);

// Description....: Writes 4 bytes to the microprocessor memory
// uint32_t address = Cell address;
// uint32_t data = Value to write (4 bytes)
uint8_t flashWrite(uint32_t address, uint32_t data);

// Description....: Writing memory buffer to microprocessor memory
// uint32_t address = Cell address;
// uint32_t pbuffer = Start address of buffer memory
// uint8_t len ??= Number of buffer bytes
void flashWriteBuffer(uint32_t address, uint32_t pbuffer, uint8_t len);

// Description....: Reading to microprocessor memory buffer
// uint32_t address = Cell address;
// uint32_t pbuffer = Start address of buffer memory
// uint8_t len ??= Number of buffer bytes
void flashReadBuffer(uint32_t address, uint32_t pbuffer, uint8_t len);


#endif

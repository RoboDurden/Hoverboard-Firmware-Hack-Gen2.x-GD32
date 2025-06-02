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

#include "../Inc/target.h"


# define TRUE												0x01
# define FALSE												0x00
	
//----------------------------------------------------------------------------
// Send buffer via USART
//----------------------------------------------------------------------------
void SendBuffer(uint32_t usart_periph, uint8_t buffer[], uint8_t length)
{
	uint8_t index = 0;
	
	for(; index < length; index++)
	{
    usart_data_transmit(usart_periph, buffer[index]);
    while (usart_flag_get(usart_periph, USART_FLAG_TC) == RESET) {}
	}
}

//----------------------------------------------------------------------------
// Calculate CRC
//----------------------------------------------------------------------------
uint16_t CalcCRC(uint8_t *ptr, int count)
{
  uint16_t  crc;
  uint8_t i;
  crc = 0;
  while (--count >= 0)
  {
    crc = crc ^ (uint16_t) *ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
      {
        crc = crc << 1 ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    } while(--i);
  }
  return (crc);
}


/* Clears a page of microprocessor memory */
void flashErase(uint32_t address) {
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR);
	fmc_page_erase(address);
	fmc_lock();
}

/* Reads 4 bytes from microprocessor memory */
uint32_t flashRead(uint32_t address) {
	return *(uint32_t*)address;
}

/* Writes 4 bytes to microprocessor memory */
uint8_t flashWrite(uint32_t address, uint32_t data) {
	uint8_t fflash = FALSE;
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR); 
	if (fmc_word_program(address, data) == FMC_READY) fflash = TRUE; 
	fmc_lock(); 
	return fflash;
}

/* Write a memory buffer to the microprocessor memory */
void flashWriteBuffer(uint32_t address, uint32_t pbuffer, uint8_t len) { 
	int16_t icount = 0; 
	while (1) { 
		if (flashWrite(address+icount, *(uint32_t*)(pbuffer + icount)) == TRUE) { 
		if (icount > len) break; 
		icount = icount + 4; 
		}
	}
}

/* Read into the microprocessor memory buffer */
void flashReadBuffer(uint32_t address, uint32_t pbuffer, uint8_t len) {
	int16_t icount = 0;
	while (1) {
		*(uint32_t*)(pbuffer + icount) = *(uint32_t*)(address+icount);
		if (icount > len) break;
		icount = icount + 4;
	}
}


/*
copied from https://github.com/GreenBytes95/KickScooter

    #pragma pack(push, 1)
	typedef struct {
		uint8_t     phase;
...
        uint8_t     bitByte;
        uint32_t    bitData;
	} config_t;
    #pragma pack(pop)

// Number of Flash Memory
# define FLASH_MAX 0xFA00
// Start Address of Flash Memory
# define FLASH_ADRESS 0x08000000

void confRead(void) {
    if (flashRead((FLASH_ADRESS + FLASH_MAX) - (sizeof(config) + 4)) == 0xFFFFFFFF) {
        confDefine();
        confWrite();
    }
    flashReadBuffer((FLASH_ADRESS + FLASH_MAX) - (sizeof(config) + 4), (uint32_t)&config, sizeof(config) - 4);
}

void confWrite(void) {
    flashErase((FLASH_ADRESS + FLASH_MAX) - (sizeof(config) + 4));
    flashWriteBuffer((FLASH_ADRESS + FLASH_MAX) - (sizeof(config) + 4), (uint32_t)&config, sizeof(config) - 4);
}

void confErase(void) {
    flashErase((FLASH_ADRESS + FLASH_MAX) - (sizeof(config) + 4));
}




*/

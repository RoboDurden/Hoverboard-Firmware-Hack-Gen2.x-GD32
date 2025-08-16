/**
  * This file was part of the hoverboard-sideboard-hack project but got re-written
  * to remove the interrupts and use a blocking I2C implementation.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  * Copyright (C) 2025 Hoverboard Havoc
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

// Define to prevent recursive inclusion
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define I2C_SPEED			100000      // [bit/s] Define I2C speed for communicating with the MPU6050
#define I2C_PERIPH 		I2C0
#define MPU_RCU_I2C		RCU_I2C0


#define I2C_OWN_ADDRESS7            0x24


#define I2C_TIMEOUT  10000
#ifndef I2C_ACK_ENABLE
	#define I2C_ACK_ENABLE  1
	#define I2C_ACK_DISABLE 0
#endif

// return values
#define I2C_OK    0
#define I2C_ERR  -1

// diagnostic tools
int8_t i2c_scanner(void);
void dump_i2c_registers(uint8_t slaveAddr) ;

void I2C_Init(void);

/* i2c write/read functions */
int8_t i2c_writeBytes(uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t i2c_writeByte (uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
int8_t i2c_writeBit  (uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t  data);
int8_t i2c_readBytes (uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t i2c_readByte  (uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data);
int8_t i2c_readBit   (uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

#endif // I2C_H


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

// Includes
#include <stdio.h>
#include <string.h>
#include "gd32f1x0.h"
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "i2c.h"
#include "target.h"

#ifdef I2C_ENABLE

//------------------------------------------------------------------------------
// Blocking write of N bytes to device @devAddr, register @regAddr.
//------------------------------------------------------------------------------
int8_t i2c_writeBytes(
                      uint32_t i2c_periph,
                      uint8_t devAddr,
                      uint8_t regAddr,
                      uint8_t length,
                      uint8_t *data)
{
    uint32_t tmo = 0;

    // 1) wait for bus idle
    while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY)) {
        if (++tmo > I2C_TIMEOUT) return I2C_ERR;
    }

    // 2) send START
    i2c_start_on_bus(i2c_periph);
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)) {
        if (++tmo > I2C_TIMEOUT) return I2C_ERR;
    }

    // 3) send slave address + write bit
    //     — GD32 uses I2C_TRANSMITTER / I2C_RECEIVER
    i2c_master_addressing(i2c_periph, devAddr << 1, I2C_TRANSMITTER);
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)) {
        if (++tmo > I2C_TIMEOUT) {
            i2c_stop_on_bus(i2c_periph);
            return I2C_ERR;
        }
    }
    // clear the address‐sent flag
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

    // 4) send register address
    i2c_data_transmit(i2c_periph, regAddr);
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
        if (++tmo > I2C_TIMEOUT) {
            i2c_stop_on_bus(i2c_periph);
            return I2C_ERR;
        }
    }

    // 5) send payload bytes
    for (uint8_t i = 0; i < length; i++) {
        i2c_data_transmit(i2c_periph, data[i]);
        tmo = 0;
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
            if (++tmo > I2C_TIMEOUT) {
                i2c_stop_on_bus(i2c_periph);
                return I2C_ERR;
            }
        }
    }

    // 6) wait for transfer complete, then STOP
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_BTC)) {
        if (++tmo > I2C_TIMEOUT) {
            i2c_stop_on_bus(i2c_periph);
            return I2C_ERR;
        }
    }
    i2c_stop_on_bus(i2c_periph);

    return I2C_OK;
}


/*
 * write 1 byte to chip register
 */
int8_t i2c_writeByte(uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    return i2c_writeBytes(i2c_periph, slaveAddr, regAddr, 1, &data);
}


/*
 * write one bit to chip register
 */
int8_t i2c_writeBit(uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    i2c_readByte(i2c_periph, slaveAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2c_writeByte(i2c_periph, slaveAddr, regAddr, b);
}



/* =========================== I2C READ Functions =========================== */

//------------------------------------------------------------------------------
// Blocking read of N bytes into data[] from device @devAddr, register @regAddr.
//------------------------------------------------------------------------------
int8_t i2c_readBytes(uint32_t i2c_periph,
                     uint8_t devAddr,
                     uint8_t regAddr,
                     uint8_t length,
                     uint8_t *data)
{
    uint32_t tmo = 0;

    // 1) wait for bus idle
    while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY)) {
        if (++tmo > I2C_TIMEOUT) return I2C_ERR;
    }

    // 2) send START + slave addr (write) + regAddr
    i2c_start_on_bus(i2c_periph);
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)) {
        if (++tmo > I2C_TIMEOUT) return I2C_ERR;
    }

    i2c_master_addressing(i2c_periph, devAddr << 1, I2C_TRANSMITTER);
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)) {
        if (++tmo > I2C_TIMEOUT) {
            i2c_stop_on_bus(i2c_periph);
            return I2C_ERR;
        }
    }
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

    i2c_data_transmit(i2c_periph, regAddr);
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
        if (++tmo > I2C_TIMEOUT) {
            i2c_stop_on_bus(i2c_periph);
            return I2C_ERR;
        }
    }

    // 3) repeated-start for read
    i2c_start_on_bus(i2c_periph);
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)) {
        if (++tmo > I2C_TIMEOUT) return I2C_ERR;
    }

    i2c_master_addressing(i2c_periph, devAddr << 1, I2C_RECEIVER);
    tmo = 0;
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)) {
        if (++tmo > I2C_TIMEOUT) {
            i2c_stop_on_bus(i2c_periph);
            return I2C_ERR;
        }
    }
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

    // 4) read each byte
    for (uint8_t i = 0; i < length; i++) {
        // on last byte, disable ACK and send STOP
        if (i == length - 1) {
            i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
            i2c_stop_on_bus(i2c_periph);
        }
        tmo = 0;
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE)) {
            if (++tmo > I2C_TIMEOUT) return I2C_ERR;
        }
        data[i] = i2c_data_receive(i2c_periph);
    }

    // re-enable ACK for next time
    i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);

    return I2C_OK;
}


/*
 * read 1 byte from chip register
 */
int8_t i2c_readByte(uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data)
{
    return i2c_readBytes(i2c_periph, slaveAddr, regAddr, 1, data);
}


/*
 * read 1 bit from chip register
 */
int8_t i2c_readBit(uint32_t i2c_periph, uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t b;
    int8_t status = i2c_readByte(i2c_periph, slaveAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return status;
}

#endif // I2C_ENABLE
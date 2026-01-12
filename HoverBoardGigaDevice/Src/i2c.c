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
#include "../Inc/defines.h"
#include "../Inc/it.h"	// for Delay in dump_i2c_registers(uint8_t slaveAddr)
#include "../Inc/i2c.h"

#ifdef I2C_ENABLE

//#define I2C_OLD
#ifdef I2C_OLD
void I2C_Init() {
    /* I2C clock configure */
     i2c_clock_config(I2C_PERIPH, I2C_SPEED, I2C_DTCY_16_9);            // I2C duty cycle in fast mode plus
    /* I2C address configure */
    i2c_mode_addr_config(I2C_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C_OWN_ADDRESS7);
    /* enable I2C */
    i2c_enable(I2C_PERIPH);
    /* enable acknowledge */
    i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
}
#else
void I2C_Init()
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(MPU_RCU_I2C);

	#ifdef I2C_PB6PB7	
		// Configure PB6 (SCL) and PB7 (SDA) as AF open-drain
		gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6 | GPIO_PIN_7);
		gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
		gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_6 | GPIO_PIN_7);  // AF1 for I2C0
	#else
		// Configure PB8 (SCL) and PB9 (SDA) as AF open-drain
	  #if TARGET == 2 // GD32/STM32F103
		rcu_periph_clock_enable(RCU_AF);        // Alternate Function clock
		gpio_pin_remap_config(GPIO_I2C0_REMAP, ENABLE); // JW: Remap I2C0 to PB8 and PB9
		gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
	  #else
		gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8 | GPIO_PIN_9);
		gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
		gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_8 | GPIO_PIN_9);  // AF1 for I2C0
	  #endif
	#endif	
	
	i2c_deinit(I2C_PERIPH);
	i2c_clock_config(I2C_PERIPH, I2C_SPEED, I2C_DTCY_16_9);            // I2C duty cycle in fast mode plus
	i2c_mode_addr_config(I2C_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C_OWN_ADDRESS7);
	i2c_enable(I2C_PERIPH);
	i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
}
#endif

void i2c_hardReset(uint32_t i2c_periph) {
	// 1. Force the I2C peripheral into reset state
	i2c_software_reset_config(i2c_periph, I2C_SRESET_SET);
	for(volatile int d=0; d<1000; d++);
	i2c_software_reset_config(i2c_periph, I2C_SRESET_RESET);

	// 2. Re-initialize the I2C peripheral from scratch
	I2C_Init();
}

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
            i2c_hardReset(i2c_periph);
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


int8_t iFound = -1;
int8_t aiFound[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};		// room for 10 defices found. Monitor with StmStudio
uint8_t i2c_scanner(void) 
{
	iFound = 0;
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) 
	{
		int8_t result = i2c_writeByte(I2C_PERIPH, addr, 0x00, 0x00);	// Try to write a dummy byte to the device
		if (result == 0) // Success means device ACK'd
		{
            if (iFound <10) aiFound[iFound] = addr;
            iFound++;
            RTT_PRINTF2(64,"\n%i found at: 0x%02X\n",iFound,addr)
		}
		for (volatile int i = 0; i < 1000; i++);	// Short delay between probes (adjust based on system clock)
	}
    RTT_PRINTF(64,"I2c scan complete. Found: %i\n",iFound)
	return iFound>0 ? aiFound[(iFound-1)] : 0;	//printf("Scan complete. Found %d device(s).\r\n", found);
}

#define MAX_REGISTERS 0x88
int16_t aiDump[MAX_REGISTERS];	// monitor with StmStudio
void dump_i2c_registers(uint8_t slaveAddr) 
{
	uint8_t data;
	int8_t result;
	for (uint8_t regAddr= 0; regAddr< MAX_REGISTERS; regAddr++) 
	{
		result = i2c_readByte(I2C_PERIPH, slaveAddr, regAddr, &data);
		aiDump[regAddr] = (result == 0) ? data : -1;
        //RTT_PRINTF2(32,"%02X = %02X\n",regAddr,aiDump[regAddr])
        RTT_PRINTF2(32,((regAddr%16<15) ? "%02X=%02X  " : "%02X: %02X\n"),regAddr,aiDump[regAddr])
		Delay(1);
	}
}

#endif // I2C_ENABLE
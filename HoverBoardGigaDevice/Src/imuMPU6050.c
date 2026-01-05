/**
  * This is a driver for an MPU-6050 or a compatible clone.
  * It has been modified to detect non-standard WHO_AM_I values,
  * specifically 0x45, as found on some devices.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  * Copyright (C) 2011-2022 InvenSense Corporation, All Rights Reserved.
  * Copyright (C) 2025 Hoverboard Havoc
  * * This program is free software: you can redistribute it and/or modify
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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../Inc/defines.h"
#include "../Inc/it.h" // for Delay()
#include "../Inc/mpu6050.h"

#ifdef MPU_6050

extern uint32_t iBug;

MPU_Data mpuData;                                       // holds the MPU data

int32_t aiLowPass[sizeof(mpuData)/2];		// mpuData only contain 16bit integers !
void DoLowPass(uint8_t iFilterShift, int16_t i, int32_t* aiState, int16_t* aiNew, uint8_t iShiftRight)
{
	for (i--; i >=0; i--)
	{
		aiState[i] = aiState[i] - (aiState[i] >> iFilterShift) + aiNew[i];  		// Calculate low-pass filter for new value
		aiNew[i] = aiState[i] >> (iFilterShift+iShiftRight);		// replace new value with low pass value
	}
}


ErrStatus    mpuStatus;                  // holds the MPU status: SUCCESS or ERROR

int mpu_config(void);

int MPU_Init() {
	
			memset(aiLowPass,0,sizeof(mpuData)*2);		// aiLowPass has int32_t so double the size of mpuData
		
       int result = mpu_config();
       if(result) {                              // IMU MPU-6050 config
            mpuStatus = ERROR;
            digitalWrite(LED_RED,SET);
            digitalWrite(LED_GREEN,RESET);
        }
        else {
            mpuStatus = SUCCESS;
            digitalWrite(LED_RED, RESET);
            digitalWrite(LED_GREEN,SET);
        }
        return result;
}

/* Hardware registers needed by driver. */
struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_accel_odr;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
};

/* Information specific to a particular device. */
struct hw_s {
    unsigned char addr;
    unsigned short max_fifo;
    unsigned char num_reg;
    unsigned short temp_sens;
    short temp_offset;
    unsigned short bank_size;
};

/* Gyro driver state variables. */
struct gyro_state_s {
    const struct gyro_reg_s *reg;
    const struct hw_s *hw;
};

/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

#define BIT_RESET           (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_DATA_RDY_EN     (0x01)

const struct gyro_reg_s reg = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1A, // Set to 0x03
    .user_ctrl      = 0x6A,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1B, // Set to 0x00  GYRO_CONFIG: FS_SEL = 0 → ±250 °/s
    .accel_cfg      = 0x1C, // // ACCEL_CONFIG: AFS_SEL = 0 → ±2 g
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3B,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .pwr_mgmt_1     = 0x6B, // PWR_MGMT_1: clear sleep bit
    .pwr_mgmt_2     = 0x6C,
    .int_pin_cfg    = 0x37,
};
const struct hw_s hw = {
    .addr           = 0x68,
    .max_fifo       = 1024,
    .num_reg        = 118,
    .temp_sens      = 340,
    .temp_offset    = -521,
    .bank_size      = 256
};

static struct gyro_state_s st = {
    .reg = &reg,
    .hw = &hw
};


int mpu_config(void)
{
    int8_t rc;
    uint8_t who_am_i;

    // 0) Check WHO_AM_I register
    rc = i2c_readBytes(MPU_I2C, hw.addr, reg.who_am_i, 1, &who_am_i);
    if (rc) return -1; // I2C read failed

    // Accept standard MPU6050 (0x68) or the clone ID (0x45)
    if (who_am_i != 0x68 && who_am_i != 0x45) {
        return -2; // Chip ID mismatch
    }

    // 1) Reset the device and wait for it to come back up.
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.pwr_mgmt_1, BIT_RESET);
    if (rc) return rc;
    Delay(100); // Wait for reset to complete

    // 2) Wake up device and set clock source to PLL.
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.pwr_mgmt_1, INV_CLK_PLL);
    if (rc) return rc;
		
    // 3) Explicitly power on all sensor axes by writing 0 to PWR_MGMT_2.
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.pwr_mgmt_2, 0x00);
    if (rc) return rc;
    Delay(10); // small delay for settings to apply

    // 4) Disable DMP & FIFO
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.user_ctrl, 0);
    if (rc) return rc;
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.fifo_en, 0);
    if (rc) return rc;

    // 5) Sample rate divider: 1 kHz/(1 + DIV) → here DIV = 4 → 200 Hz
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.rate_div, 4);
    if (rc) return rc;

    // 6) Configure DLPF_CFG to 42 Hz
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.lpf, INV_FILTER_42HZ);
    if (rc) return rc;

    // 7) Full-scale ranges
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.gyro_cfg, (INV_FSR_250DPS << 3));
    if (rc) return rc;
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.accel_cfg, (INV_FSR_2G << 3));
    if (rc) return rc;

    // 8) INT pin: active-high, push-pull, latch until cleared
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.int_pin_cfg, BIT_LATCH_EN | BIT_ANY_RD_CLR);
    if (rc) return rc;

    // 9) Enable only the Data Ready interrupt
    rc = i2c_writeByte(MPU_I2C, hw.addr, reg.int_enable, BIT_DATA_RDY_EN);
    if (rc) return rc;

    return 0;
}




int MPU_ReadAll()
{
	if (SUCCESS == mpuStatus) 
	{
		uint8_t buf[14];

		// burst‐read accel(6) + temp(2) + gyro(6)
		if (i2c_readBytes(MPU_I2C, st.hw->addr, st.reg->raw_accel, 14, buf) != I2C_OK) 
		{	// handle I2C error here if you need to
				return ERROR;
		}

		// unpack accel
		mpuData.accel.x = (int16_t)((buf[0] << 8) | buf[1]);
		mpuData.accel.y = (int16_t)((buf[2] << 8) | buf[3]);
		mpuData.accel.z = (int16_t)((buf[4] << 8) | buf[5]);

		// unpack temp
		mpuData.temperature    = (int16_t)((buf[6] << 8) | buf[7]);

		// unpack gyro
		mpuData.gyro.x  = (int16_t)((buf[8]  << 8) | buf[9]);
		mpuData.gyro.y  = (int16_t)((buf[10] << 8) | buf[11]);
		mpuData.gyro.z  = (int16_t)((buf[12] << 8) | buf[13]);
		
	#if IMU_LP > 0
		DoLowPass(IMU_LP, sizeof(mpuData)/2, aiLowPass, (int16_t*) &mpuData,0);
	#endif
	}
	return mpuStatus;
}

#endif // CUSTOM_IMU


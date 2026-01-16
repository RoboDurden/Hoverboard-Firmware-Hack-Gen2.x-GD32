/**
  * This is a driver for the Bosch BMI160 IMU, adapted from an MPU6050 driver.
  * It provides the same interface for initialization and data reading.
  *
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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../Inc/defines.h"
#include "../Inc/it.h" // for Delay()
#include "../Inc/mpu6050.h" // Using the same header for MPU_Data struct

#ifdef BMI_160 // Make sure to define BMI_160 and undefine MPU_6050 in your project settings/defines.h

// The I2C address for the BMI160
#define BMI160_I2C_ADDR         0x68 // Or 0x69 if SDO pin is high

// BMI160 Register Map
#define BMI160_CHIP_ID_ADDR     0x00
#define BMI160_DATA_ADDR        0x0C // Start of Gyro data
#define BMI160_TEMP_ADDR        0x20
#define BMI160_ACC_CONF_ADDR    0x40
#define BMI160_ACC_RANGE_ADDR   0x41
#define BMI160_GYR_CONF_ADDR    0x42
#define BMI160_GYR_RANGE_ADDR   0x43
#define BMI160_CMD_REG_ADDR     0x7E

// BMI160 Commands
#define BMI160_SOFT_RESET_CMD   0xB6
#define BMI160_ACC_NORMAL_CMD   0x11
#define BMI160_GYR_NORMAL_CMD   0x15

// BMI160 Expected Chip ID
#define BMI160_CHIP_ID          0xD1

extern uint32_t iBug;

MPU_Data mpuData; // holds the BMI160 data, using the same shared struct

int32_t aiLowPass[sizeof(mpuData)/2]; // mpuData only contains 16bit integers !
void DoLowPass(uint8_t iFilterShift, int16_t i, int32_t* aiState, int16_t* aiNew, uint8_t iShiftRight)
{
    for (i--; i >=0; i--)
    {
        aiState[i] = aiState[i] - (aiState[i] >> iFilterShift) + aiNew[i];      // Calculate low-pass filter for new value
        aiNew[i] = aiState[i] >> (iFilterShift+iShiftRight);        // replace new value with low pass value
    }
}

ErrStatus mpuStatus; // holds the BMI160 status: SUCCESS or ERROR

int mpu_config(void);

/**
 * @brief Initializes the BMI160 sensor.
 * @return 0 on success, non-zero on failure.
 */
int MPU_Init() {
    memset(aiLowPass, 0, sizeof(mpuData) * 2); // aiLowPass has int32_t so double the size of mpuData

    int result = mpu_config();
    if (result) { // IMU BMI160 config failed
        mpuStatus = ERROR;
        digitalWrite(LED_RED, SET);
        digitalWrite(LED_GREEN, RESET);
    } else {
        mpuStatus = SUCCESS;
        digitalWrite(LED_RED, RESET);
        digitalWrite(LED_GREEN, SET);
    }
    return result;
}

/**
 * @brief Configures the BMI160 with default settings.
 * @return 0 on success, non-zero on failure.
 */
int mpu_config(void) {
    uint8_t chip_id = 0;
    int8_t rc;

    // 1) Check Chip ID
    rc = i2c_readBytesTimeout(IMU_TIMEOUT_MS, MPU_I2C, BMI160_I2C_ADDR, BMI160_CHIP_ID_ADDR, 1, &chip_id);
    if (rc || chip_id != BMI160_CHIP_ID) {
        return -1; // Error: incorrect chip ID or I2C failure
    }
    Delay(10);

    // 2) Soft reset the sensor
    rc = i2c_writeByte(MPU_I2C, BMI160_I2C_ADDR, BMI160_CMD_REG_ADDR, BMI160_SOFT_RESET_CMD);
    if (rc) return rc;
    Delay(50); // Wait for the sensor to reset

    // 3) Set accelerometer and gyroscope to normal power mode
    rc = i2c_writeByte(MPU_I2C, BMI160_I2C_ADDR, BMI160_CMD_REG_ADDR, BMI160_ACC_NORMAL_CMD);
    if (rc) return rc;
    Delay(50);

    rc = i2c_writeByte(MPU_I2C, BMI160_I2C_ADDR, BMI160_CMD_REG_ADDR, BMI160_GYR_NORMAL_CMD);
    if (rc) return rc;
    Delay(50);

    // 4) Configure accelerometer
    // ODR = 200Hz (0b1000), Bandwidth = Normal (0b010 << 4) -> 0x28
    rc = i2c_writeByte(MPU_I2C, BMI160_I2C_ADDR, BMI160_ACC_CONF_ADDR, 0x28);
    if (rc) return rc;
    // Range = +/- 2g (0b0011)
    rc = i2c_writeByte(MPU_I2C, BMI160_I2C_ADDR, BMI160_ACC_RANGE_ADDR, 0x03);
    if (rc) return rc;

    // 5) Configure gyroscope
    // ODR = 200Hz (0b1000), Bandwidth = Normal (0b010 << 4) -> 0x28
    rc = i2c_writeByte(MPU_I2C, BMI160_I2C_ADDR, BMI160_GYR_CONF_ADDR, 0x28);
    if (rc) return rc;
    // Range = +/- 250 dps (0b0100), the closest to MPU6050's 250dps
    rc = i2c_writeByte(MPU_I2C, BMI160_I2C_ADDR, BMI160_GYR_RANGE_ADDR, 0x04);
    if (rc) return rc;

    return 0; // Success
}

/**
 * @brief Reads all sensor data from the BMI160.
 * @return SUCCESS or ERROR.
 */
int MPU_ReadAll() {
    if (SUCCESS == mpuStatus) {
        uint8_t buf[12]; // Buffer for 6 axes (12 bytes)

        // Burst-read gyro(6) + accel(6)
        if (i2c_readBytes(MPU_I2C, BMI160_I2C_ADDR, BMI160_DATA_ADDR, 12, buf) != I2C_OK) {
            // handle I2C error here if you need to
            return ERROR;
        }

        // Unpack gyro data (little-endian)
        mpuData.gyro.x = (int16_t)((buf[1] << 8) | buf[0]);
        mpuData.gyro.y = (int16_t)((buf[3] << 8) | buf[2]);
        mpuData.gyro.z = (int16_t)((buf[5] << 8) | buf[4]);

        // Unpack accel data (little-endian)
        mpuData.accel.x = (int16_t)((buf[7] << 8) | buf[6]);
        mpuData.accel.y = (int16_t)((buf[9] << 8) | buf[8]);
        mpuData.accel.z = (int16_t)((buf[11] << 8) | buf[10]);

        // Read temperature separately
        uint8_t temp_buf[2];
        if (i2c_readBytes(MPU_I2C, BMI160_I2C_ADDR, BMI160_TEMP_ADDR, 2, temp_buf) != I2C_OK) {
            return ERROR;
        }

        // Unpack temperature data (little-endian)
        mpuData.temperature = (int16_t)((temp_buf[1] << 8) | temp_buf[0]);
        // Note: To convert BMI160 temperature to °C: T_celsius = 23 + (raw_temp / 512.0)
        // Here we just store the raw value as the struct expects.

	#if IMU_LP > 0
        // Apply low-pass filter
        DoLowPass(IMU_LP, sizeof(mpuData)/2, aiLowPass, (int16_t*) &mpuData, 0);
	#endif

        return SUCCESS;
    }
    return ERROR;
}

#endif // BMI_160

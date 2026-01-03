/**
  * Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  * Copyright (C) 2025 Hoverboard Havoc
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  * Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
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
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "defines.h"
#include "../Inc/i2c.h"

#define MPU_I2C                     I2C_PERIPH

#ifndef IMU_LP
  #define IMU_LP 4
#endif

int MPU_Init();
int MPU_ReadAll();

#endif // MPU6050_H


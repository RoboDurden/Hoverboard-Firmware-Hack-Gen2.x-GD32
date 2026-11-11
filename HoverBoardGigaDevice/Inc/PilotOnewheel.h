/*
* This file is part of the Hoverboard-Firmware-Hack-Gen2.x-GD32 project.
*
* Copyright (C) 2026 Damian Krawczyk
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

#ifndef PILOT_ONEWHEEL_H
#define PILOT_ONEWHEEL_H

// PilotOnewheel: self-balancing (pitch) pilot for a one-wheel style vehicle.
// The MASTER board reads its onboard IMU, runs the balance loop and drives its own motor.
// The same motor command is sent to the SLAVE board over the master-slave uart,
// so both motors are controlled from the one IMU on the master board.

#include <stdint.h>

// ---------------- safety / bring-up ----------------

//#define ONEWHEEL_MOTORS_OFF		// FIRST FLASH: motors stay disabled, only the angles are calculated and logged.
								// Check the axis mapping below, then comment this line out.

// ---------------- IMU mounting ----------------
// The pilot needs to know how the master board is mounted: which imu axis points up and which points to the tail.
// All gyro and accelerometer axes and signs follow from these two numbers (pitch positive = nose down).
//
// SETUP WITH THE BUZZER: with ONEWHEEL_BEEP_AXIS the motors stay off and the buzzer repeats every 4 seconds:
//   fast ticking         = measuring the gyro offsets, hold the board still
//   one long beep        = imu not found
//   1 to 6 short beeps   = number of the imu axis that points up right now
// 1. hold the board level (as when riding)            -> count the beeps = ONEWHEEL_AXIS_UP
// 2. stand the board on its nose (nose down, tail up) -> count the beeps = ONEWHEEL_AXIS_TAIL
// 3. enter both numbers below, comment out ONEWHEEL_BEEP_AXIS and flash again.
//#define ONEWHEEL_BEEP_AXIS
#define ONEWHEEL_AXIS_UP		6		// 1=+x 2=-x 3=+y 4=-y 5=+z 6=-z
#define ONEWHEEL_AXIS_TAIL		2		// must be a different axis than ONEWHEEL_AXIS_UP

// ---------------- motor direction ----------------
// Both motors drive the same wheel, so they MUST push in the same direction or they fight each other.
// Find the signs with ONEWHEEL_TEST_DIR before the first balance test:
//   wheel lifted. Every 2 seconds: master motor alone -> pause -> slave motor alone -> pause   (low pwm, no balancing)
//   The wheel must turn the SAME way in both phases, and that way must be 'towards the nose'.
//   If the two phases turn opposite ways, flip ONEWHEEL_DIR_SLAVE. If both turn towards the tail, flip both.
//#define ONEWHEEL_TEST_DIR				// uncomment for the direction test (works with or without ONEWHEEL_MOTORS_OFF)
#define ONEWHEEL_TEST_PWM		80		// of 1000. The idle motor brakes a little, so the wheel turns slowly.
#define ONEWHEEL_DIR_MASTER		1		// 1 or -1
#define ONEWHEEL_DIR_SLAVE		1		// 1 or -1  (main.c already mirrors the slave like on a hoverboard)

// ---------------- balance controller (start values, need tuning) ----------------
// pwm goes from -1000 to +1000. All values can be changed at runtime with McuViewer/StmStudio (fOw.. variables)
#define ONEWHEEL_KP				15.0f	// pwm per degree of pitch
#define ONEWHEEL_KI				0.0f	// pwm per (degree * second)   , learns the pwm needed to hold a speed
										// 0 for the bench test with lifted wheel (motor speed follows the tilt). Try 30 when riding.
#define ONEWHEEL_KD				0.8f	// pwm per (degree/second)     , damping
#define ONEWHEEL_I_MAX			600.0f	// limit of the integral term in pwm
#define ONEWHEEL_PWM_MAX		300.0f	// limit of the motor command. 300 for the bench test, raise to 1000 when the board is tuned.

#define ONEWHEEL_FILTER_TAU		0.5f	// seconds, complementary filter: gyro below this time, accelerometer above

// ---------------- engage / disengage ----------------
#define ONEWHEEL_START_ANGLE	5.0f	// degrees: motors engage when pitch is closer to level than this ..
#define ONEWHEEL_START_ROLL		20.0f	// .. and roll is less than this ..
#define ONEWHEEL_START_MS		200		// .. for this time
#define ONEWHEEL_SOFTSTART_MS	400		// ramp up the controller output after engaging
#define ONEWHEEL_MAX_PITCH		35.0f	// degrees: motors off beyond this pitch
#define ONEWHEEL_MAX_ROLL		50.0f	// degrees: motors off beyond this roll (board fell on its side)
#define ONEWHEEL_IMU_ERRORS		5		// motors off after this many failed imu readings in a row (5 ms each)

// duty warning: when the motors run out of headroom the board can no longer catch the rider (nosedive)
#define ONEWHEEL_DUTY_WARN		0.8f	// fraction of ONEWHEEL_PWM_MAX: beep and raise the nose above this
#define ONEWHEEL_TILTBACK_ANGLE	6.0f	// degrees the nose is raised
#define ONEWHEEL_TILTBACK_SPEED	4.0f	// degrees per second

// rider detection with the two photo interrupters of the hoverboard foot pads (PHOTO_L / PHOTO_R)
//#define ONEWHEEL_RIDER_SENSOR			// uncomment: motors only engage when a foot pad is pressed
#define ONEWHEEL_RIDER_ACTIVE	0		// pin level when the pad is pressed (0 or 1), check with McuViewer: iOwPhotoL/iOwPhotoR
#define ONEWHEEL_RIDER_OFF_MS	300		// motors off when both pads are released for this time


void Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave);

extern uint8_t bOnewheelBeep;		// 1 = duty warning, main.c turns the buzzer on

#endif // PILOT_ONEWHEEL_H

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

// PilotOnewheel.c : self-balancing pilot for a one-wheel vehicle, see PilotOnewheel.h for the settings

#include "../Inc/defines.h"
#ifdef PILOT_ONEWHEEL

#if !defined(IMU_ENABLE)
	#error "PILOT_ONEWHEEL needs an imu (MPU_6050, MPU_6050old, MPU_6500 or BMI_160) in your defines_2-x-y.h"
#endif

#include "../Inc/it.h"
#include "../Inc/mpu6050.h"
#include <math.h>
#include <stdio.h>

extern uint8_t bPilotTimeout;		// 1 = disable bldc (with soft brake). Also disables the slave.
extern MPU_Data mpuData;
extern ErrStatus mpuStatus;

#define OW_GYRO_LSB		131.0f		// LSB per deg/s at +-250 deg/s
#define OW_RAD2DEG		57.29578f
#define OW_CALIB_COUNT	200			// gyro samples (5 ms each) averaged at startup
#define OW_CALIB_MOVE	(3*131)		// restart calibration if the board moves more than 3 deg/s

enum { OW_CALIBRATE = 0, OW_IDLE, OW_RUN };

// runtime tunable with McuViewer / StmStudio
float fOwKp = ONEWHEEL_KP;
float fOwKi = ONEWHEEL_KI;
float fOwKd = ONEWHEEL_KD;
float fOwPwmMax = ONEWHEEL_PWM_MAX;

// state, for logging
uint8_t iOwState = OW_CALIBRATE;
uint8_t iOwCalibRestarts = 0, iOwEngaged = 0;	// counters for ONEWHEEL_BLACKBOX
float fOwPitch = 0, fOwRoll = 0;		// degrees, complementary filter
float fOwPitchAcc = 0, fOwRollAcc = 0;	// degrees, accelerometer only
float fOwPitchRate = 0;					// deg/s
float fOwSetpoint = 0;					// degrees, moved by the duty tiltback
float fOwIntegral = 0;					// pwm
float fOwOut = 0;						// pwm -1000..1000
uint8_t iOwPhotoL = 0, iOwPhotoR = 0;
uint8_t bOnewheelBeep = 0;

static int32_t aiGyroSum[2];
static int16_t aiGyroFirst[2];
static int16_t iCalibCount = 0;
static float fGyroOffPitch = 0, fGyroOffRoll = 0;

static uint8_t bAngleInit = 0;
static uint8_t iImuErrors = 0;
static uint32_t iTimeLast = 0;
static uint32_t iTimeLevel = 0;		// since when the board is level enough to engage
static uint32_t iTimeEngage = 0;
static uint32_t iTimeRider = 0;		// last time a rider was detected

#if ((ONEWHEEL_AXIS_UP+1)/2) == ((ONEWHEEL_AXIS_TAIL+1)/2)
	#error "ONEWHEEL_AXIS_UP and ONEWHEEL_AXIS_TAIL must be two different imu axes"
#endif

// imu mounting as signed unit vectors in the imu frame. Axis code: 1=+x 2=-x 3=+y 4=-y 5=+z 6=-z
static int8_t aiUp[3], aiTail[3], aiSide[3];	// aiSide = tail x up = rotation axis of 'nose down'
static uint8_t bAxisInit = 0;

static void AxisVector(uint8_t iCode, int8_t* ai)
{
	ai[0] = ai[1] = ai[2] = 0;
	ai[(iCode-1)/2] = (iCode & 1) ? 1 : -1;
}
static void AxisInit(void)
{
	AxisVector(ONEWHEEL_AXIS_UP, aiUp);
	AxisVector(ONEWHEEL_AXIS_TAIL, aiTail);
	aiSide[0] = aiTail[1]*aiUp[2] - aiTail[2]*aiUp[1];
	aiSide[1] = aiTail[2]*aiUp[0] - aiTail[0]*aiUp[2];
	aiSide[2] = aiTail[0]*aiUp[1] - aiTail[1]*aiUp[0];
	bAxisInit = 1;
}
static int16_t Dot(const int16_t* aiImu, const int8_t* aiAxis)	// aiImu = x,y,z
{
	return aiImu[0]*aiAxis[0] + aiImu[1]*aiAxis[1] + aiImu[2]*aiAxis[2];
}

#ifdef ONEWHEEL_BEEP_AXIS
// setup mode: tells with the buzzer which imu axis points up, see PilotOnewheel.h
static void BeepAxis(uint32_t iNow)
{
	static uint8_t iBeeps = 0;
	uint32_t iT = iNow % 4000;

	if ((mpuStatus != SUCCESS) || (iImuErrors >= ONEWHEEL_IMU_ERRORS))	bOnewheelBeep = iT < 1500;	// one long beep
	else if (iOwState == OW_CALIBRATE)		bOnewheelBeep = (iNow % 200) < 50;		// ticking
	else
	{
		if (iT < 10)	// start of the 4 seconds: which axis measures +1g ?
		{
			const int16_t* ai = (const int16_t*)&mpuData.accel;
			uint8_t i, iMax = 0;
			for (i = 1; i < 3; i++)	if (ABS(ai[i]) > ABS(ai[iMax]))	iMax = i;
			iBeeps = 2*iMax + (ai[iMax] > 0 ? 1 : 2);
		}
		bOnewheelBeep = (iT < iBeeps*400u) && ((iT % 400) < 150);
	}
}
#endif

static uint8_t RiderPresent(void)
{
	#if defined(ONEWHEEL_RIDER_SENSOR) && defined(PHOTO_L) && defined(PHOTO_R)
		iOwPhotoL = digitalRead(PHOTO_L);
		iOwPhotoR = digitalRead(PHOTO_R);
		return (iOwPhotoL == ONEWHEEL_RIDER_ACTIVE) || (iOwPhotoR == ONEWHEEL_RIDER_ACTIVE);
	#else
		return 1;
	#endif
}

static void Disengage(void)
{
	iOwState = OW_IDLE;
	iTimeLevel = 0;
	fOwOut = fOwIntegral = fOwSetpoint = 0;
	bOnewheelBeep = 0;
}

// returns 0 while the gyro offsets are being measured. Board must not move.
static uint8_t CalibrateGyro(int16_t iGyroPitch, int16_t iGyroRoll)
{
	if (iCalibCount == 0)
	{
		aiGyroFirst[0] = iGyroPitch;
		aiGyroFirst[1] = iGyroRoll;
		aiGyroSum[0] = aiGyroSum[1] = 0;
	}
	else if (	(ABS(iGyroPitch - aiGyroFirst[0]) > OW_CALIB_MOVE) || (ABS(iGyroRoll - aiGyroFirst[1]) > OW_CALIB_MOVE)	)
	{
		iCalibCount = 0;	// board was moved, start again
		#ifdef ONEWHEEL_BLACKBOX
			if (iOwCalibRestarts < 255)	iOwCalibRestarts++;
		#endif
		return 0;
	}
	aiGyroSum[0] += iGyroPitch;
	aiGyroSum[1] += iGyroRoll;
	if (++iCalibCount < OW_CALIB_COUNT)
		return 0;

	fGyroOffPitch = (float)aiGyroSum[0] / OW_CALIB_COUNT;
	fGyroOffRoll = (float)aiGyroSum[1] / OW_CALIB_COUNT;
	return 1;
}

extern uint8_t wState;		// 1=ledGreen, 2=ledOrange, 4=ledRed. Without 32=Battery3Led the three leds show these bits

#ifdef ONEWHEEL_BLACKBOX
// Test without debugger: what the pilot saw is saved to flash 20 and 60 seconds after power on (battery power only),
// and can be read with the ST-Link later:  mdw 0x0800F800 12   (second last 1kB page of a 64kB mcu)
#pragma pack(push, 1)
typedef struct {
	uint32_t iMagic;			// 0x0B0C5E01
	uint32_t iUptime;			// ms
	int16_t iPitchMin, iPitchMax, iRollMin, iRollMax;	// degrees * 10
	int16_t iPwmMin, iPwmMax;
	int16_t aiAcc[3];			// last raw accelerometer x,y,z
	int16_t iBattery;			// volt * 10
	uint8_t iMaxState;			// 0 = calibrate, 1 = idle, 2 = run
	uint8_t iImuErrMax;
	uint8_t wHallSeen;			// bit n set = hall state n seen. Only bits 1-6 are valid states
	uint8_t wFlags;				// 1 = imu ok seen, 2 = imu not found seen, 4 = motor enabled seen, 8 = timeout seen
	uint8_t iEngaged;			// how often the pilot engaged
	uint8_t iCalibRestarts;		// how often the gyro calibration started again because the board moved
	uint16_t iLoops;			// pilot calls / 16
	int32_t iOdom;				// hall steps the motor has turned
	uint32_t iSlaveFrames;		// valid frames received from the slave
} BlackBox;
#pragma pack(pop)

static BlackBox oBox = { .iPitchMin = 32767, .iPitchMax = -32768, .iRollMin = 32767, .iRollMax = -32768, .iPwmMin = 32767, .iPwmMax = -32768 };
static uint8_t iBoxWritten = 0;
static uint16_t iBoxLoops = 0;
extern volatile uint8_t hall;
extern FlagStatus bldc_enable, timedOut;
extern int32_t iOdom;
extern float batteryVoltage;
void flashErase(uint32_t address);
void flashWriteBuffer(uint32_t address, uint8_t *pbuffer, uint16_t len);

static void BlackBoxUpdate(uint32_t iNow)
{
	iBoxLoops++;
	oBox.iLoops = iBoxLoops >> 4;
	oBox.iCalibRestarts = iOwCalibRestarts;
	oBox.iEngaged = iOwEngaged;
	oBox.wFlags |= (mpuStatus == SUCCESS) ? 1 : 2;
	if (bldc_enable == SET)	oBox.wFlags |= 4;
	if (timedOut == SET)	oBox.wFlags |= 8;
	oBox.wHallSeen |= 1 << (hall & 7);
	if (iOwState > oBox.iMaxState)		oBox.iMaxState = iOwState;
	if (iImuErrors > oBox.iImuErrMax)	oBox.iImuErrMax = iImuErrors;
	if (bAngleInit)
	{
		int16_t iPitch = fOwPitch * 10, iRoll = fOwRoll * 10, iPwm = fOwOut;
		oBox.iPitchMin = MIN(oBox.iPitchMin, iPitch);	oBox.iPitchMax = MAX(oBox.iPitchMax, iPitch);
		oBox.iRollMin = MIN(oBox.iRollMin, iRoll);		oBox.iRollMax = MAX(oBox.iRollMax, iRoll);
		oBox.iPwmMin = MIN(oBox.iPwmMin, iPwm);			oBox.iPwmMax = MAX(oBox.iPwmMax, iPwm);
	}

	if (	((iBoxWritten == 0) && (iNow > 20000)) || ((iBoxWritten == 1) && (iNow > 60000))	)
	{
		iBoxWritten++;
		if (batteryVoltage < 15)	// powered by the ST-Link only: keep the record of the last battery test
			return;
		oBox.iMagic = 0x0B0C5E01;
		oBox.iUptime = iNow;
		oBox.aiAcc[0] = mpuData.accel.x;	oBox.aiAcc[1] = mpuData.accel.y;	oBox.aiAcc[2] = mpuData.accel.z;
		oBox.iBattery = batteryVoltage * 10;
		oBox.iOdom = iOdom;
		#ifdef MASTER
			extern uint32_t iMasterSlaveFramesOk;
			oBox.iSlaveFrames = iMasterSlaveFramesOk;
		#endif
		flashErase(ONEWHEEL_BLACKBOX_ADDR);
		flashWriteBuffer(ONEWHEEL_BLACKBOX_ADDR, (uint8_t*)&oBox, sizeof(oBox));
	}
}
#endif

// shows the pilot state on the three leds (instead of the battery level) and logs to RTT
static void Log(uint32_t iNow)
{
	if (mpuStatus != SUCCESS)			wState = 4;			// red			: imu not found
	else if (iOwState == OW_CALIBRATE)	wState = 4 | 2;		// red + orange	: measuring gyro offsets, keep the board still
	else if (iOwState == OW_IDLE)		wState = 2;			// orange		: waiting for the board to be level
	else								wState = 1;			// green		: balancing

	#ifdef ONEWHEEL_BEEP_AXIS
		BeepAxis(iNow);
	#endif
	#ifdef ONEWHEEL_BLACKBOX
		BlackBoxUpdate(iNow);
	#endif

#ifdef RTT_REMOTE
	static uint32_t iTimeNextLog = 0;
	if (iNow < iTimeNextLog)
		return;
	iTimeNextLog = iNow + 100;

	char s[128];
	sprintf(s, "state: %d\tpitch: " F2IF "\taccP: " F2IF "\troll: " F2IF "\taccR: " F2IF "\tpwm: %d\timuErr: %d"
		#ifdef WINDOWS_RN
			"\r"
		#endif
		"\n",
		iOwState, F2I(fOwPitch), F2I(fOwPitchAcc), F2I(fOwRoll), F2I(fOwRollAcc), (int)fOwOut, iImuErrors);
	SEGGER_RTT_WriteString(0, s);
#endif
}

void Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave)		// called every DELAY_IN_MAIN_LOOP = 5 ms
{
	uint32_t iNow = millis();
	float fDt = (iNow - iTimeLast) * 0.001f;
	iTimeLast = iNow;
	fDt = CLAMP(fDt, 0.001f, 0.02f);

	*pPwmMaster = *pPwmSlave = 0;

	#ifdef ONEWHEEL_TEST_DIR
		// direction test: one motor at a time, so wrong signs can be seen without the motors fighting
		uint8_t iPhase = (iNow / 2000) % 4;		// 0 = master, 1 = pause, 2 = slave, 3 = pause
		iOwState = 10 + iPhase;
		bPilotTimeout = iPhase & 1;
		if (iPhase == 0)	*pPwmMaster = ONEWHEEL_TEST_PWM * ONEWHEEL_DIR_MASTER;
		if (iPhase == 2)	*pPwmSlave = ONEWHEEL_TEST_PWM * ONEWHEEL_DIR_SLAVE;
		return;
	#endif

	bPilotTimeout = 1;		// motors stay off unless everything below is fine

	// ---- read imu ----
	if (mpuStatus != SUCCESS)
	{
		// imu not found at startup: try again every second
		static uint32_t iTimeNextInit = 0;
		Disengage();
		iOwState = OW_CALIBRATE;
		iCalibCount = 0;
		bAngleInit = 0;
		if (iNow >= iTimeNextInit)
		{
			iTimeNextInit = iNow + 1000;
			I2C_Init();
			MPU_Init();
		}
		Log(iNow);
		return;
	}
	if (MPU_ReadAll() != SUCCESS)
	{
		I2C_Init();		// try to re-init if the i2c bus fails
		if (iImuErrors < 255)	iImuErrors++;
		if (iImuErrors >= ONEWHEEL_IMU_ERRORS)
		{
			Disengage();
			bAngleInit = 0;		// angles are stale, start again from the accelerometer
		}
		else if (iOwState == OW_RUN)	// short glitch: keep the last motor command
		{
			bPilotTimeout = 0;
			*pPwmMaster = (int16_t)fOwOut * ONEWHEEL_DIR_MASTER;
			*pPwmSlave = (int16_t)fOwOut * ONEWHEEL_DIR_SLAVE;
		}
		Log(iNow);
		return;
	}
	iImuErrors = 0;

	// pitch positive = nose down. The imu axes follow from ONEWHEEL_AXIS_UP and ONEWHEEL_AXIS_TAIL
	if (!bAxisInit)	AxisInit();
	const int16_t* aiGyro = (const int16_t*)&mpuData.gyro;
	const int16_t* aiAcc = (const int16_t*)&mpuData.accel;
	int16_t iGyroPitch = Dot(aiGyro, aiSide);
	int16_t iGyroRoll = -Dot(aiGyro, aiTail);
	float fAccFwd = Dot(aiAcc, aiTail);
	float fAccSide = Dot(aiAcc, aiSide);
	float fAccUp = Dot(aiAcc, aiUp);

	// The pilot does not wait for the gyro calibration: a board held in the hand never is still enough.
	// Without calibration the gyro offset (a few deg/s) only shifts the angle by about a degree.
	// The offsets are measured in the background as soon as the board rests for a second while the motors are off.
	static uint8_t bGyroCalibrated = 0;
	if (iOwState == OW_CALIBRATE)
		iOwState = OW_IDLE;
	if (!bGyroCalibrated && (iOwState == OW_IDLE))
		bGyroCalibrated = CalibrateGyro(iGyroPitch, iGyroRoll);

	// ---- angles: gyro for fast changes, accelerometer for the long term ----
	fOwPitchRate = (iGyroPitch - fGyroOffPitch) / OW_GYRO_LSB;
	float fRollRate = (iGyroRoll - fGyroOffRoll) / OW_GYRO_LSB;

	fOwPitchAcc = atan2f(fAccFwd, fAccUp) * OW_RAD2DEG;
	fOwRollAcc = atan2f(fAccSide, fAccUp) * OW_RAD2DEG;

	if (!bAngleInit)
	{
		bAngleInit = 1;
		fOwPitch = fOwPitchAcc;
		fOwRoll = fOwRollAcc;
	}
	else
	{
		// only trust the accelerometer when it measures about 1g (not while accelerating or bumping)
		float fG2 = (fAccFwd*fAccFwd + fAccSide*fAccSide + fAccUp*fAccUp) / (16384.0f*16384.0f);
		float fAlpha = ((fG2 > 0.6f*0.6f) && (fG2 < 1.4f*1.4f)) ? fDt / (ONEWHEEL_FILTER_TAU + fDt) : 0;
		fOwPitch += fOwPitchRate * fDt;
		fOwPitch += fAlpha * (fOwPitchAcc - fOwPitch);
		fOwRoll += fRollRate * fDt;
		fOwRoll += fAlpha * (fOwRollAcc - fOwRoll);
	}

	uint8_t bRider = RiderPresent();
	if (bRider)	iTimeRider = iNow;

	// ---- engage / disengage ----
	if (iOwState == OW_IDLE)
	{
		if (bRider && (ABS(fOwPitch) < ONEWHEEL_START_ANGLE) && (ABS(fOwRoll) < ONEWHEEL_START_ROLL))
		{
			if (!iTimeLevel)	iTimeLevel = iNow;
			if (iNow - iTimeLevel >= ONEWHEEL_START_MS)
			{
				iOwState = OW_RUN;
				iTimeEngage = iNow;
				#ifdef ONEWHEEL_BLACKBOX
					if (iOwEngaged < 255)	iOwEngaged++;
				#endif
				fOwIntegral = fOwSetpoint = 0;
			}
		}
		else	iTimeLevel = 0;
	}
	else if (iOwState == OW_RUN)
	{
		if (	(ABS(fOwPitch) > ONEWHEEL_MAX_PITCH) || (ABS(fOwRoll) > ONEWHEEL_MAX_ROLL)
				|| (iNow - iTimeRider > ONEWHEEL_RIDER_OFF_MS)	)
			Disengage();
	}

	// ---- balance controller ----
	if (iOwState == OW_RUN)
	{
		float fError = fOwPitch - fOwSetpoint;

		fOwIntegral += fOwKi * fError * fDt;
		fOwIntegral = CLAMP(fOwIntegral, -ONEWHEEL_I_MAX, ONEWHEEL_I_MAX);

		float fOut = fOwKp * fError + fOwIntegral + fOwKd * fOwPitchRate;

		uint32_t iSinceEngage = iNow - iTimeEngage;
		if (iSinceEngage < ONEWHEEL_SOFTSTART_MS)
			fOut *= (float)iSinceEngage / ONEWHEEL_SOFTSTART_MS;

		fOwOut = CLAMP(fOut, -fOwPwmMax, fOwPwmMax);

		// duty warning: beep and raise the nose so the rider slows down before the motors saturate
		float fTarget = 0;
		bOnewheelBeep = 0;
		if (ABS(fOwOut) > ONEWHEEL_DUTY_WARN * fOwPwmMax)
		{
			bOnewheelBeep = 1;
			fTarget = fOwOut > 0 ? -ONEWHEEL_TILTBACK_ANGLE : ONEWHEEL_TILTBACK_ANGLE;
		}
		float fStep = ONEWHEEL_TILTBACK_SPEED * fDt;
		fOwSetpoint += CLAMP(fTarget - fOwSetpoint, -fStep, fStep);

		#if !defined(ONEWHEEL_MOTORS_OFF) && !defined(ONEWHEEL_BEEP_AXIS)
			bPilotTimeout = 0;
			*pPwmMaster = (int16_t)fOwOut * ONEWHEEL_DIR_MASTER;
			*pPwmSlave = (int16_t)fOwOut * ONEWHEEL_DIR_SLAVE;
		#endif
	}

	Log(iNow);
}

#endif // PILOT_ONEWHEEL

#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/comms.h"
#include "../Inc/bldc.h"
#include "stdio.h"
#include "string.h"

#ifdef REMOTE_ROS2

//#define REMOTE_ROS2_PID_TUNING

#pragma pack(1)

// Only master communicates with steerin device
#ifdef MASTER_OR_SINGLE

extern uint8_t usart0_rx_buf[1];
extern uint8_t usart1_rx_buf[1];
extern uint8_t usart2_rx_buf[1];


static int16_t iReceivePos = -1;	

extern uint8_t bRemoteTimeout; 	// any Remote can set this to 1 to disable motor (with soft brake)
extern int32_t steer;
extern int32_t speed;

extern int32_t iOdom;
extern float batteryVoltage; 							// global variable for battery voltage
extern float currentDC; 									// global variable for current dc
extern float realSpeed; 									// global variable for real Speed
extern DataSlave oDataSlave;
extern uint8_t  wStateSlave;
extern uint8_t  wState;

// From driver.c:
typedef struct PIDController PIDController;
extern PIDController pid;
void PID_Init(PIDController *pid, float kp, float ki, float kd,
              int16_t min_pwm, int16_t max_pwm, float max_i);

// main.c
extern uint8_t iDrivingMode;	//  0=pwm, 1=speed in revs/s*1024, 2=torque in NewtonMeter*1024, 3=iOdometer
extern int16_t pwmSlave;

// From bldc.c:
extern int32_t revs32;
extern uint8_t nFILTER_SHIFT;
extern int32_t bldc_inputFilterPwm;
extern int32_t bldc_outputFilterPwm;
extern uint16_t buzzerTimer;
extern uint32_t speedCounterSlowLog;


// from https://github.com/alex-makarov/hoverboard-firmware-hack-FOC/blob/master/Src/bldc.c
// Changed to input int32_t since iOdom is int32_t.
int16_t modulo32(int32_t m, int16_t rest_classes)
{
  return (((m % rest_classes) + rest_classes) %rest_classes);
}

#define START_FRAME 0xABCD
#define START_FRAME_MSB 0xAB
#define START_FRAME_LSB 0xCD

typedef struct {
   uint16_t start; // START_FRAME 0xABCD
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;

#define START_FRAME_PID 0xEFCD
#define START_FRAME_PID_MSB 0xEF
#define START_FRAME_PID_LSB START_FRAME_LSB

typedef struct {
   uint16_t start; // START_FRAME_PID 0xEFCD
   int16_t  kp;    // kp*1000  (fixed-point)
   int16_t  ki;    // ki*1000  (fixed-point)
   int16_t  kd;    // kd*10000 (fixed-point)
   int16_t  fs;    // FILTER_SHIFT [0..13]
   int16_t  iDrivingMode; //0=pwm, 1=speed in revs/s*1024, 2=torque in NewtonMeter*1024, 3=iOdometer
   uint16_t checksum;
} SerialPidConfig;

static uint8_t aReceiveBuffer[sizeof(SerialPidConfig)]; // SerialPidConfig is larger than SerialCommand

#define ENCODER_MAX 9000 // Max value of wheelR_cnt/wheelL_cnt before it wraps to 0

typedef struct {
   uint16_t start;         // START_FRAME 0xABCD
   int16_t  cmd1;          // Not used
   int16_t  cmd2;          // Not used
   int16_t  speedR_meas;   // Unit: RPM
   int16_t  speedL_meas;   // Unit: RPM
   int16_t  wheelR_cnt;    // Range: 0-ENCODER_MAX
   int16_t  wheelL_cnt;    // Range: 0-ENCODER_MAX
   int16_t  left_dc_curr;  // Unit: 0.01 A
   int16_t  right_dc_curr; // Unit: 0.01 A
   int16_t  batVoltage;    // Unit: 0.01 V
   int16_t  boardTemp;     // Unit: 0.1 degrees
   uint16_t cmdLed;        // Not used
   uint16_t checksum;      // checksum of all other attributes
} SerialFeedback;


uint32_t iTimeLastRx = 0;
uint32_t iTimeNextTx = 0;

// Send frame to steer device
// Called from main() every 2*DELAY_IN_MAIN_LOOP = 10ms
void RemoteUpdate(void)
{
	if (millis() - iTimeLastRx > LOST_CONNECTION_STOP_MILLIS)
	{
		bRemoteTimeout = 1;
		//speed = steer = 0;
	}

	if (millis() < iTimeNextTx)	
		return;
	iTimeNextTx = millis() + SEND_INTERVAL_MS -1; //-1 to avoid drift (only called every 10ms so precision is ~10ms)
	
	//Send feedback to ROS2
	SerialFeedback feedback;
	feedback.start = START_FRAME;
	feedback.cmd1 = bldc_inputFilterPwm;  // Not used by ROS2 hoverboard-driver
	feedback.cmd2 = bldc_outputFilterPwm; // Not used by ROS2 hoverboard-driver
	feedback.cmdLed = speed;              // Not used by ROS2 hoverboard-driver
#ifndef REMOTE_ROS2_PID_TUNING
	feedback.batVoltage = (int16_t) (batteryVoltage * 100);
	feedback.boardTemp = 250; // Dummy value (250 => 25 degrees)
	feedback.speedL_meas = (int16_t) (realSpeed * 100); // TODO: Need to scale? 100 is just a guess...
	feedback.wheelL_cnt = modulo32(iOdom, ENCODER_MAX);
	feedback.left_dc_curr = (int16_t) (currentDC * 100);
 #ifdef MASTER
	feedback.speedR_meas = (int16_t) (oDataSlave.realSpeed * 100); // TODO: Need to scale? 100 is just a guess... 
	feedback.wheelR_cnt = modulo32(oDataSlave.iOdom, ENCODER_MAX);
	feedback.right_dc_curr = (int16_t) (oDataSlave.currentDC * 100);
 #else // SINGLE (SLAVE not possible due to ifdef MASTER_OR_SINGLE at top of this file)
	feedback.speedR_meas = (int16_t) (realSpeed * 100); // Fake both wheels with same speed for single. TODO: Need to scale?
	feedback.wheelR_cnt = modulo32(iOdom, ENCODER_MAX); // Fake both wheels with same speed for single.
	feedback.right_dc_curr = 0;
 #endif
#else // ifdef REMOTE_ROS2_PID_TUNING
	feedback.batVoltage = speedCounterSlowLog;
	feedback.boardTemp = millis();
	feedback.speedL_meas = revs32 >> 2;
	feedback.wheelL_cnt = iOdom;
	feedback.left_dc_curr = (int16_t) (currentDC * 1000);
	feedback.speedR_meas = revs32;
	feedback.wheelR_cnt = buzzerTimer;
	feedback.right_dc_curr = pwmSlave;
#endif // REMOTE_ROS2_PID_TUNING

	feedback.checksum = (uint16_t)(
		feedback.start ^
		feedback.cmd1 ^
		feedback.cmd2 ^
		feedback.speedR_meas ^
		feedback.speedL_meas ^
		feedback.wheelR_cnt ^
		feedback.wheelL_cnt ^
		feedback.left_dc_curr ^
		feedback.right_dc_curr ^
		feedback.batVoltage ^
		feedback.boardTemp ^
		feedback.cmdLed);
	SendBuffer(USART_REMOTE, (uint8_t*) &feedback, sizeof(feedback));
}


uint16_t messageSize = 0;

// Update USART steer input
void RemoteCallback(void)
{
	uint8_t cRead = USART_REMOTE_BUFFER[0];

	if (iReceivePos<0) {
		// Protocol is in little-endian byte order, i.e. LSB is sent before MSB on wire.
		// (little-endian is used by eg. STM32F103, Raspberry Pi, Intel (x86/x86-64), ESP32, etc.)
		if (cRead == START_FRAME_LSB) { // First Start character is captured
			iReceivePos=0;
		}
	}
	else if (iReceivePos==1) {
		if (cRead == START_FRAME_MSB) {
			messageSize = sizeof(SerialCommand);
		}
#ifdef REMOTE_ROS2_PID_TUNING
		else if (cRead == START_FRAME_PID_MSB) {
			messageSize = sizeof(SerialPidConfig);
		}
#endif
		else { // Second Start character NOT captured, start over
			iReceivePos=-1;
		}
	}

	// Data reading has begun, save in aReceiveBuffer until whole message is received, then compare checksum.
	if (iReceivePos >= 0) {
		aReceiveBuffer[iReceivePos++] = cRead;
		if (iReceivePos == messageSize)
		{
			if (aReceiveBuffer[1] == START_FRAME_MSB)
			{
				SerialCommand* pData = (SerialCommand*) aReceiveBuffer;
				uint16_t checksum = (uint16_t)(pData->start ^ pData->steer ^ pData->speed);
				if (pData->checksum == checksum)
				{
					iTimeLastRx = millis();
					bRemoteTimeout = 0;

					speed = pData->speed;
					steer = pData->steer;

					ResetTimeout();	// Reset the pwm timout to avoid stopping motors
				}
			}
#ifdef REMOTE_ROS2_PID_TUNING
			else if (aReceiveBuffer[1] == START_FRAME_PID_MSB)
			{
				SerialPidConfig* pData = (SerialPidConfig*) aReceiveBuffer;
				uint16_t checksum = (uint16_t)(pData->start ^ pData->kp ^ pData->ki ^ pData->kd ^ pData->fs ^ pData->iDrivingMode);
				if (pData->checksum == checksum)
				{
					float kp = pData->kp/1000.0f;
					float ki = pData->ki/1000.0f;
					float kd = pData->kd/10000.0f;
					PID_Init(&pid, kp, ki, kd, -BLDC_TIMER_MID_VALUE, BLDC_TIMER_MID_VALUE, BLDC_TIMER_MID_VALUE);

					uint8_t iNew = (uint8_t) pData->iDrivingMode;
					iNew = CLAMP(iNew , 0, 4); // Sanity check: Range [0..4]
					iDrivingMode = iNew;

					iNew = pData->fs;
					iNew = CLAMP(iNew , 0, 13); // Sanity check: Range [0..13]
					nFILTER_SHIFT = iNew;
				}
			}
#endif // REMOTE_ROS2_PID_TUNING
			iReceivePos = -1;
		}
	}	
}


#endif // MASTER_OR_SINGLE


#ifndef REMOTE_ROS2_PID_TUNING
  #if DRIVING_MODE != 1
    #error "RemoteROS2 assumes DRIVING_MODE == 1"
  #endif
#endif
// Outputs pPwmMaster and pPwmSlave in revs/s*1024 (fixed-point with 10 bit fraction)
void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave)
{
#ifndef REMOTE_ROS2_PID_TUNING
	*pPwmMaster = ( (speed + (steer/2)) *1024) / 60;	// speed and steer in some kind of RPM
	*pPwmSlave  = ( (speed - (steer/2)) *1024) / 60;	// speed and steer in some kind of RPM
#else // ifdef REMOTE_ROS2_PID_TUNING
	*pPwmMaster = speed;
	*pPwmSlave  = speed;
#endif
}


#endif // REMOTE_ROS2
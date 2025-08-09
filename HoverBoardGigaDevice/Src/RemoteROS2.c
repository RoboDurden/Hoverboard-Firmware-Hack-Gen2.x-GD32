#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/comms.h"
#include "../Inc/bldc.h"
#include "stdio.h"
#include "string.h"

#ifdef REMOTE_ROS2

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
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;

static uint8_t aReceiveBuffer[sizeof(SerialCommand)];

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
	feedback.cmd1 = 0; // Not used
	feedback.cmd2 = 0; // Not used
	feedback.batVoltage = (int16_t) (batteryVoltage * 100);
	feedback.boardTemp = 250; // Dummy value (250 => 25 degrees)
	feedback.cmdLed = 0; // Not used
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
		if (cRead != START_FRAME_MSB) { // Second Start character NOT captured, start over
			iReceivePos=-1;
		}
	}

	// Data reading has begun, save in aReceiveBuffer until whole message is received, then compare checksum.
	if (iReceivePos >= 0) {
		aReceiveBuffer[iReceivePos++] = cRead;
		if (iReceivePos == sizeof(SerialCommand))
		{
			SerialCommand* pData = (SerialCommand*) aReceiveBuffer;
			uint16_t checksum = (uint16_t)(pData->start ^ pData->steer ^ pData->speed);
			if (pData->checksum == checksum)
			{
				iTimeLastRx = millis();
				bRemoteTimeout = 0;
				speed = pData->speed * 3; // TODO: What is the range/unit received? Need any scaling?
				steer = pData->steer * 3; // TODO: What is the range/unit received? Need any scaling?
				
				speed = CLAMP(speed , -1000, 1000); // Sanity check: Range for speed is +-1000
				steer = CLAMP(steer , -1000, 1000); // Sanity check: Range for steer is +-1000
				
				//if (speed > 300) speed = 300;	else if (speed < -300) speed = -300;		// for testing this function

				ResetTimeout();	// Reset the pwm timout to avoid stopping motors
			}
			iReceivePos = -1;
		}
	}	
}

#endif // MASTER_OR_SINGLE

#endif // REMOTE_ROS2
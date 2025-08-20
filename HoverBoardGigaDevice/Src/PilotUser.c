// PilotUser.c extend the firmware with custom code :-)

#include "../Inc/defines.h"
#ifdef PILOT_USER

#include "../Inc/it.h"		// for Delay(milliseconds)
#include <stdio.h>
#include <string.h>

// -- runtime variables you might want to use in your new code -------------------

// write only:
extern uint8_t bPilotTimeout;		// set to 1 to disable bldc (with soft brake)

// read/write
extern uint8_t  wState;				// 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
extern uint8_t iDrivingMode;	//  0=pwm, 1=speed in revs/s*1024, 2=torque in NewtonMeter*1024, 3=iOdometer

// read only
extern int32_t steer;		// -1000 to +1000
extern int32_t speed;		// -1000 to +1000
extern int32_t iOdom;						// global variable for position
extern float batteryVoltage; 		// global variable for battery voltage
extern float currentDC; 				// global variable for current dc
extern float realSpeed; 				// global variable for real Speed
extern int32_t revs32;					// revs/s *1024
extern int32_t torque32;				// torque in Nm*1024 
extern DataSlave oDataSlave;	//	currentDC, realSpeed and iOdom and wState of (possible) slave
extern uint16_t buzzerTimer;	// fastest timer available, counts upwards with PWM_FREQ, limited to 12 kHz for BLDC_SINE

// write only:
extern uint8_t  wStateSlave;	// wSate to be sent to (possible) slave


void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave)
{
	*pPwmMaster = CLAMP(speed + steer,-1000,+1000);	// or something like that
	*pPwmMaster = CLAMP(speed - steer,-1000,1000);	// or something like that
}

#endif // PILOT_USER
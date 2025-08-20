// PilotUser.c extend the firmware with custom code :-)

#include "../Inc/defines.h"
#include "../Inc/it.h"		// for Delay(milliseconds)
#include <stdio.h>
#include <string.h>

// -- runtime variables you might want to use in your new code -------------------
extern uint8_t bPilotTimeout;		// set to 1 to disable bldc (with soft brake)
extern int32_t steer;		// -1000 to +1000
extern int32_t speed;		// -1000 to +1000
extern int32_t iOdom;						// global variable for position
extern float batteryVoltage; 		// global variable for battery voltage
extern float currentDC; 				// global variable for current dc
extern float realSpeed; 				// global variable for real Speed
extern uint8_t  wState;				// 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
extern DataSlave oDataSlave;	//	currentDC, realSpeed and iOdom and wState of (possible) slave


void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave)
{
	*pPwmMaster = CLAMP(speed + steer,-1000,+1000);	// or something like that
	*pPwmMaster = CLAMP(speed - steer,-1000,1000);	// or something like that
}

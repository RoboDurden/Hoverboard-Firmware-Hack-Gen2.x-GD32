#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/bldc.h"
#include <string.h>


#ifdef REMOTE_DUMMY

extern uint32_t iBug;
extern uint8_t iBug8;

extern int32_t steer;
extern int32_t speed;
extern uint32_t msTicks;
extern uint8_t iDrivingMode;	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)
//#define SLOW_SINE
uint8_t bRemoteInit = 1;

int8_t iRemotePeriod = REMOTE_PERIOD;  // 3 = 3 seconds period of the zigzag curve
float fRemoteScaleMax = 1;	// to flatten the zigzag curve to send constant iRemoteMax for some time
int32_t iRemoteMax = 0;

uint8_t iOldDrivingMode = 0;
int8_t iOldRemotePeriod = REMOTE_PERIOD;



uint32_t msTicksInit = 0;
void RemoteUpdate(void)
{
	
	#ifdef MASTER_OR_SINGLE

	
	  if ((iOldDrivingMode != iDrivingMode)  )//|| (memcmp(aoPIDInitOld, aoPIDInit, sizeof(aoPIDInit)) != 0)	)
		{
			iOldDrivingMode	= iDrivingMode;
			DriverInit(iDrivingMode);
		}	
		if (iOldRemotePeriod != iRemotePeriod)		// values might have changed via StmStudio or PlatformIO/VisualCode
		{
			iOldRemotePeriod = iRemotePeriod;
			msTicksInit = msTicks;	// to start zigzag from 0 no matter how long the startup in main.c takes
		}
		
		if (bRemoteInit)
		{
			bRemoteInit = 0;
			msTicksInit = msTicks;	// to start zigzag from 0 no matter how long the startup in main.c takes
			switch(iDrivingMode)	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)
			{
				case 0: iRemoteMax = 500; break;	// pwm value
				case 1: iRemoteMax = 0.5 *1024; break;	// 1.5*1024 = max speed 1.5 revs/s
				case 2: iRemoteMax = 5.0 *1024; break;	// 1.5*1024 = max speed 1.5 Nm (Newton meter)
				case 3: iRemoteMax = 90; break;	// 90 = 360�
			}
			fRemoteScaleMax = 1.6*(CLAMP(iRemotePeriod,3,9)/9.0);  // to flatten the zigzag curve to send constant iRemoteMax for some time
		}
	
		steer = 0;
		speed = CLAMP((fRemoteScaleMax*iRemoteMax/100) * (ABS( (int)(((msTicks-msTicksInit)/iRemotePeriod+250) % 1000) - 500) - 250),-iRemoteMax,iRemoteMax);   // repeats from +300 to -300 to +300 :-)
		speed = CLAMP(speed , -iRemoteMax, iRemoteMax);
		//speed = 300;
		//speed = 0;	// uncomment this to turn the motor manually when TEST_HALL2LED
		//speed = 1.5*1024;
	#else
		SetEnable(SET);
		SetBldcInput(-speed);
	#endif
	
	ResetTimeout();	// Reset the pwm timout to avoid stopping motors	
}

void RemoteCallback(void){};

int16_t	RemoteCalculate(int16_t pwmInput)
{
	return pwmInput;
}
	
	
#endif
#include "../Inc/defines.h"
#include "../Inc/it.h"


#ifdef REMOTE_DUMMY


extern int32_t steer;
extern int32_t speed;
extern uint32_t msTicks;
extern uint8_t iDrivingMode;	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)

//#define SLOW_SINE

void RemoteUpdate(void)
{
	int32_t iMax = 250;	// pwm 300 as default
	switch(iDrivingMode)	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)
	{
		case 1: iMax = 3.0 *1024; break;	// 1.5*1024 = max speed 1.5 revs/s
	}
	
	#ifdef MASTER_OR_SINGLE
		steer = 0;
		#ifdef SLOW_SINE
			speed = ((iMax*30)/1000) * (ABS((	((int32_t)msTicks/90+100) % 400) - 200) - 100);		// for longer max speed
		#else
			speed = ((iMax*16)/1000) * (ABS((	((int32_t)msTicks/30+100) % 400) - 200) - 100);
		#endif
	
		speed = CLAMP(speed , -iMax, iMax);
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
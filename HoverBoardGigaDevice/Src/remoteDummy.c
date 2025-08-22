#include "../Inc/defines.h"
#include "../Inc/it.h"


#ifdef REMOTE_DUMMY


extern int32_t steer;
extern int32_t speed;
extern uint32_t msTicks;
extern uint8_t iDrivingMode;	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)

//#define SLOW_SINE
uint32_t iTimeRTT = 0;
void RemoteUpdate(void)
{
	int32_t iMax = 0;
	switch(iDrivingMode)	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)
	{
		case 0: iMax = 500; break;	// pwm value
		case 1: iMax = 0.5 *1024; break;	// 1.5*1024 = max speed 1.5 revs/s
		case 2: iMax = 5.0 *1024; break;	// 1.5*1024 = max speed 1.5 Nm (Newton meter)
		case 3: iMax = 900; break;	// 90 = 360�
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
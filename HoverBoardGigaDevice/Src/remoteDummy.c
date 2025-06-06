#include "../Inc/defines.h"
#include "../Inc/it.h"


#ifdef REMOTE_DUMMY


extern int32_t steer;
extern int32_t speed;
extern uint32_t msTicks;

#define MAX_SPEED 1000		// can range from 0 to 1000

void RemoteUpdate(void)
{
	#ifdef MASTER_OR_SINGLE
		speed = ((MAX_SPEED*12)/1000) * (ABS((	((int32_t)msTicks/30+100) % 400) - 200) - 100);
		//speed = ((MAX_SPEED*30)/1000) * (ABS((	((int32_t)msTicks/90+100) % 400) - 200) - 100);		// for longer max speed
	
		speed = CLAMP(speed , -MAX_SPEED, MAX_SPEED);
		//speed = 100;
		//speed = 0;	// uncomment this to turn the motor manually when TEST_HALL2LED
	#else
		SetEnable(SET);
		SetPWM(-speed);
	#endif
	
	ResetTimeout();	// Reset the pwm timout to avoid stopping motors	
}

void RemoteCallback(void){};

#endif
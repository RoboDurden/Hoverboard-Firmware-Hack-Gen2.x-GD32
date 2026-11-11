// remoteNone.c : no remote control at all, the motors are controlled by the pilot only (PILOT_ONEWHEEL, PILOT_USER, ..)

#include "../Inc/defines.h"
#include "../Inc/it.h"

#ifdef REMOTE_NONE

void RemoteUpdate(void)
{
	ResetTimeout();	// Reset the pwm timout to avoid stopping motors
}

void RemoteCallback(void){};

#endif

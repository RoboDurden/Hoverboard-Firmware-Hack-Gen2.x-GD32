#include "../Inc/defines.h"

#ifdef REMOTE_ADC


extern int32_t steer;
extern int32_t speed;
extern uint32_t msTicks;
extern adc_buf_t adc_buffer;

void RemoteUpdate(void)
{
	#ifdef MASTER_OR_SINGLE
		//speed = 3 * (ABS((	((int32_t)msTicks/30+100) % 400) - 200) - 100);
		//speed = CLAMP(speed , -1000, 1000);
		//speed = 100;
		//speed = 0;	// uncomment this to turn the motor manually when TEST_HALL2LED
	
		speed = adc_buffer.speed/2 - 1024; 
		speed = CLAMP(speed , -500, 500);

		steer = adc_buffer.steer/2 - 1024; 
		steer = CLAMP(-steer , -1000, 1000);
	
		//	DEBUG_LedSet(bSet,iColor) macro. iCol: 0=green, 1=organge, 2=red
		int iLevel = adc_buffer.speed >> 10;	// 12 bit adc value
		for (int i=0; i<3; i++){	DEBUG_LedSet(i < iLevel,i);	}
	#endif
	
	
}

void RemoteCallback(void){};

#endif
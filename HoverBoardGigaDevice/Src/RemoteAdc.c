#include "../Inc/defines.h"
#include "../Inc/it.h"

#ifdef REMOTE_ADC

extern uint8_t bRemoteTimeout; 	// any Remote can set this to 1 to disable motor (with soft brake)
extern int32_t steer;
extern int32_t speed;
extern uint32_t msTicks;
extern adc_buf_t adc_buffer;
#ifdef BUZZER
	extern uint8_t buzzerFreq;    						// global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
	extern uint8_t buzzerPattern; 						// global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
#endif

typedef struct
{
  uint16_t iSpeedNeutral;
  uint16_t iSteerNeutral;
  uint16_t iSpeedMax;
  uint16_t iSpeedMin;
  uint16_t iSteerMax;
  uint16_t iSteerMin;
} AdcCalobration;

extern ConfigData oConfig;

#define LOWPASS 0.01

int16_t iConfigMode = 0;	// 0 = default behavior
float fSpeed=2048, fSteer=2048;	// mid point of 12 bit adc

uint32_t iTimeNextCase = 0;
uint32_t iTimeWait = 0;

#define WAIT_NEXT_STEP 5000
#define WAIT_BETWEEN 2000

void 	_NextCase()
{
		fSpeed = fSteer = 2048;		// reset
		iTimeWait = millis() + WAIT_BETWEEN;
		iConfigMode++;
}

void RemoteUpdate(void)
{
	ResetTimeout();	// Reset the pwm timout to avoid stopping motors	
	
	#ifdef MASTER_OR_SINGLE
	
		if (!iConfigMode)
		{
			int16_t iSpeed = adc_buffer.speed;
			iSpeed = CLAMP(iSpeed,oConfig.iSpeedMin,oConfig.iSpeedMax);
			
			if (iSpeed >= oConfig.iSpeedNeutral)
			{
				iSpeed -= oConfig.iSpeedNeutral;	// should be positive
				int16_t iRange = oConfig.iSpeedMax-oConfig.iSpeedNeutral;
				speed = (1000.0*iSpeed)/iRange;
			}
			else
			{
				iSpeed -= oConfig.iSpeedNeutral;		// should be negative
				int16_t iRange = oConfig.iSpeedNeutral-oConfig.iSpeedMin;
				speed = (1000.0*iSpeed)/iRange;
			}

			int16_t iSteer = adc_buffer.steer;
			if (iSteer >= oConfig.iSteerNeutral)
			{
				iSteer -= oConfig.iSteerNeutral;		// should be positive
				int16_t iRange = oConfig.iSteerMax-oConfig.iSteerNeutral;
				steer = (1000.0*iSteer)/iRange;
			}
			else
			{
				iSteer -= oConfig.iSteerNeutral;		// should be negative
				int16_t iRange = oConfig.iSteerNeutral-oConfig.iSteerMin;
				steer = (1000.0*iSteer)/iRange;
			}
			
			//speed = adc_buffer.speed/2 - 1024; 
			//steer = adc_buffer.steer/2 - 1024; 
			
			speed = CLAMP(speed , -1000, 1000);
			steer = CLAMP(-steer , -1000, 1000);
		
			//	DEBUG_LedSet(bSet,iColor) macro. iCol: 0=green, 1=organge, 2=red
			int iLevel = adc_buffer.speed >> 10;	// 12 bit adc value
			#ifdef DEBUG_LED
//				for (int i=0; i<3; i++){	DEBUG_LedSet(i < iLevel,i);	}
			#endif
			return;
		}

		if (iTimeWait)
		{
			BUZZER_MelodyDown();
			if (iTimeWait < millis())
				return;
			
			iTimeWait = 0;
			iTimeNextCase = millis() + WAIT_NEXT_STEP;
				
		}
		BuzzerSet(8,iConfigMode)	// (iFrequency, iPattern)
		switch (iConfigMode)
		{
		case 1:	// read neutral position for adc_buffer.speed and adc_buffer.steer
			fSpeed = (1-LOWPASS) * fSpeed + LOWPASS * adc_buffer.speed;
			fSteer = (1-LOWPASS) * fSteer + LOWPASS * adc_buffer.steer;
			if (!iTimeNextCase)	// first call after button release
				iTimeNextCase = millis() + WAIT_NEXT_STEP;
			else if (millis() > iTimeNextCase)
			{
				oConfig.iSpeedNeutral = fSpeed;
				oConfig.iSteerNeutral = fSteer;
				_NextCase();
			}
			break;
		case 2:	// read max adc_buffer.speed 
			fSpeed = (1-LOWPASS) * fSpeed + LOWPASS * adc_buffer.speed;
			if (millis() > iTimeNextCase)
			{
				oConfig.iSpeedMax = fSpeed;
				_NextCase();
			}
			break;
		case 3:	// read min adc_buffer.speed 
			fSpeed = (1-LOWPASS) * fSpeed + LOWPASS * adc_buffer.speed;
			if (millis() > iTimeNextCase)
			{
				oConfig.iSpeedMin = fSpeed;
				_NextCase();
			}
			break;
		case 4:	// read max adc_buffer.steer
			fSteer= (1-LOWPASS) * fSteer + LOWPASS * adc_buffer.steer;
			if (millis() > iTimeNextCase)
			{
				oConfig.iSteerMax = fSteer;
				_NextCase();
			}
			break;
		case 5:	// read min adc_buffer.steer
			fSteer= (1-LOWPASS) * fSteer + LOWPASS * adc_buffer.steer;
			if (millis() > iTimeNextCase)
			{
				oConfig.iSteerMin = fSteer;
				int16_t iRangeMax = oConfig.iSpeedMax - oConfig.iSpeedNeutral;
				int16_t iRangeMin = oConfig.iSpeedNeutral - oConfig.iSpeedMin;
				if (iRangeMin != 0)
					if (iRangeMax/iRangeMin>10)	oConfig.iSpeedMin = oConfig.iSpeedNeutral;	// only forwards
				
				if (iRangeMax != 0)
					if (iRangeMin/iRangeMax>10)	oConfig.iSpeedMax = oConfig.iSpeedNeutral;	// only backwards
					
				ConfigWrite();
				iConfigMode = 0;
				BUZZER_MelodyDown();
			}
			break;
		}
	#endif
		
}

void RemoteCallback(void){};

#endif
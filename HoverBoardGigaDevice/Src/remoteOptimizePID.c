#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/bldc.h"
#include <string.h>


#ifdef REMOTE_OPTIMIZEPID

int32_t iRemoteMax = 90;
extern uint32_t iBug;
extern uint8_t iBug8;

#ifdef RTT_REMOTE

#define BUFFER_SIZE 64 // Define a reasonable buffer size for incoming messages
static char message_buffer[BUFFER_SIZE];	// Static buffer to store the incoming RTT message
static unsigned int buffer_index = 0;	// Index to keep track of the current position in the buffer

	void parse_message(const char *message) 
	{
		iBug++;
		iBug8 = message[0];
		float f = 0.0f;
		if (sscanf(message, "max=%f", &f) == 1) 	// Use sscanf to parse the string. It returns the number of items successfully read.
		{
			iRemoteMax = f;
		}
	}

	void check_rtt_for_message(void) 
	{
		char c = SEGGER_RTT_GetKey(); // Read a single character from the RTT buffer

		if (c) 	// Check if a character was actually read
		{ 
			iBug8 = c;
			if (c == '\n' || c == '\r') 	// Check for end of line characters
			{
				if (buffer_index > 0) 
				{
					message_buffer[buffer_index] = '\0';	// Null-terminate the string
					parse_message(message_buffer);	// Process the received message
					buffer_index = 0;	// Reset the buffer index for the next message
				}
			}
			else if (buffer_index < (BUFFER_SIZE - 1)) 
			{
				message_buffer[buffer_index++] = c;	// Add the character to the buffer and increment the index
			}
			// If the buffer is full but no newline is received, we discard the message
			// to prevent overflow. A reset is triggered on the next newline.
		}
	}
#endif

#define REMOTE_PERIOD 3


extern int32_t steer;
extern int32_t speed;
extern uint32_t msTicks;
extern uint8_t iDrivingMode;	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)
extern float batteryVoltage;
uint32_t iTimeRTT = 0;
uint8_t bRemoteInit = 1;

int8_t iRemotePeriod = REMOTE_PERIOD;  // 3 = 3 seconds period of the zigzag curve
float fRemoteScaleMax = 1;	// to flatten the zigzag curve to send constant iRemoteMax for some time

uint8_t iOldDrivingMode = 0;
int8_t iOldRemotePeriod = REMOTE_PERIOD;

extern PIDInit aoPIDInit[3];

#define COUNT_SPEEDS 3
uint8_t iOptimizeSpeeds = COUNT_SPEEDS;
extern uint32_t iOptimizeErrors;
extern uint32_t iOptimizeTotal;
uint8_t aiFailure[COUNT_SPEEDS];
uint8_t iFailureMax=0, iFailureMaxOld=0;
extern uint16_t iOptimizeThreshold;	// 100% error would 1024, 5% error = 51, 2% == 33
int8_t iFailureMaxPos = -1;	// if >= 0 then only optimize for this speed
uint8_t iMinDistance = 0, iMinDistanceSolo = 30;
uint8_t iFailureMaxPosCount = 0;


int32_t speedOld = 0;
uint32_t iError32Sum = 0, iError32SumOld = 0;
uint8_t iRemoteRetries = 0;

int8_t iOptimizePos = -1;	//  <0 to start optimizing on next test cycle
int8_t iOptimizeVector = 0, iOptimizeAdd = 1;	//  
float afOptimizeBase[4];
#define VECTOR_BUFFER 30		// keep the latest iFailureMax in an array to move on when 4 times the same pid was tested
int8_t aiOptimizeVector[VECTOR_BUFFER];

uint16_t iOptimizeCounter = 0;


float fNew=0;
void _OptimizeSet(PIDInit* pPID,int8_t iDirection)	// iDirection=1: same direction, -1: opposite directoin, 0 same iOptimizeAdd
{
	if (iDirection != 0)
	{
		if (iDirection<0)	iOptimizeAdd *= -1;
		iOptimizeVector += iOptimizeAdd;
	}
	fNew = afOptimizeBase[iOptimizePos];
	fNew *= 1.0+iOptimizeVector/10.0;
	if (fNew<0)
	{
		fNew = 0;
		iOptimizeAdd *= -1;
	}
	switch (iOptimizePos)
	{
		case 0: pPID->kp = fNew; break;
		case 1: pPID->ki = fNew; break;
		case 2: pPID->kd = fNew; break;
		case 3: 
				if (fNew > BLDC_TIMER_MID_VALUE)	fNew = BLDC_TIMER_MID_VALUE;
				pPID->max_i = fNew; 
				break;
	}
}

void _OptimizeInit(PIDInit* pPID)
{
	afOptimizeBase[0] = pPID->kp;
	afOptimizeBase[1] = pPID->ki;
	afOptimizeBase[2] = pPID->kd;
	afOptimizeBase[3] = pPID->max_i;
	
	iOptimizeVector = 0;
	iOptimizeAdd = 1;
	memset(aiOptimizeVector,127,sizeof(aiOptimizeVector));
	if (iFailureMaxPos<0)	// not optimizing on only one speed
		memset(aiFailure,0,sizeof(aiFailure));
}

uint8_t iCountMax=0;
char sMessage[512];

/* to calculate the .xf nunber of x decimals based on 1/(afOptimizeBase[i]/10) 
for sprintf(sMessage, "%stesting pid: %.3f, %.4f, %.5f\t, pos %i %i\n",sMessage,pPID->kp,pPID->ki,pPID->kd,iFailureMaxPos,iFailureMaxPosCount);

uint16_t _NumPlaces (uint32_t n) 
{
    uint16_t r = 1;
    while (n > 9) 
		{
        n /= 10;
        r++;
    }
    return r;
}
*/

uint32_t msTicksInit = 0;
void RemoteUpdate(void)
{
	#ifdef RTT_REMOTE
		check_rtt_for_message();	// NOT YET WORKING :-( thanks to Gemini 2.5Pro
	#endif
	
	iDrivingMode = CLAMP(iDrivingMode,1,3);
	if ((iOldDrivingMode != iDrivingMode)  )
	{
		iOldDrivingMode	= iDrivingMode;
		DriverInit(iDrivingMode);
		iBug++;
	}	
	if (iOldRemotePeriod != iRemotePeriod)		// values might have changed via StmStudio or PlatformIO/VisualCode
	{
		iOldRemotePeriod = iRemotePeriod;
		msTicksInit = msTicks;	// to start zigzag from 0 no matter how long the startup in main.c takes
	}
	
	PIDInit* pPID = &aoPIDInit[iDrivingMode-1];
	if (bRemoteInit)
	{
		bRemoteInit = 0;
		msTicksInit = msTicks;	// to start zigzag from 0 no matter how long the startup in main.c takes
		fRemoteScaleMax = 5;
		iOptimizePos = -1;
	}

	steer = 0;
	speed = CLAMP((fRemoteScaleMax*iRemoteMax/100) * (ABS( (int)(((msTicks-msTicksInit)/iRemotePeriod+250) % 1000) - 500) - 250),-iRemoteMax,iRemoteMax);   // repeats from +300 to -300 to +300 :-)
	speed = CLAMP(speed , -iRemoteMax, iRemoteMax);

	if (	(speed<=0) && (speedOld>0)	)		// beginning of a zigzag
	{
		if (iOptimizePos <0)
		{
			iRemoteRetries = iOptimizeErrors = iOptimizeTotal = 0;
			iOptimizePos = 0;		// start to optimize kp
			_OptimizeInit(pPID);
		}
		else
		{
			aiFailure[iRemoteRetries++] = iOptimizeErrors*100/iOptimizeTotal;
			iOptimizeErrors = iOptimizeTotal = 0;
			if (	(iRemoteRetries==iOptimizeSpeeds) || (iFailureMaxPos>=0)	)
			{
				iFailureMax = 0;
				uint8_t iFailureMaxPosNew = -1;
				for (uint8_t i=0; i<iOptimizeSpeeds; i++)
				{
					if (iFailureMax < aiFailure[i]) 
					{
						iFailureMax = aiFailure[i];
						iFailureMaxPosNew = i;
					}
				}
				iMinDistance = 100;
				for (uint8_t i=0; i<iOptimizeSpeeds; i++)
				{
					if (i != iFailureMaxPosNew) 
					{
						uint8_t iDist = aiFailure[iFailureMaxPosNew] - aiFailure[i];
						if (iMinDistance > iDist)
							iMinDistance = iDist;
					}
				}
				if (iMinDistance > iMinDistanceSolo)	// one speed is more than 50% worse then the others, only optimize this for now
				{
					if (iFailureMaxPos<0)
						iFailureMaxPosCount = 3;	// no optimize only one speed for all thre pid parameters before testing the other speeds again.
					
					iFailureMaxPos = iFailureMaxPosNew;
				}
				
				switch(iDrivingMode)
				{
				case 3:
					if (iFailureMax > 80)	// more than 80% not below threshold
						iOptimizeThreshold += 3;	// add 3° more failure 
					if (iFailureMax < 70)	// more than 80% not below threshold
						iOptimizeThreshold -= 2;	// reduce by 2° failure 
					iOptimizeThreshold = CLAMP(iOptimizeThreshold,1,45);
					break;
				default:	// constant speed or max torque. torque will not yet work as the pid will only limit the torque!
					if (iFailureMax > 80)	// more than 80% not below threshold
						iOptimizeThreshold += 5;	// 100% error would 1024, 5% error = 51, 2% == 33
					if (iFailureMax < 60)	// more than 80% not below threshold
						iOptimizeThreshold -= 3;	// 100% error would 1024, 5% error = 51, 2% == 33
					iOptimizeThreshold = CLAMP(iOptimizeThreshold,33,204);
				}
						
				
				iRemoteRetries = 0;
				sprintf(sMessage, "aiFailure: %i %i %i\t, threshold: %i\n",aiFailure[0],aiFailure[1],aiFailure[2],iOptimizeThreshold);

				if (iFailureMaxOld == 0)
				{
					_OptimizeInit(pPID);
				}
				else					// we have a previous test result
				{
					iOptimizeCounter++;
					//memcpy(aiOptimizeVector+1,aiOptimizeVector,sizeof(aiOptimizeVector)-1);
					for (int8_t i=VECTOR_BUFFER-2; i>=0; i--)	aiOptimizeVector[i+1] = aiOptimizeVector[i];
					aiOptimizeVector[0] = iOptimizeVector;
					if (iFailureMax >= iFailureMaxOld)	// worsepid setting
					{
						_OptimizeSet(pPID,-1);	// back to previous
						
						iCountMax = 1;
						for (uint8_t i=0; i<VECTOR_BUFFER;i++)
						{
							uint8_t iCount = 1;
							for (uint8_t j=i+1; j<VECTOR_BUFFER;j++)
							{
								if (aiOptimizeVector[j] == 127)
									break;
								if (aiOptimizeVector[i] == aiOptimizeVector[j])
									iCount++;
							}
							if (iCount > iCountMax)	iCountMax = iCount;
						}
						sprintf(sMessage, "%sworse: max repitions: %i\n",sMessage,iCountMax);

						if (iCountMax >= 3)	
						{
							//iBug = 42;
							iOptimizePos = (iOptimizePos+1)%3;
							_OptimizeInit(pPID);
							
							if (iFailureMaxPosCount>0)
								iFailureMaxPosCount--;
							else
								iFailureMaxPos = -1;	// go back to test all speeds
							
						}
						else
						{
						}
					}
					else
					{
						_OptimizeSet(pPID,1);		// continue in this direction
					}
					
					sprintf(sMessage, "%stesting pid: %.3f, %.4f, %.5f\t, pos %i %i\n",sMessage,pPID->kp,pPID->ki,pPID->kd,iFailureMaxPos,iFailureMaxPosCount);
					DriverInit(iDrivingMode);
				}
				iRemoteRetries = 0;
				iFailureMaxOld = iFailureMax;
			}
		}
		if (iFailureMaxPos >=0)	iRemoteRetries = iFailureMaxPos;
		switch (iDrivingMode)
		{
			case 1:
				iRemoteMax = 512+ iRemoteRetries * 307 * (int16_t)(10.0*batteryVoltage / 42.0);		// 7s battery can not reach same max speed as 42V 10s battery
				iRemotePeriod = 3;
				iOptimizeSpeeds = 3;
				break;
			case 2:
				iRemoteMax = 1024+ iRemoteRetries * 1024;	// 1024 = max target 1.0 Nm (Newton meter)
				iRemotePeriod = 3;
				iOptimizeSpeeds = 3;
				break;
			case 3:
				iRemoteMax = 45 + iRemoteRetries * 45;		// 90 == 360°
				iRemotePeriod = 4 + iRemoteRetries * 1;	// higher position targets need more seconds
				iOptimizeSpeeds = 3;
				break;
		}
		//sprintf(sMessage, "%smax target: %i\t, bug: %i %i '%s'\n",sMessage,iRemoteMax,iBug,iBug8,message_buffer);
		sprintf(sMessage, "%smax target: %i\n",sMessage,iRemoteMax);
		//iRemoteRetries++;
	}
	#ifdef RTT_REMOTE
		if (strlen(sMessage))
		{
			#ifdef WINDOWS_RN
				add_cr_before_lf_inplace(sMessage,sizeof(sMessage));
			#endif
			
			SEGGER_RTT_WriteString(0, sMessage);
			sMessage[0] = '\0';
		}
	#endif
	
	speedOld = speed;
	ResetTimeout();	// Reset the pwm timout to avoid stopping motors	
}

void RemoteCallback(void){};

int16_t	RemoteCalculate(int16_t pwmInput)
{
	return pwmInput;
}
	
	
#endif	// REMOTE_OPTIMIZEPID
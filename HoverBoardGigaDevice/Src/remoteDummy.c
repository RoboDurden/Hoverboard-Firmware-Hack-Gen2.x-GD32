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
extern PIDInit aoPIDInit[3];
PIDInit aoPIDInitOld[3];

uint8_t bRemoteInit = 1;

int8_t iRemotePeriod = REMOTE_PERIOD;  // 3 = 3 seconds period of the zigzag curve
float fRemoteScaleMax = 1;	// to flatten the zigzag curve to send constant iRemoteMax for some time
int32_t iRemoteMax = 0;

uint8_t iOldDrivingMode = 0;
int8_t iOldRemotePeriod = REMOTE_PERIOD;

#ifdef RTT_REMOTE

	extern uint8_t iDrivingMode;	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)
	extern float batteryVoltage;
	extern int32_t iOdom;
	extern float batteryVoltage; 							// global variable for battery voltage
	extern float currentDC; 									// global variable for current dc
	extern float realSpeed; 									// global variable for real Speed
	extern int32_t revs32;
	extern int32_t torque32;

	uint32_t iTimeNextLog = 0;
	char sMessage[512];
	#define BUFFER_SIZE 64 // Define a reasonable buffer size for incoming messages
	static char message_buffer[BUFFER_SIZE];	// Static buffer to store the incoming RTT message
	static unsigned int buffer_index = 0;	// Index to keep track of the current position in the buffer

volatile int32_t rtt_diag_last_c = -2;
volatile uint32_t rtt_diag_rx_count = 0;
volatile uint32_t rtt_diag_rd = 0;
volatile uint32_t rtt_diag_wr = 0;
volatile uint32_t rtt_diag_size = 0;
volatile uint32_t rtt_diag_flags = 0;
volatile char rtt_diag_chars[16];


	uint8_t _ParseFloatLite(const char* s, float* pf)
	{
		int8_t sign = 1;
		uint32_t whole = 0;
		uint32_t frac = 0;
		uint32_t scale = 1;
		uint8_t has_digit = 0;

		if (*s == '-') {
			sign = -1;
			s++;
		} else if (*s == '+') {
			s++;
		}

		while ((*s >= '0') && (*s <= '9')) {
			has_digit = 1;
			whole = whole * 10u + (uint32_t)(*s - '0');
			s++;
		}

		if ((*s == '.') || (*s == ',')) {
			s++;
			while ((*s >= '0') && (*s <= '9')) {
				has_digit = 1;
				if (scale < 1000000u) {
					frac = frac * 10u + (uint32_t)(*s - '0');
					scale *= 10u;
				}
				s++;
			}
		}

		if (!has_digit || (*s != '\0')) return 0;

		*pf = (float)sign * ((float)whole + ((float)frac / (float)scale));
		return 1;
	}

	uint8_t _TestKey(const char* s, char* sKey, float* pf)
	{
		uint8_t i = 0;
		while (sKey[i]) {
			if (s[i] != sKey[i]) return 0;
			i++;
		}
		if (s[i] != '=') return 0;

		return _ParseFloatLite(&s[i + 1], pf);
	}

	void parse_message(const char *message) 
	{
		float f = 0.0f;

		char asKey[][10] = {"max","period","p","i","d","drive"};
		int8_t iKey = sizeof(asKey)/10-1;
		if (strcmp(message, "help") == 0)
		{
			sprintf(sMessage, "available 'cmd'=42.17 :");
	   		for (; iKey>=0; iKey--) sprintf(sMessage + strlen(sMessage), "\t'%s'", asKey[iKey]);

			iTimeNextLog = msTicks + 2000;
			return;
		}
		sprintf(sMessage, "received: '%s'\n",message);
		for (; iKey>=0; iKey--)
		{
			if (_TestKey(message,asKey[iKey],&f))	break;
		}
		PIDInit* pPID = &aoPIDInit[iDrivingMode-1];
		switch (iKey)
		{
			case 0: iRemoteMax = f; break;
			case 1: iRemotePeriod = ABS(f); break;
			case 2: pPID->kp= ABS(f); break;
			case 3: pPID->ki= ABS(f); break;
			case 4: pPID->kd= ABS(f); break;
			case 5: iDrivingMode = CLAMP(f,0,3); break;
		}
	}

	void check_rtt_for_message(void) 
	{
		int c = SEGGER_RTT_GetKey(); // Read a single character from the RTT buffer

		rtt_diag_last_c = c;
		rtt_diag_rd = _SEGGER_RTT.aDown[0].RdOff;
		rtt_diag_wr = _SEGGER_RTT.aDown[0].WrOff;
		rtt_diag_size = _SEGGER_RTT.aDown[0].SizeOfBuffer;
		rtt_diag_flags = _SEGGER_RTT.aDown[0].Flags;

		if (c >= 0) {
			rtt_diag_chars[rtt_diag_rx_count & 15] = (char)c;
			rtt_diag_rx_count++;
		}

		if (c>=0) 	// Check if a character was actually read
		{ 
			if (c == '\n' || c == '\r') 	// Check for end of line characters
			{
				if (buffer_index > 0) 
				{
					message_buffer[buffer_index] = '\0';	// Null-terminate the string
					parse_message(message_buffer);	// Process the received message
					buffer_index = 0;	// Reset the buffer index for the next message
				}
			}
			else if ((c >= ' ') && (c <= '~') && (buffer_index < (BUFFER_SIZE - 1))) 
			{
				message_buffer[buffer_index++] = c;	// Add the character to the buffer and increment the index
			}
			// If the buffer is full but no newline is received, we discard the message
			// to prevent overflow. A reset is triggered on the next newline.
		}
	}
	
	uint16_t iCounterLog = 0;
	
	
#endif




uint32_t msTicksInit = 0;
void RemoteUpdate(void)
{
	#ifdef RTT_REMOTE
		check_rtt_for_message();	// NOT YET WORKING :-( thanks to Gemini 2.5Pro
	#endif
	
	#ifdef MASTER_OR_SINGLE

	  if ((iOldDrivingMode != iDrivingMode)  || (memcmp(aoPIDInitOld, aoPIDInit, sizeof(aoPIDInit)) != 0)	)
		{
			iOldDrivingMode	= iDrivingMode;
			DriverInit(iDrivingMode);
			memcpy(aoPIDInitOld, aoPIDInit, sizeof(aoPIDInit));
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
				case 0: iRemoteMax = 200; break;	// pwm value
				case 1: iRemoteMax = 1.0 *1024; break;	// 1.5*1024 = max speed 1.5 revs/s
				case 2: iRemoteMax = 5.0 *1024; break;	// 1.5*1024 = max speed 1.5 Nm (Newton meter)
				case 3: iRemoteMax = 90; break;	// 90 = 360�
			}
			fRemoteScaleMax = 1.6*(CLAMP(iRemotePeriod,3,9)/9.0);  // to flatten the zigzag curve to send constant iRemoteMax for some time
			//fRemoteScaleMax = 10;		// to get near rectangle +-iRemoteMax 
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

	#ifdef RTT_REMOTE
		
		if (iTimeNextLog < msTicks)
		{
			iTimeNextLog = msTicks + 200;
			iCounterLog++;

			sprintf(sMessage + strlen(sMessage), F2IF " V\t" F2IF " A\todom: %6ld\ttarget: %5ld\trevs: %5ld\ttorque: %5ld",
					F2I(batteryVoltage),
					F2I(currentDC),
					(long)iOdom, 
					(long)speed, 
					(long)(revs32 >> (REVS32_SHIFT - 10)), 
					(long)torque32  );
			if (iCounterLog%10==0)
			{
				PIDInit* pPID = &aoPIDInit[iDrivingMode-1];
    			sprintf(sMessage + strlen(sMessage), "\tdriveMde: %i , pid: " F2IF " " F3IF " " F4IF, iDrivingMode, F2I(pPID->kp), F3I(pPID->ki), F4I(pPID->kd));
			}
		}
		if (strlen(sMessage))
		{
    		strcat(sMessage, "\n");			
			#ifdef WINDOWS_RN
				add_cr_before_lf_inplace(sMessage,sizeof(sMessage));
			#endif
			
			SEGGER_RTT_WriteString(0, sMessage);
			sMessage[0] = '\0';
		}
	#endif
	
	ResetTimeout();	// Reset the pwm timout to avoid stopping motors	
}

void RemoteCallback(void){};

int16_t	RemoteCalculate(int16_t pwmInput)
{
	return pwmInput;
}
	
	
#endif

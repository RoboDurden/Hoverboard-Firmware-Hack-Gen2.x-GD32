#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/bldc.h"

#ifdef REMOTE_ADC

extern uint8_t bRemoteTimeout; // any Remote can set this to 1 to disable motor (with soft brake)
extern int32_t steer;
extern int32_t speed;
extern uint32_t msTicks;
extern adc_buf_t adc_buffer;
extern uint8_t iDrivingMode;
#ifdef RTT_REMOTE
extern float batteryVoltage;
extern float currentDC;
extern int32_t revs32;
uint32_t iTimeNextLog = 0;
char sMessage[160];
#endif

void RemoteUpdate(void)
{
	ResetTimeout(); // Reset the pwm timout to avoid stopping motors

#ifdef MASTER_OR_SINGLE
	{
		static int32_t iSpeedFilt = -1;
		if (iSpeedFilt < 0)
			iSpeedFilt = (int32_t)adc_buffer.speed << 4;
		iSpeedFilt += adc_buffer.speed - (iSpeedFilt >> 4); // ~160 ms time constant at 10 ms update
		int16_t iSpeedAdc = iSpeedFilt >> 4;

		// MIN > MAX also works for a reversed pot
		int32_t iPot = ((int32_t)iSpeedAdc - REMOTE_ADC_MIN) * 1000 / (REMOTE_ADC_MAX - REMOTE_ADC_MIN);
		iPot = CLAMP(iPot, 0, 1000);
		iPot = iPot < REMOTE_ADC_DEADBAND ? 0 : (iPot - REMOTE_ADC_DEADBAND) * 1000 / (1000 - REMOTE_ADC_DEADBAND);
		steer = 0;

#ifdef REMOTE_ADC_PRINT_ONLY
		speed = 0;
		bRemoteTimeout = 1;
#else
		if (iDrivingMode == 1) // main.c applies SPEED_COEFFICIENT only in pwm mode
			speed = iPot * REMOTE_ADC_MAX_REVS1024 / 1000 * SPEED_COEFFICIENT;
		else
			speed = iPot;
#endif

#ifdef RTT_REMOTE
		if (iTimeNextLog < msTicks)
		{
			iTimeNextLog = msTicks + 200;
#ifdef REMOTE_ADC_PRINT_ONLY
			sprintf(sMessage, "adc raw: %4u\tfilt: %4d\tpot: %5ld\n",
					adc_buffer.speed, iSpeedAdc, (long)iPot);
#else
			sprintf(sMessage, F2IF " V\t" F2IF " A\tadc: %4d\ttarget: %6ld\trevs: %6ld\n",
					F2I(batteryVoltage), F2I(currentDC), iSpeedAdc, (long)speed,
					(long)(revs32 >> (REVS32_SHIFT - 10)));
#endif
#ifdef WINDOWS_RN
			add_cr_before_lf_inplace(sMessage, sizeof(sMessage));
#endif
			SEGGER_RTT_WriteString(0, sMessage);
		}
#endif
	}
#endif
}

void RemoteCallback(void) {};

#endif
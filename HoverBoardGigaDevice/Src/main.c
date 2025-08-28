#define ARM_MATH_CM3


#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/bldc.h"
#include "../Inc/commsMasterSlave.h"

//#include "../Inc/commsSteering.h"

//#include "../Inc/commsBluetooth.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>     
//#include "arm_math.h" 
#ifdef MPU_6050
	#include "../Inc/mpu6050.h"
	extern ErrStatus    mpuStatus;                  // holds the MPU-6050 status: SUCCESS or ERROR
#endif


#ifdef BUZZER
	extern uint8_t buzzerFreq;    						// global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
	extern uint8_t buzzerPattern; 						// global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
#endif


#ifndef STAND_STILL_THRESHOLD // Posible to override in config.h or define_*.h
#define STAND_STILL_THRESHOLD 50
#endif

uint8_t iDrivingMode = DRIVING_MODE;	//  0=pwm, 1=speed in revs*1024, (not yet: 3=torque, 4=iOdometer)
uint8_t bRemoteTimeout = 0; 	// any Remote can set this to 1 to disable motor (with soft brake)
uint8_t bPilotTimeout = 0;	// any Pilot can set this to 1 to disable motor (with soft brake)
uint32_t iBug = 0;
uint8_t iBug8 = -1;
uint32_t steerCounter = 0;								// Steer counter for setting update rate
int32_t speed = 0; 												// global variable for speed.    -1000 to 1000
int32_t speedShutoff = 0;
int16_t speedLimit = 1000;


#define STATE_LedGreen 1	
#define STATE_LedOrange 2	
#define STATE_LedRed 4	
#define STATE_LedUp 8
#define STATE_LedDown 16
#define STATE_LedBattLevel 32
#define STATE_Disable 64
#define STATE_Shutoff 128

uint8_t  wState = STATE_LedBattLevel;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
uint8_t  wStateSlave = STATE_LedBattLevel;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff

#ifdef MASTER_OR_SLAVE
	DataSlave oDataSlave;
#endif

int32_t ShutOff(void);

#ifdef MASTER_OR_SINGLE

int32_t steer = 0; 												// global variable for steering. -1000 to 1000
FlagStatus activateWeakening = RESET;			// global variable for weakening
FlagStatus beepsBackwards = SET;  			// global variable for beeps backwards
			
extern uint8_t failsafe_status;
			
extern float batteryVoltage; 							// global variable for battery voltage
extern float currentDC; 									// global variable for current dc
extern float realSpeed; 									// global variable for real Speed
uint8_t slaveError = 0;										// global variable for slave error
	

#ifndef REMOTE_AUTODETECT

extern FlagStatus timedOut;								// Timeoutvariable set by timeout timer

uint32_t inactivity_timeout_counter = 0;	// Inactivity counter

void ShowBatteryState(int8_t iLevel);

	const float lookUpTableAngle[181] =  
	{ -1, -0.937202577, -0.878193767, -0.822607884, -0.770124422, -0.720461266, -0.673369096, -0.628626737, -0.58603728, -0.545424828, -0.506631749, -0.46951635, -0.433950895, -0.399819915, -0.367018754, -0.335452314, -0.30503398, -0.275684674, -0.24733204, -0.219909731, -0.193356783, -0.167617063, -0.142638788, -0.118374098, -0.094778672, -0.071811398, -0.049434068, -0.027611115, -0.006309372, 0.014502141, 0.03485241, 0.054768601, 0.074276213, 0.093399224, 0.112160212, 0.130580478, 0.148680146, 0.166478264, 0.183992885, 0.201241154, 0.218239378, 0.235003093, 0.251547129, 0.267885663, 0.284032276, 0.3, 0.315801365, 0.331448439, 0.34695287, 0.362325923, 0.377578512, 0.392721236, 0.407764409, 0.422718089, 0.437592106, 0.45239609, 0.467139493, 0.48183162, 0.496481645, 0.511098642, 0.5256916, 0.540269454, 0.554841097, 0.569415411, 0.584001283, 0.598607627, 0.613243408, 0.627917665, 0.642639528, 0.657418247, 0.672263213, 0.687183982, 0.702190301, 0.717292134, 0.732499689, 0.747823448, 0.763274197, 0.778863056, 0.794601516, 0.810501473, 0.826575268, 0.842835728, 0.859296209, 0.875970644, 0.892873598, 0.910020317, 0.927426794, 0.94510983, 0.963087109, 0.981377271, 1, 1.018976116, 1.038327677, 1.058078086, 1.07825222, 1.098876565, 1.119979359, 1.141590767, 1.163743061, 1.186470823, 1.209811179, 1.23380405, 1.258492439, 1.283922754, 1.310145166, 1.33721402, 1.365188293, 1.394132116, 1.42411537, 1.455214362, 1.487512601, 1.521101681, 1.556082309, 1.592565485, 1.630673867, 1.670543366, 1.712325006, 1.7561871, 1.802317825, 1.85092826, 1.902255998, 1.956569473, 2.014173151, 2.075413814, 2.140688197, 2.210452351, 2.28523318, 2.365642792, 2.452396478, 2.546335439, 2.648455802, 2.75994605, 2.882235846, 3.01706052, 3.166547428, 3.333333333, 3.520726642, 3.732935875, 3.97539819, 4.255263139, 4.582124498, 4.96916252, 5.434992778, 6.006790189, 6.72584757, 7.658112588, 8.915817681, 10.70672711, 13.46326037, 18.25863694, 28.69242032, 68.95533643, -158.4943784, -36.21729907, -20.22896451, -13.92536607, -10.55089693, -8.447794056, -7.010715755, -5.965979741, -5.171786508, -4.547320366, -4.043147824, -3.62733258, -3.278323283, -2.981049637, -2.72465641, -2.501126036, -2.304408198, -2.129851283, -1.973820239, -1.833433222, -1.706376086, -1.590769119, -1.485069639, -1.387999671, -1.29849148, -1.21564602, -1.138700863, -1.067005175, -1};
#endif	// not REMOTE_AUTODETECT

#endif


	FlagStatus enable = RESET;
		int16_t pwmSlave = 0;

	

uint32_t iTimeNextLoop = 0;
//----------------------------------------------------------------------------
// MAIN function
//----------------------------------------------------------------------------
#if defined(PLATFORMIO) && TARGET==2
	uint32_t SystemCoreClock = 72000000;  // Define the missing symbol
#endif
int main (void)
{
	iBug = 1;
	ConfigRead();		// reads oConfig defined in defines.h from flash
	
	#ifdef MASTER_OR_SINGLE
		FlagStatus chargeStateLowActive = SET;
		int16_t pwmMaster = 0;
		int16_t scaledSpeed = 0;
		int16_t scaledSteer  = 0;
		float expo = 0;
		float steerAngle = 0;
		float xScale = 0;
	#endif
	
	//SystemClock_Config();
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);	//  Configure SysTick to generate an interrupt every millisecond

	Delay(1000);

	
	if (	Watchdog_init() == ERROR)	// Init watchdog
		while(1);	// If an error accours with watchdog initialization do not start device
	
	// Init Interrupts
	Interrupt_init();

	#if TARGET != 3	// did not work for gd32e230 :-/
		// Init timeout timer
		TimeoutTimer_init();
	#endif
	

	// Init GPIOs
	GPIO_init();
	#ifndef REMOTE_AUTODETECT
		DEBUG_LedSet(SET,0)
		//pinMode(LED_GREEN,	GPIO_MODE_IPU);		// input_pullup turns led on with target 2
		//digitalWrite(LED_GREEN,0);	// not working for taget 2
	
		#ifdef UPPER_LED
			digitalWrite(UPPER_LED,SET);
		#endif

		#ifdef SELF_HOLD
			// Activate self hold direct after GPIO-init
			digitalWrite(SELF_HOLD,SET);
		#endif
	#endif

	#ifdef USART0_BAUD
			USART0_Init(USART0_BAUD);
	#endif
	#ifdef USART1_BAUD
			USART1_Init(USART1_BAUD);
	#endif
	#ifdef USART2_BAUD
			USART2_Init(USART2_BAUD);
	#endif

	#ifdef I2C_ENABLE
		I2C_Init();
		i2c_scanner();
		dump_i2c_registers(0x68);
	#endif 

	#ifdef MPU_6050
		MPU_Init();
	#endif

	// Init ADC
	ADC_init();

	// Init PWM
	PWM_init();

	InitBldc();		// virtual function implemented by bldcBC.c and bldcSINE.c
	DriverInit(iDrivingMode);



/*
	// added by deepseek: Apply fixed PWM pattern to lock rotor to known position
	does heavily freeze the motor :-(
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, BLDC_TIMER_MID_VALUE);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, BLDC_TIMER_MID_VALUE);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, 0);
	uint32_t iTimeWait = millis() + 500;
	while (millis()<iTimeWait){	fwdgt_counter_reload();};
*/
	
	// Device has 1,6 seconds to do all the initialization
	// afterwards watchdog will be fired
	//while(1)
	fwdgt_counter_reload();

#ifdef REMOTE_AUTODETECT
  while(1)
	{
		if (millis() < iTimeNextLoop)	
			continue;
		iTimeNextLoop = millis() + DELAY_IN_MAIN_LOOP;
		
		steerCounter++;		// something like DELAY_IN_MAIN_LOOP = 5 ms
		
		if ((steerCounter % 2) == 0)	// something like DELAY_IN_MAIN_LOOP = 10 ms
		{
			RemoteUpdate();
		}
		SetEnable(1);

		// Reload watchdog (watchdog fires after 1,6 seconds)
		fwdgt_counter_reload();
	}
}

#else	

	// Startup-Sound
	BUZZER_MelodyDown()
	#ifdef BUTTON
		// Wait until button is released
		
		uint32_t iTimePushed = millis();
		while (BUTTON_PUSHED == digitalRead(BUTTON))
		{
			fwdgt_counter_reload();	// Reload watchdog while button is pressed
			#ifdef REMOTE_ADC
				if (millis()-iTimePushed > 2000)
				{
					iConfigMode = 1;
					BuzzerSet(8,2);	// (iFrequency, iPattern)
				}
			#endif
		} 
		Delay(10); //debounce to prevent immediate ShutOff (100 is to much with a switch instead of a push button)
	#endif

	DEBUG_LedSet(RESET,0)
	//pinMode(LED_GREEN,	GPIO_MODE_OUT_PP);	//turns off led on target 2
	//digitalWrite(LED_GREEN,1);	// not working for target 2

	#ifdef UPPER_LED
		digitalWrite(UPPER_LED,RESET);
	#endif
iBug = 10;

	while(1)
	{
		if (millis() < iTimeNextLoop)	
			continue;
		iTimeNextLoop = millis() + DELAY_IN_MAIN_LOOP;
		steerCounter++;		// something like DELAY_IN_MAIN_LOOP = 5 ms
		DEBUG_LedSet(	(steerCounter%20) < 10	,0)
		//DEBUG_LedSet(	digitalRead(BUTTON)	,1)		
		//iBug = digitalRead(BUTTON);
		
		
		#ifdef MOSFET_OUT
			digitalWrite(MOSFET_OUT,	(steerCounter%200) < 100	);	// onboard led blinking :-)
		#endif
		
		#ifdef SLAVE	
			SetBldcInput(pwmSlave);
		#else	//MASTER_OR_SINGLE
			if ((steerCounter % 2) == 0)	// something like DELAY_IN_MAIN_LOOP = 10 ms
			{
				#if defined(PILOT_DEFAULT) && defined(SEND_IMU_DATA)
					if (MPU_ReadAll() != SUCCESS) 
					{
						I2C_Init();	// try to re-init if i2c bus fails. Need when I2C_SPEED is 400000 instead of 200000
						//MPU_Init();		// re-init mpu6050 (not really needed)
						iBug++;
					}
				#endif
				
				RemoteUpdate();
				//DEBUG_LedSet(RESET,0)
			}
			
			if (speedShutoff)	speed = ShutOff();
			
			#ifdef PILOT_DEFAULT
				if (iDrivingMode == 0)
				{
					// Calculate expo rate for less steering with higher speeds
					expo = MAP((float)ABS(speed), 0, 1000, 1, 0.5);
					
					// Each speedvalue or steervalue between 50 and -50 (STAND_STILL_THRESHOLD) means absolutely no pwm
					// -> to get the device calm 'around zero speed'
					scaledSpeed = speed < STAND_STILL_THRESHOLD && speed > -STAND_STILL_THRESHOLD ? 0 : CLAMP(speed, -speedLimit, speedLimit) * SPEED_COEFFICIENT;
					scaledSteer = steer < STAND_STILL_THRESHOLD && steer > -STAND_STILL_THRESHOLD ? 0 : CLAMP(steer, -speedLimit, speedLimit) * STEER_COEFFICIENT * expo;
					
					// Map to an angle of 180 degress to 0 degrees for array access (means angle -90 to 90 degrees)
					steerAngle = MAP((float)scaledSteer, -1000, 1000, 180, 0);
					xScale = lookUpTableAngle[(uint16_t)steerAngle];

					// Mix steering and speed value for right and left speed
					if(steerAngle >= 90)
					{
						pwmSlave = CLAMP(scaledSpeed, -1000, 1000);
						pwmMaster = CLAMP(pwmSlave / xScale, -1000, 1000);
					}
					else
					{
						pwmMaster = CLAMP(scaledSpeed, -1000, 1000);
						pwmSlave = CLAMP(xScale * pwmMaster, -1000, 1000);
					}
				}
				else
				{
					// simply boost/drop speed left and right according to steer going from -1.0 to +1.0
					pwmMaster = speed * (1+steer/1000.0);		
					pwmSlave = speed * (1-steer/1000.0);
				}
			#else
				Pilot(&pwmMaster,&pwmSlave);
			#endif
			
					// Set output
			SetBldcInput(pwmMaster);

			#ifdef USART_MASTERSLAVE
				// Decide if slave will be enabled
				if  ((enable == SET && timedOut == RESET))
					wStateSlave = wStateSlave & !STATE_Disable;
				else
					wStateSlave |= STATE_Disable;

				#ifdef MASTER
					SendSlave(-pwmSlave);
				#endif

			#endif

			if (batteryVoltage > BAT_LOW_LVL1)	// Show green battery symbol when battery level BAT_LOW_LVL1 is reached
			{
				ShowBatteryState(0);
				#ifdef BEEP_BACKWARDS
				if (beepsBackwards == SET && speed < -50)
					{
						BuzzerSet(5,4)	// (iFrequency, iPattern)
					}
					else
				#endif
				{
					BuzzerSet(0,0)	// (iFrequency, iPattern)
				}
			}
			else if (batteryVoltage > BAT_LOW_LVL2) // Make silent sound and show orange battery symbol when battery level BAT_LOW_LVL2 is reached
			{
				ShowBatteryState(1);
				#ifdef BATTERY_LOW_BEEP
					BuzzerSet(6,3)	// (iFrequency, iPattern)
				#endif
			}
			else if  (batteryVoltage > BAT_LOW_DEAD) // Make even more sound and show red battery symbol when battery level BAT_LOW_DEAD is reached
			{
				ShowBatteryState(2);
				#ifdef BATTERY_LOW_BEEP
					BuzzerSet(6,2)	// (iFrequency, iPattern)
				#endif
			}
			else 	// Shut device off, when battery is dead
			{
				ShowBatteryState(3);
				#ifdef BATTERY_LOW_BEEP
					BuzzerSet(6,1)	// (iFrequency, iPattern)
				#endif
				#ifdef BATTERY_LOW_SHUTOFF
					ShutOff();
				#endif
			}

			#ifdef BUTTON
				// Shut device off when button is pressed
				if (BUTTON_PUSHED == digitalRead(BUTTON))
				//if (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN))
				{
					while (BUTTON_PUSHED == digitalRead(BUTTON)) {fwdgt_counter_reload();}
					//while (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN)) {fwdgt_counter_reload();}
					ShutOff();
				}
			#endif
			
			// Calculate inactivity timeout (Except, when charger is active -> keep device running)
			if (ABS(pwmMaster) > 50 || ABS(pwmSlave) > 50 || !chargeStateLowActive)
			{
				inactivity_timeout_counter = 0;
			}
			else
			{
				inactivity_timeout_counter++;
			}
			
			#ifdef INACTIVITY_TIMEOUT
				// Shut off device after INACTIVITY_TIMEOUT in minutes
				if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1))
				{ 
					ShutOff();
				}
			#endif
		#endif	

		#if defined(CHARGE_STATE) && defined(MASTER_OR_SINGLE)
			chargeStateLowActive = digitalRead(CHARGE_STATE);
			enable = chargeStateLowActive;
		#else
			enable = SET;			
		#endif
		
		if (bPilotTimeout || bRemoteTimeout || (wState & STATE_Disable))	enable = RESET;
		
		// Enable is depending on arm switch
		//#ifdef USART_CRSF
		//enable &= (getChannel(4)-1024) > 0; //arm switch doesn't work
		//#endif
		//enable = SET;		// robo testing
		//^enable = RESET;
		
		//DEBUG_LedSet(enable,0);	// macro. iCol: 0=green, 1=organge, 2=red
			
		// Enable channel output
		SetEnable(enable);

		//wState = STATE_LedBattLevel;	// overwrite wState recevied from RemoteUart or RemoteUartBus
		
		#if (!defined(TEST_HALL2LED)) && (!defined(DEBUG_LED))
			if (!(wState & STATE_LedBattLevel))
			{
				#ifdef LED_GREEN
					digitalWrite(LED_GREEN,wState & STATE_LedGreen ? SET : RESET);
				#endif
				#ifdef LED_ORANGE
					digitalWrite(LED_ORANGE,wState & STATE_LedOrange ? SET : RESET);
				#endif
				#ifdef LED_RED
					digitalWrite(LED_RED,wState & STATE_LedRed ? SET : RESET);
				#endif
			}
		#endif
		#ifdef UPPER_LED
			digitalWrite(UPPER_LED,wState & STATE_LedUp ? SET : RESET);
		#endif
		#ifdef UPPER_LED
			digitalWrite(UPPER_LED,wState & STATE_LedDown ? SET : RESET);
		#endif

		if (wState & STATE_Shutoff)	ShutOff();


		//Delay(DELAY_IN_MAIN_LOOP);
		
		fwdgt_counter_reload(); // Reload watchdog until device is off
  }
}

//----------------------------------------------------------------------------
// Turns the device off
//----------------------------------------------------------------------------

int32_t ShutOff(void)
{
	
	if (speedShutoff)
	{
		speedShutoff += speedShutoff > 0 ? -5 : +5 ;
		if (ABS(speedShutoff) > 10)
			return speedShutoff;
	}
	else if (ABS(speed)>10)
	{
		speedShutoff = speed;
		return speedShutoff;
	}
	
	BUZZER_MelodyUp()
	
	#ifdef USART_MASTERSLAVE
	
		#ifdef MASTER
			// Send shut off command to slave
			wStateSlave = STATE_Shutoff;
			SendSlave(0);
		#endif

	// Disable usart
		usart_deinit(USART_MASTERSLAVE);
	#endif
	
	// Set pwm and enable to off
	SetEnable(RESET);
	iDrivingMode = 0;
	SetBldcInput(0);
	
	#ifdef SELF_HOLD
		digitalWrite(SELF_HOLD,RESET);
	#endif
	while(1)	fwdgt_counter_reload(); // Reload watchdog until device is off
}


#ifdef MASTER_OR_SINGLE

//----------------------------------------------------------------------------
// Shows the battery state on the LEDs
//----------------------------------------------------------------------------
void ShowBatteryState(int8_t iLevel)
{
	#ifdef DEBUG_LED
		return;
	#else
		if (!(wState & STATE_LedBattLevel))
			return;
		uint32_t aPin[3];
		uint8_t iLedCount=0;
		#if defined(LED_GREEN)
			aPin[iLedCount++] = LED_GREEN;
		#endif
		#if defined(LED_ORANGE)
			aPin[iLedCount++] = LED_ORANGE;
		#endif
		#if defined(LED_RED)
			aPin[iLedCount++] = LED_RED;
		#endif

		if (iLedCount == 3)		// green -> orange -> red -> blinking red
		{
			int8_t i=2; 
			if (iLevel == 3)
			{
				digitalWrite(aPin[2],(steerCounter%50) < 25);		// red led blinking
				i--;
			}
			for(;i>=0;i--)
			{
				digitalWrite(aPin[i],i==iLevel);
			}
			
		}
		else if (iLedCount == 2)	// show the level 0-3 in 2 bits
		{
			digitalWrite(aPin[0],iLevel & 0b01);		
			digitalWrite(aPin[1],iLevel & 0b10);		
		}
		else if (iLedCount == 1)
		{
			if (iLevel == 0) 		
				digitalWrite(aPin[0],1);		
			else
			{
				uint16_t iPeriod = 300/(2*iLevel);
				digitalWrite(aPin[0],(steerCounter%iPeriod) < (iPeriod/2)	);		// led blinking
			}
		}
	#endif
}

#endif


#endif

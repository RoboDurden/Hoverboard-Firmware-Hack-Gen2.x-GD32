#ifndef DEFINES_H
#define DEFINES_H

#include "../Inc/target.h"
#include "../Inc/setup.h"
#include "../Inc/configSelect.h"
#include "../Inc/remote.h"


#if defined(REMOTE_AUTODETECT)
	#include "defines/defines_2-ad.h"		// https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/??
	
	#define SINGLE
	#define MASTER_OR_SINGLE
	#define BAT_CELLS         	6        // battery number of cells. Normal Hoverboard battery: 10s
	#define SPEED_COEFFICIENT   -1
	#define STEER_COEFFICIENT   1
	#define BLDC_BC
	
#else
	#define STRINGIZE_AUX(a) #a
	#define STRINGIZE(a) STRINGIZE_AUX(a)
	#define INCLUE_FILE(target,version) STRINGIZE(defines/defines_2-target-version.h)

	#include INCLUE_FILE(TARGET , LAYOUT)	// "defines_2-target-version.h"
#endif



#ifdef BUTTON
	#define BUTTON_PUSHED 1
#else
	#ifdef BUTTON_PU
		#define BUTTON_PUSHED 1		// very strangely, even so the button needs a pullup, digitalRead gives 1 when button pushed
		#define BUTTON BUTTON_PU
	#endif
#endif

#ifdef DISABLE_BUTTON
	#undef BUTTON
	#undef SELF_HOLD
#endif

#ifdef BLDC_SINE
	#if PWM_FREQ > 12000
		#undef PWM_FREQ
		#define PWM_FREQ	12000		// 16 kHz seems to be to fast for the longer CalculateBLDC code execution and the 3 hall interrupts
	#endif
#endif

#define BLDC_TIMER_PERIOD       (72000000u / 2u / PWM_FREQ) // = 2250

#ifndef TIMER_BLDC	// these defines should be equal for all Gen2 boards as they only have on bldc capable TIMER = TIMER0
	#define TIMER_BLDC 		TIMER0
	#define RCU_TIMER_BLDC 		RCU_TIMER0
	#define TIMER_BLDC_CHANNEL_G 	TIMER_CH_2
	#define TIMER_BLDC_CHANNEL_B 	TIMER_CH_1
	#define TIMER_BLDC_CHANNEL_Y 	TIMER_CH_0
#endif

#ifndef TIMER_TIMEOUT
	#define TIMER_TIMEOUT TIMER13
	#define TIMEOUT_IrqHandler TIMER13_IRQHandler
	#define RCU_TIMER_TIMEOUT	RCU_TIMER13
	#define TIMER_TIMEOUT_IRQn TIMER13_IRQn
#endif

/*
#ifdef DEBUG_LED_PIN
  #define DEBUG_LedSet(bSet){gpio_bit_write(DEBUG_LED_PORT, DEBUG_LED_PIN, bSet);}
#else
  #define DEBUG_LedSet(bSet)
#endif
*/

#ifdef DEBUG_LED
#define DEBUG_LedSet(bSet,iCol){digitalWrite((iCol==0 ? LED_GREEN : (iCol==2 ? LED_ORANGE : LED_RED)), bSet);}
#else
  #define DEBUG_LedSet(bSet,iCol)
#endif

#ifdef REMOTE_ADC
	#ifndef USART1_TX
		#error "REMOTE_ADC only works with USART1 PA2/PA3"
	#endif
	#undef MASTERSLAVE_USART
	#undef REMOTE_USART
	#if defined(USART0_TX) && defined(MASTER)	// only uart left
		#define MASTERSLAVE_USART 0
	#endif
#endif

/*
#if defined(MASTERSLAVE_USART) && defined(REMOTE_USART)
	#if !defined(USART0_TX) || !defined(USART1_TX)
		error"you board only has one USART. Choose between MASTERSLAVE_USART and REMOTE_USART"
	#endif
#endif
*/

#if defined(MASTER) || defined(SLAVE)
	#define MASTER_OR_SLAVE
#endif


//#if defined(MASTER_OR_SLAVE) && (!defined(USART0_MASTERSLAVE)) && (!defined(USART1_MASTERSLAVE))
#if defined(MASTER_OR_SLAVE) && (!defined(MASTERSLAVE_USART))
	#error "MASTER or SLAVE set in config.h but no but no uart available. Please choose SINGLE (and REMOTE_UARTBUS)"
#endif

#if (defined(REMOTE_UART) || defined(REMOTE_UARTBUS) || defined(REMOTE_CRSF)) && !defined(REMOTE_USART)
	#error "a usart remote selected in config.h but neither USART0_REMOTE nor USART1_REMOTE in your defines_2-?.h"
#endif

#if defined(MASTERSLAVE_USART) && defined(REMOTE_USART) 
	#if MASTERSLAVE_USART == REMOTE_USART
		#error "MASTERSLAVE_USART must be different from REMOTE_USART"
	#endif
#endif

#ifdef REMOTE_USART
	#if REMOTE_USART == 0
		#define HAS_USART0
		#define USART0_BAUD REMOTE_BAUD		// defined in remoteUart.h or remoteCrsf.h or remoteUartBus.h
		#define USART_REMOTE USART0
		#define USART_REMOTE_BUFFER usart0_rx_buf		// defined in setup.c
	#else 
		#if REMOTE_USART == 1
			#define HAS_USART1
			#define USART1_BAUD REMOTE_BAUD		// defined in remoteUart.h or remoteCrsf.h or remoteUartBus.h
			#define USART_REMOTE USART1
			#define USART_REMOTE_BUFFER usart1_rx_buf		// defined in setup.c
		#else 
			#if REMOTE_USART == 2
				#define HAS_USART2
				#define USART2_BAUD REMOTE_BAUD		// defined in remoteUart.h or remoteCrsf.h or remoteUartBus.h
				#define USART_REMOTE USART2
				#define USART_REMOTE_BUFFER usart2_rx_buf		// defined in setup.c
			#else
				#error "no REMOTE_USART choosen (0, 1 or 2)"
			#endif
		#endif
	#endif
#endif

#ifdef MASTERSLAVE_USART
	#if MASTERSLAVE_USART == 0
		#define HAS_USART0
		#define USART0_BAUD 115200
		#define USART_MASTERSLAVE USART0
		#define USART_MASTERSLAVE_BUFFER usart0_rx_buf		// defined in setup.c
	#else
			#if MASTERSLAVE_USART == 1
				#define HAS_USART1
				#define USART1_BAUD 115200
				#define USART_MASTERSLAVE USART1
				#define USART_MASTERSLAVE_BUFFER usart1_rx_buf		// defined in setup.c
			#else
			 	#if MASTERSLAVE_USART == 2
				#define HAS_USART2
				#define USART2_BAUD 115200
				#define USART_MASTERSLAVE USART2
				#define USART_MASTERSLAVE_BUFFER usart2_rx_buf		// defined in setup.c
			#else
			#endif
		#endif
	#endif
#endif


	
// ADC value conversion defines
#ifndef MOTOR_AMP_CONV_DC_AMP
	#define MOTOR_AMP_CONV_DC_AMP 0.201465201465  // 3,3V * 1/3 - 0,004Ohm * IL(ampere) = (ADC-Data/4095) *3,3V
#endif
#ifndef ADC_BATTERY_VOLT
	#define ADC_BATTERY_VOLT      0.024169921875  	// V_Batt to V_BattMeasure = factor 30: ( (ADC-Data/4095) *3,3V *30 )
#endif

#define BAT_LOW_LVL1     BAT_CELLS * CELL_LOW_LVL1	
#define BAT_LOW_LVL2     BAT_CELLS * CELL_LOW_LVL2
#define BAT_LOW_DEAD     BAT_CELLS * CELL_LOW_DEAD


#ifdef BUZZER
	extern uint8_t buzzerFreq;    						// global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
	extern uint8_t buzzerPattern; 						// global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
  #define BuzzerSet(iFrequency, iPattern) {buzzerFreq = iFrequency;buzzerPattern = iPattern;}
  #define BUZZER_MelodyDown(){int8_t iFreq=8;for (; iFreq>= 0; iFreq--){ buzzerFreq = iFreq; Delay(100); } buzzerFreq = 0;}
  #define BUZZER_MelodyUp(){int8_t iFreq=0; for (; iFreq<= 8; iFreq++){ buzzerFreq = iFreq; Delay(100); } buzzerFreq = 0;}
#else
  #define BuzzerSet(iFrequency, iPattern)
  #define BUZZER_MelodyDown()
  #define BUZZER_MelodyUp()
#endif


// Useful math function defines
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#ifndef MAX
	#define MAX(x, high) (((x) > (high)) ? (high) : (x))
#endif	
#define MAP(x, xMin, xMax, yMin, yMax) ((x - xMin) * (yMax - yMin) / (xMax - xMin) + yMin)

// ################################################################################

// ADC buffer struct
typedef struct
{
  uint16_t v_batt;
	uint16_t current_dc;
	#ifdef REMOTE_ADC
		uint16_t speed;
		uint16_t steer;
	#endif
	
} adc_buf_t;

//#pragma pack(1)
typedef struct
{
  uint8_t wState;
	float currentDC; 									// global variable for current dc
	float realSpeed; 									// global variable for real Speed
	int32_t iOdom;	
} DataSlave;



#ifdef REMOTE_AUTODETECT
	#define PINS_DETECT 18
		
	#define EEPROM_VERSION 1000
	#pragma pack(push, 1)
	typedef struct
	{
		uint32_t iVersion;
		uint16_t wState;
		int8_t aiPinScan[PINS_DETECT];		// lists NOT the uint32_t PA7 but the index of PA7 in PinAD aoPin[COUNT_PinDigital]
		uint8_t padding[1];  // Pad to x*4 bytes for word alignment.
	} ConfigData;
	#pragma pack(pop)
#else	
	#define EEPROM_VERSION 2

	#pragma pack(push, 1)
	typedef struct {
			uint32_t iVersion;
			uint16_t wState;
			uint16_t iSpeedNeutral;		// = 2048 REMOTE_ADC
			uint16_t iSteerNeutral;		// = 2048 REMOTE_ADC
			uint16_t iSpeedMax;				// = 4096 REMOTE_ADC
			uint16_t iSpeedMin;				// = 0		REMOTE_ADC
			uint16_t iSteerMax;				// = 4096	REMOTE_ADC
			uint16_t iSteerMin;				// = 0		REMOTE_ADC
			uint8_t padding[2];  // Pad to 20 bytes for word alignment. added by Deepseek
	} ConfigData;
	#pragma pack(pop)
#endif



#endif		// DEFINES_H

// robo: 2.13 = very much like 2.0 ! But USART0 only on empty header of bluetooth module,
//	different hall order and an optional third led header (with PB5 and a blinking output)

#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
	#define HAS_BUZZER
#endif

/// autodetect by robo durden 2025/06/06
#define HALL_A		PB11
#define HALL_B		PC14
#define HALL_C		PF1
#define PHASE_A		PB0
#define PHASE_B		PB1
//#define PHASE_C		P??

#define LED_RED			PB3
#define LED_ORANGE	PA12
#define LED_GREEN		PA15
#define UPPER_LED		PA1
#define LOWER_LED		PA0
//#define ONBOARD_LED		P??
#define BUZZER		PB10

#define VBATT				PA4
#define CURRENT_DC	PA6
#define SELF_HOLD		PB2
#define BUTTON			PC15
//#define BUTTON_PU		P??



#define UPPER_LED2	PB5		//	robo: the middle pin of the 3pin led header between the 3led (left) and 2led (right)
													//	VBATT!!! , PB5 , ? = simply blinking,
// Mosfet output, little onboard led
// #define MOSFET_OUT	PC13 not showing up with autodetect

// Brushless Control DC (BLDC) defines
#define BLDC_GH PA10		// green	, Tommyboi2001 all bldc pins same as 2.0
#define BLDC_GL PB15		
#define BLDC_BH PA9			// blue
#define BLDC_BL PB14		
#define BLDC_YH PA8			// yellow
#define BLDC_YL PB13		
#define TIMER_BLDC_PULLUP	GPIO_PUPD_NONE	// robo: not sure if some boards indeed nned GPIO_PUPD_PULLUP like 2.2 or 2.3


// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define USART0_TX	PB6
#define USART0_RX	PB7

// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4
#define USART1_TX		PA2
#define USART1_RX		PA3
	
#define ADC_BATTERY_VOLT      0.0258320368	// robo newly gaged

#define CHARGE_STATE	PF0			// motors will stop if charger is pluged in. disable if you use the charger to power the board

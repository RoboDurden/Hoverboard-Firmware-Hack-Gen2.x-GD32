// from https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/20
#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
	#define HAS_BUZZER
#endif

// WARNING, this is the 2.1.4 defines and only as a placeholder to let firmware compile

// autodetect 2024/04/05 by robo
#define HALL_A		PA1
#define HALL_B		PA0
#define HALL_C		PA2

/*
#define LED_RED		PA15
#define LED_ORANGE		PB4
#define LED_GREEN		PB3
#define UPPER_LED		PC13
#define LOWER_LED		PB5
//#define ONBOARD_LED		P??
#define BUZZER		PA11

#define SELF_HOLD		PA3
//#define BUTTON		P??
#define BUTTON_PU		PA4
*/

#define VBATT		PA5
#define CURRENT_DC		PA7


#define ADC_BATTERY_VOLT      0.025392524927  	//	robo, calibrated with 27V power supply
#define MOTOR_AMP_CONV_DC_AMP 0.0201465201465		//	robo,  only a very quick comparison to multimeter


// Brushless Control DC (BLDC) defines
#define BLDC_GH PA10		// green	, Tommyboi2001 all bldc pins same as 2.0
#define BLDC_GL PB15		
#define BLDC_BH PA9			// blue
#define BLDC_BL PB14		
#define BLDC_YH PA8			// yellow
#define BLDC_YL PB13		
#define TIMER_BLDC_PULLUP	GPIO_MODE_AF_PP	// robo: not sure if some boards indeed nned GPIO_PUPD_PULLUP like 2.2 or 2.3

// Timer BLDC short circuit emergency shutoff define
//#define TIMER_BLDC_EMERGENCY_SHUTDOWN	TODO_PIN


// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define USART0_TX	PB6
#define USART0_RX	PB7


// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4

// master-slave header
#define USART2_TX	PB10
#define USART2_RX	PB11


// from https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/20
#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
	#define HAS_BUZZER
#endif

// WARNING, this is the 2.1.4 defines and only as a placeholder to let firmware compile

// autodetect 2024/04/05 by robo
#define HALL_A		PA1
#define HALL_B		PA0
#define HALL_C		PA2

#define LED_GREEN		PB3		// autodetect :-)
#define LED_RED			PA15
#define LED_ORANGE		PB4

/*
*/
#define UPPER_LED		PC13
#define LOWER_LED		PB5
//#define ONBOARD_LED		P??
#define BUZZER		PB1

#define SELF_HOLD		PA4
#define BUTTON		PA3
//#define BUTTON_PU		PA3

#define VBATT		PA5
#define CURRENT_DC		PA7


#define ADC_BATTERY_VOLT      0.0121565132741  	//	robo, calibrated with 27V power supply
#define MOTOR_AMP_CONV_DC_AMP 0.0201465201465		//	robo,  only a very quick comparison to multimeter


// Brushless Control DC (BLDC) defines
#define BLDC_GH PA10		// green	, Tommyboi2001 all bldc pins same as 2.0
#define BLDC_GL PB15		
#define BLDC_BH PA9			// blue
#define BLDC_BL PB14		
#define BLDC_YH PA8			// yellow
#define BLDC_YL PB13		
#define TIMER_BLDC_PULLUP	GPIO_MODE_AF_PP	// robo: not sure if some boards indeed nned GPIO_PUPD_PULLUP like 2.2 or 2.3

// Brushless Control DC (BLDC) defines
// Channel G
#define RCU_TIMER_BLDC RCU_TIMER0
#define TIMER_BLDC TIMER0
#define TIMER_BLDC_CHANNEL_G TIMER_CH_2
#define TIMER_BLDC_GH_PIN GPIO_PIN_10
#define TIMER_BLDC_GH_PORT GPIOA
#define TIMER_BLDC_GL_PIN GPIO_PIN_15
#define TIMER_BLDC_GL_PORT GPIOB
// Channel B
#define TIMER_BLDC_CHANNEL_B TIMER_CH_1
#define TIMER_BLDC_BH_PIN GPIO_PIN_9
#define TIMER_BLDC_BH_PORT GPIOA
#define TIMER_BLDC_BL_PIN GPIO_PIN_14
#define TIMER_BLDC_BL_PORT GPIOB
// Channel Y
#define TIMER_BLDC_CHANNEL_Y TIMER_CH_0
#define TIMER_BLDC_YH_PIN GPIO_PIN_8
#define TIMER_BLDC_YH_PORT GPIOA
#define TIMER_BLDC_YL_PIN GPIO_PIN_13
#define TIMER_BLDC_YL_PORT GPIOB

#define USART_STEER_COM USART0
#define USART_STEER_COM_TX_PIN GPIO_PIN_6
#define USART_STEER_COM_TX_PORT GPIOB
#define USART_STEER_COM_RX_PIN GPIO_PIN_7
#define USART_STEER_COM_RX_PORT GPIOB


// Timer BLDC short circuit emergency shutoff define
//#define TIMER_BLDC_EMERGENCY_SHUTDOWN	TODO_PIN


// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define USART0_TX	PB6
#define USART0_RX	PB7


// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4

// master-slave header
#define USART2_TX	PB10
#define USART2_RX	PB11


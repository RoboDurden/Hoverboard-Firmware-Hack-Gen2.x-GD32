// Gen2_2.1

#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
	#define HAS_BUZZER
#endif

#define HALL_A		PA1
#define HALL_B		PA0
#define HALL_C		PA2

#define LED_RED			PB3
#define LED_ORANGE	PA15
#define LED_GREEN		PA12
#define UPPER_LED		PA11
//#define ONBOARD_LED		P??

#define BUZZER			PB1

#define VBATT		PA5
#define CURRENT_DC		PA7

#define SELF_HOLD		PA4
#define BUTTON		PA3
//#define BUTTON_PU		PA3


#define ADC_BATTERY_VOLT      0.01205891378
#define MOTOR_AMP_CONV_DC_AMP 0.201465201465	// 3,3V * 1/3 - 0,004Ohm * IL(ampere) = (ADC-Data/4095) *3,3V


// Brushless Control DC (BLDC) defines
#define BLDC_GH PA10		// green	, Tommyboi2001 all bldc pins same as 2.0
#define BLDC_GL PB15		
#define BLDC_BH PA9			// blue
#define BLDC_BL PB14		
#define BLDC_YH PA8			// yellow
#define BLDC_YL PB13		
#define TIMER_BLDC_PULLUP	GPIO_MODE_AF_PP


// Timer BLDC short circuit emergency shutoff define
//#define TIMER_BLDC_EMERGENCY_SHUTDOWN	TODO_PIN


// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define USART0_TX	PB6
#define USART0_RX	PB7


// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4
/*
// Usart master slave defines JMA OK
#define USART_MASTERSLAVE USART2 //was USART1
#define USART_MASTERSLAVE_TX_PIN GPIO_PIN_10 //was GPIO_PIN_2
#define USART_MASTERSLAVE_TX_PORT GPIOB //was GPIOA
#define USART_MASTERSLAVE_RX_PIN GPIO_PIN_11 //was GPIO_PIN_3
#define USART_MASTERSLAVE_RX_PORT GPIOB //was GPIOA
*/


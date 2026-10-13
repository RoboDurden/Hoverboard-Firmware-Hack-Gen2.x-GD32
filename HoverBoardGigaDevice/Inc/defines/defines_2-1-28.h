/**
 * @file    target_board_config.h
 * @brief   Pin mapping for Hoverboard PCB two-sys_V1.4
 * @note    Board features 3 gate driver chips (FD2103).
 * @note    IMPORTANT: Swap Green and Blue motor wires for correct Hall sync.
 *          Work In Progress
 * @see     https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x
 */


#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
	#define HAS_BUZZER
#endif
// 'r'=red,	'o'=orange,	'g'=green,	'u'=up,	'd'=down,	'p'=pcb led,	'b'=buzzer
#define LED_RED		PB4
#define LED_ORANGE		PB8
#define LED_GREEN		PB3
//#define UPPER_LED		P??
//#define LOWER_LED		P??
//#define ONBOARD_LED		P??
#define BUZZER		PB9

// Brushless Control DC (BLDC) defines
#define BLDC_GH PB15
#define BLDC_GL PA10
#define BLDC_BH PB14
#define BLDC_BL PA9
#define BLDC_YH PB13
#define BLDC_YL PA8

#define TIMER_BLDC_PULLUP	GPIO_PUPD_NONE	// robo: not sure if some boards indeed nned GPIO_PUPD_PULLUP like 2.2 or 2.3



// Hall sensor defines

#define HALL_A	PA1
#define HALL_B	PC14
#define HALL_C	PB11

// #define HALL_A	PA1
// #define HALL_B	PB11
// #define HALL_C	PC14

// #define HALL_A	PB11
// #define HALL_B	PA1
// #define HALL_C	PC14

// #define HALL_A	PB11
// #define HALL_B	PC14
// #define HALL_C	PA1

// #define HALL_A	PC14
// #define HALL_B	PB11
// #define HALL_C	PA1

// #define HALL_A	PC14
// #define HALL_B	PA1
// #define HALL_C	PB11

// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define USART0_TX	PB6
#define USART0_RX	PB7

// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4
#define USART1_TX		PA2
#define USART1_RX		PA3

#define VBATT	PA0
// from https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/20

#ifdef MASTER_OR_SINGLE		// layout 2.2 has buzzer on the slave board.
	#define HAS_BUZZER
#endif

/* GD32F130 48pin possible IO pins: 
	C13 C14 C15 F0 F1 A0 A1 A2 
	A3 A4 A5 A6 A7 B0 B1 B2 B10 B11
	B12 B13 B14 B15 A8 A9 A10 A11 A12 A13 F6 F7
	A14 A15 B3 B4 B5 B6 B7 B8 B9 
	
	mostly used for 6 BLDC mosfet pins: B13 B14 B15 A8 A9 A10
	mostly used for USART0: B6 B7
	mostly used for USART1: A2 A3
	ST-Flash pins: A13 A14 (also used as green and red led on 2.2)
	
	so mostly available for other use:	
	C13 C14 C15 F0 F1 A0 A1 A4 A5 A6 A7 B0 B1 B2 B10 B11 B12 A11 F6 F7 A12 A15 B3 B4 B5 B8 B9 
	so available for analog input:
	A0 A1 A2 A3 A4 A5 A6 A7 B0 B1 	
*/

// LED defines, colors probably mismatch !
#define LED_GREEN 			PA5
#define LED_ORANGE 			PA14
#define LED_RED 				PB3

#define UPPER_LED 	PB4
#define LOWER_LED 	PB5

//#define MOSFET_OUT TODO_PIN		// onboard led on some boards

// Brushless Control DC (BLDC) defines
#define TIMER_BLDC_PULLUP	GPIO_PUPD_PULLUP	// robo, based on Herleybob:defines.h
// Channel G
#define BLDC_GH		PA8		// channels G=green and Y=yellow swopped compared to 2.0
#define BLDC_GL		PB13
// Channel B
#define BLDC_BH		PA9
#define BLDC_BL		PB14
// Channel Y
#define BLDC_YH		PA10
#define BLDC_YL		PB15

// Timer BLDC short circuit emergency shutoff define
// Is initialized here but never used somewhere else in code.
// setup.c:176	gpio_mode_set(TIMER_BLDC_EMERGENCY_SHUTDOWN_PORT , GPIO_MODE_AF, GPIO_PUPD_NONE, TIMER_BLDC_EMERGENCY_SHUTDOWN);  
//#define TIMER_BLDC_EMERGENCY_SHUTDOWN		TODO_PIN	// TODO

// Hall sensor defines
// mateuszfcg tested with PA1,PA2,PA0 
//#define HALL_A	PB0	// robo from front+back-photo
//#define HALL_B	PB1	// robo from front+back-photo
//#define HALL_C	PA5	// robo from front+back-photo
#define HALL_A	PB11	// rhody, swapped with B by robo to keep the 6 bldc output pins as usual
#define HALL_B	PA1		// rhody
#define HALL_C	PA0		// rhody, swapped with B by robo to keep the 6 bldc output pins as usual

// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4
#define USART1_TX		PA2
#define USART1_RX		PA3

#define VBATT	PA4		//maybe 
#define CURRENT_DC	PA6		//maybe 

#define SELF_HOLD	PB2		// rhody tested
#define BUTTON		PC14 	// rhody tested

#ifdef HAS_BUZZER
	// Buzzer defines
	#define BUZZER	PB9 // rhody tested
#endif

//#define CHARGE_STATE TODO_PIN

// photo diodes / light barriers on the backside
#define PHOTO_L		TODO_PIN
#define PHOTO_R		TODO_PIN



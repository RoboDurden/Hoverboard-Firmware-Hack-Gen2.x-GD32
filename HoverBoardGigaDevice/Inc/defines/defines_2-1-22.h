// big thanks to Vicky-lp01 for adding this file with a pull request.

#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
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
	ST-Flash pins: A13 A14
	
	so mostly available for other use:	
	C13 C14 C15 F0 F1 A0 A1 A4 A5 A6 A7 B0 B1 B2 B10 B11 B12 A11 F6 F7 A12 A15 B3 B4 B5 B8 B9 
	so available for analog input:
	A0 A1 A2 A3 A4 A5 A6 A7 B0 B1 	
*/



// LED defines //If you use this, check the colors, mine is not working on the master
#define LED_GREEN PB3
#define LED_ORANGE PB4
#define LED_RED PA15

#define UPPER_LED PB5
#define LOWER_LED PF1

// Mosfet output 
#define MOSFET_OUT PC13

// Brushless Control DC (BLDC) defines
#define TIMER_BLDC_PULLUP	GPIO_PUPD_NONE	
// Channel G 
#define BLDC_GH PA8
#define BLDC_GL PB13
// Channel B 
#define BLDC_BH PA10
#define BLDC_BL PB15
// Channel Y 
#define BLDC_YH PA9
#define BLDC_YL PB14

// Timer BLDC short circuit emergency shutoff define - Problably
#define TIMER_BLDC_EMERGENCY_SHUTDOWN PB12

// Hall sensor defines 
#define HALL_A PC14
#define HALL_C PA1
#define HALL_B PB11

#define USART0_TX	PB6
#define USART0_RX	PB7

#define USART1_TX	PA2
#define USART1_RX	PA3


// ADC defines 
#define VBATT	PA0 // Also probably
#define CURRENT_DC PA6

// Self hold defines 
#define SELF_HOLD PA4

// Button defines 
#define BUTTON PA5


#ifdef BUZZER
	#define BUZZER PF0
#endif

#define CHARGE_STATE PC15


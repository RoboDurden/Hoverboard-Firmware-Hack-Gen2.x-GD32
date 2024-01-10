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
	ST-Flash pins: A13 A14 (also used as green and red led on 2.2)
	
	so mostly available for other use:	
	C13 C14 C15 F0 F1 A0 A1 A4 A5 A6 A7 B0 B1 B2 B10 B11 B12 A11 F6 F7 A12 A15 B3 B4 B5 B8 B9 
	so available for analog input:
	A0 A1 A2 A3 A4 A5 A6 A7 B0 B1 	
*/

#define TODO_PIN PF4	// PF4 is only accessible on the largest GD32F130Rx LQFP64 pinouts mcu

// LED defines
#define LED_GREEN GPIO_PIN_15	// Batman313v, color might be wrong
#define LED_GREEN_PORT GPIOC	// Batman313v
#define LED_ORANGE GPIO_PIN_14	// Batman313v, color might be wrong
#define LED_ORANGE_PORT GPIOC	// Batman313v
#define LED_RED GPIO_PIN_13		// Batman313v, color might be wrong
#define LED_RED_PORT GPIOC		// Batman313v

#define UPPER_LED_PIN GPIO_PIN_1	// Batman313v, color might be wrong
#define UPPER_LED_PORT GPIOF		// Batman313v
#define LOWER_LED_PIN GPIO_PIN_0	// Batman313v, color might be wrong
#define LOWER_LED_PORT GPIOF		// Batman313v

// Mosfet output
// seems to be an ordinary LED output ?
// led.c:91	gpio_bit_write(MOSFET_OUT_PORT, MOSFET_OUT_PIN, counter_Blue >= setValue_Blue ? RESET : SET); 
#define MOSFET_OUT_PIN TODO_PIN		
#define MOSFET_OUT_PORT TODO_PORT

// Brushless Control DC (BLDC) defines
#define TIMER_BLDC_PULLUP	GPIO_PUPD_NONE	// robo
// Channel G
#define RCU_TIMER_BLDC RCU_TIMER0
#define TIMER_BLDC TIMER0
#define TIMER_BLDC_CHANNEL_G TIMER_CH_2
#define TIMER_BLDC_GH_PIN GPIO_PIN_10	//SAME AS 2.0 :-)
#define TIMER_BLDC_GH_PORT GPIOA		//SAME AS 2.0 :-)
#define TIMER_BLDC_GL_PIN GPIO_PIN_15	//SAME AS 2.0 :-)
#define TIMER_BLDC_GL_PORT GPIOB		//SAME AS 2.0 :-)
// Channel B
#define TIMER_BLDC_CHANNEL_B TIMER_CH_1
#define TIMER_BLDC_BH_PIN GPIO_PIN_9	//SAME AS 2.0 :-)
#define TIMER_BLDC_BH_PORT GPIOA		//SAME AS 2.0 :-)
#define TIMER_BLDC_BL_PIN GPIO_PIN_14	//SAME AS 2.0 :-)
#define TIMER_BLDC_BL_PORT GPIOB		//SAME AS 2.0 :-)
// Channel Y
#define TIMER_BLDC_CHANNEL_Y TIMER_CH_0
#define TIMER_BLDC_YH_PIN GPIO_PIN_8	//SAME AS 2.0 :-)
#define TIMER_BLDC_YH_PORT GPIOA		//SAME AS 2.0 :-)
#define TIMER_BLDC_YL_PIN GPIO_PIN_13	//SAME AS 2.0 :-)
#define TIMER_BLDC_YL_PORT GPIOB		//SAME AS 2.0 :-)

// Timer BLDC short circuit emergency shutoff define
// Is initialized here but never used somewhere else in code.
// setup.c:176	gpio_mode_set(TIMER_BLDC_EMERGENCY_SHUTDOWN_PORT , GPIO_MODE_AF, GPIO_PUPD_NONE, TIMER_BLDC_EMERGENCY_SHUTDOWN_PIN);  
#define TIMER_BLDC_EMERGENCY_SHUTDOWN_PIN TODO_PIN
#define TIMER_BLDC_EMERGENCY_SHUTDOWN_PORT TODO_PORT

// Hall sensor defines
#define HALL_A_PIN GPIO_PIN_0	// Batman313v
#define HALL_A_PORT GPIOB		// Batman313v
#define HALL_B_PIN GPIO_PIN_5	// Batman313v
#define HALL_B_PORT GPIOB		// Batman313v
#define HALL_C_PIN GPIO_PIN_4	// Batman313v
#define HALL_C_PORT GPIOB		// Batman313v

// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define HAS_USART0	// uncomment if this layout has a usart0
#ifdef HAS_USART0
	#define USART0_TX_PIN	GPIO_PIN_6
	#define USART0_TX_PORT	GPIOB
	#define USART0_RX_PIN	GPIO_PIN_7
	#define USART0_RX_PORT	GPIOB
	
	//#define USART0_MASTERSLAVE		// uncomment if this usart is used for master-slave communication
	#define USART0_REMOTE						// uncomment if this usart is used for optional remote control
#endif

// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4
#define HAS_USART1	// uncomment if this layout has a usart1
#ifdef HAS_USART1
	#define USART1_TX_PIN		GPIO_PIN_2
	#define USART1_TX_PORT	GPIOA
	#define USART1_RX_PIN		GPIO_PIN_3
	#define USART1_RX_PORT	GPIOA
	
	#define USART1_MASTERSLAVE		// uncomment if this usart is used for master-slave communication
	//#define USART1_REMOTE				// uncomment if this usart is used for optional remote control
#endif


// ADC defines
#define VBATT_PIN	GPIO_PIN_0			// Batman313v, might be CURRENT_DC !!!
#define VBATT_PORT GPIOA				// Batman313v
#define VBATT_CHANNEL ADC_CHANNEL_4
#define CURRENT_DC_PIN	GPIO_PIN_1		// Batman313v, might be VBATT !!!
#define CURRENT_DC_PORT GPIOA			// Batman313v
#define CURRENT_DC_CHANNEL ADC_CHANNEL_6

// Self hold defines
// important pin keeps the mosfet open after the on/off button got pushed !
// main.c:306: gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, SET); 
// and turns off power on Shutdown:
// main.c:513:	 gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, RESET); 
#define SELF_HOLD_PIN GPIO_PIN_6		// Batman313v
#define SELF_HOLD_PORT GPIOA			// Batman313v

// Button defines
// on/off (POW) push-button. So also a connection (i guess with some smd resistor in between) to a MCU pin.
// main.c:457: if (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN)) 
#define BUTTON_PIN GPIO_PIN_5		// Batman313v
#define BUTTON_PORT GPIOA			// Batman313v


#ifdef BUZZER
	// Buzzer defines
	#define BUZZER_PIN GPIO_PIN_8		// Batman313v
	#define BUZZER_PORT GPIOB			// Batman313v
#endif

#ifdef MASTER
	// Charge state defines
	// This seems to be a digital input that hast to be high in order to enable the motors. 
	// main.c:381: chargeStateLowActive = gpio_input_bit_get(CHARGE_STATE_PORT, CHARGE_STATE_PIN);
	// If not found it should be okay to simply comment this line because chargeStateLowActive in initialised as set = true
	#define CHARGE_STATE_PIN TODO_PIN
	#define CHARGE_STATE_PORT TODO_PORT
#endif

// Debug pin defines - seems to be never used in code.
#define DEBUG_PIN TODO_PIN
#define DEBUG_PORT TODO_PORT

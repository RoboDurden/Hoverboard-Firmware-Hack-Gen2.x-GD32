#ifdef SLAVE	// this layout has buzzer on the slave board !
	#define HAS_BUZZER
#endif


// 2.2 unused/unkown: B4 B5 A6 A2 A0

// LED defines
//#define LED_GREEN 			TODO_PIN	//GPIO_PIN_13	// lerwinDE: in conflict with flash pins DIO and CLK !!!
#define LED_ORANGE 			PF1	// UPPER_LED with 4x blue led
//#define LED_RED 				TODO_PIN	//GPIO_PIN_14	// lerwinDE: in conflict with flash pins DIO and CLK !!!

//#define UPPER_LED 	TODO_PIN	
//#define LOWER_LED 	TODO_PIN	


// Mosfet output
// seems to be an ordinary LED output ?
// led.c:91	gpio_bit_write(MOSFET_OUT_PORT, MOSFET_OUT_PIN, counter_Blue >= setValue_Blue ? RESET : SET); 
//#define MOSFET_OUT TODO_PIN		// TODO

// Brushless Control DC (BLDC) defines
#define TIMER_BLDC_PULLUP	GPIO_PUPD_PULLUP	// robo, based on Herleybob:defines.h
// Channel G
//#define RCU_TIMER_BLDC RCU_TIMER0
//#define TIMER_BLDC TIMER0
//#define TIMER_BLDC_CHANNEL_G TIMER_CH_2
#define BLDC_GH PA8		// GPIO_PIN_8	robo, based on Herleybob:defines.h
#define BLDC_GL PA7		// robo, based on Herleybob:defines.h
// Channel B
//#define TIMER_BLDC_CHANNEL_B TIMER_CH_1
#define BLDC_BH PA9		// robo, based on Herleybob:defines.h
#define BLDC_BL PB0		// robo, based on Herleybob:defines.h
// Channel Y
//#define TIMER_BLDC_CHANNEL_Y TIMER_CH_0
#define BLDC_YH PA10		// robo, based on Herleybob:defines.h
#define BLDC_YL PB1		// robo, based on Herleybob:defines.h



//#define BLDC_CUR_G_PIN PA5
//#define BLDC_CUR_B_PIN PA4


// Timer BLDC short circuit emergency shutoff define
// Is initialized here but never used somewhere else in code.
// setup.c:176	gpio_mode_set(TIMER_BLDC_EMERGENCY_SHUTDOWN_PORT , GPIO_MODE_AF, GPIO_PUPD_NONE, TIMER_BLDC_EMERGENCY_SHUTDOWN_PIN);  
//#define TIMER_BLDC_EMERGENCY_SHUTDOWN TODO_PIN	// TODO

// Hall sensor defines
#define HALL_A PB8		// robo, based on Herleybob:defines.h A = U ?
#define HALL_B PA12	// robo, based on Herleybob:defines.h B = V ?
#define HALL_C PB3		// robo, based on Herleybob:defines.h C = W ?


// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define USART0_TX	PB6
#define USART0_RX	PB7

// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4
#define USART1_TX		PA2
#define USART1_RX		PA15	// robo 2025: is this correct or should it be PA3 ?


// ADC defines
#define VBATT	PA1				// robo, no gpio_mode_set() inHerleybob:setup.c
#define ADC_BATTERY_VOLT      0.02500961912134820371101460718516 // V_Batt to V_BattMeasure = factor 30: ( (ADC-Data/4095) *3,3V *30 )

#define CURRENT_DC	PA3	// robo, no gpio_mode_set() inHerleybob:setup.c

// Self hold defines
// important pin keeps the mosfet open after the on/off button got pushed !
// main.c:306: gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, SET); 
// and turns off power on Shutdown:
// main.c:513:	 gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, RESET); 
#define SELF_HOLD PA1		// lerwinDE: master: A11 is used a hold bin, slave: A11 is buzzer pini

// Button defines
// on/off (POW) push-button. So also a connection (i guess with some smd resistor in between) to a MCU pin.
// main.c:457: if (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN)) 
#define BUTTON PB2			// robo, based on Herleybob:defines.h


#ifdef BUZZER
	// Buzzer defines
	#define BUZZER PA11		// robo, based on Herleybob:defines.h
#endif

#define CHARGE_STATE PF0		// TODO


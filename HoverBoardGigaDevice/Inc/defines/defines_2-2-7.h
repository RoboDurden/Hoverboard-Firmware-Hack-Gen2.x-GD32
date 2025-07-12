// Gen1 board (useful when the mosfets for one motor burned)
//#define MOTOR_LEFT		// motor right chosen


#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
	#define HAS_BUZZER
#endif

#define LED_RED 	PB2		// LED_PIN onboard

#define SELF_HOLD PA5	// OFF_PIN
#define BUTTON PA1	// BUTTON_PIN
#define VBATT				PA1 // or PC2 = V_BATT_MEAS
#define BUZZER	PA4


#ifdef MOTOR_LEFT
	#define CURRENT_DC	PC0
	#define TIMER_BLDC_EMERGENCY_SHUTDOWN	PB12	// left motor overcurrent

	#define HALL_A PB5
	#define HALL_B PB6
	#define HALL_C PB7

	#define PHASE_A PA0
	#define PHASE_B PC3

	#define BLDC_GH PC6
	#define BLDC_GL PA7
	#define BLDC_BH PC7
	#define BLDC_BL PB0
	#define BLDC_YH PC8
	#define BLDC_YL PB1

#else
	#define CURRENT_DC	PC1
	#define TIMER_BLDC_EMERGENCY_SHUTDOWN	PA6	// right motor overcurrent

	#define HALL_A PC10	// right motor hall sensor u
	#define HALL_B PC11	// right motor hall sensor v
	#define HALL_C PC12	// right motor hall sensor w

	#define PHASE_A PC4
	#define PHASE_B PC5


	#define BLDC_GH PA8
	#define BLDC_GL PB13
	#define BLDC_BH PA9		
	#define BLDC_BL PB14
	#define BLDC_YH PA10		
	#define BLDC_YL PB15

#endif
#define TIMER_BLDC_PULLUP	GPIO_MODE_AF_PP	// robo: not sure if some boards indeed nned GPIO_PUPD_PULLUP like 2.2 or 2.3


#define USART1_TX		PA2
#define USART1_RX		PA3

#define USART2_TX	PB10
#define USART2_RX	PB11

#define CHARGE_STATE	PA12			// motors will stop if charger is pluged in. disable if you use the charger to power the board

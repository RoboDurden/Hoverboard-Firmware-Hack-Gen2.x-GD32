// Gen2.2.3  https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/84
// a.k.a. SMART-L-V2.0 (used in Andersson 3.1 and Andersson 3.2 with 10 inch wheels)

#define STM32F103 // Configure system clock to 64 MHz (maximum supported with High Speed Internal (HSI) oscilator as clock source for STM32F103)

// Tuned for STM32F103 based board (64MHz => PWM range: [-1000 - 1000]) with 10inch wheel at low speeds (400-600 revs/s*1024)
#define PIDINIT_a3o {\
	{16,	0.015, 0.21, 0.00005, 1.0, 1.0, 0}, \
	{16,	0.2, 1.0, 0.0005,	1.0, 1.0, 0},	/* UNTESTED */ \
	{1,		4.0, 2.0, 0.1	,		0.5, 0.5, 30}	/* UNTESTED */ }

	
#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
	#define HAS_BUZZER
#endif

#define HALL_A		PA0
#define HALL_B		PA2
#define HALL_C		PA1


//#define LED_RED			PB3
//#define LED_ORANGE	PA15
//#define LED_GREEN		PA12 // JW: PB3 in defines_2-2-1.h
//#define UPPER_LED		PA11 // robo according to "Hoverboard JC2015.7.31.pdf" PA11
//#define LOWER_LED		P??
//#define ONBOARD_LED		P??
//#define CHARGE_STATE PF0 // JW: Charger port is directly connected to battery cable, so probably not possible to detect charger state.
#define BUZZER		PB1

#define SELF_HOLD		PA4
#define BUTTON		PA3
//#define BUTTON_PU		PA3


#define VBATT		PA5
#define CURRENT_DC		PA7


#define ADC_BATTERY_VOLT      0.012824507539  	//	JW: calibrated with 10s battery
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

// below the empty IMU header
#define USART0_TX	PB6
#define USART0_RX	PB7


// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4

// master-slave header
#define USART2_TX	PB10
#define USART2_RX	PB11


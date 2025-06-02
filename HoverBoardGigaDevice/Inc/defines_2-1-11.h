// robo: 2.13 = very much like 2.0 ! But USART0 only on empty header of bluetooth module,
//	different hall order and an optional third led header (with PB5 and a blinking output)

#ifdef MASTER_OR_SINGLE		// layout 2.2 and 2.7 have buzzer on the slave board.
	#define HAS_BUZZER
#endif



// LED defines
#define LED_GREEN PA15
#define LED_ORANGE PA12
#define LED_RED PB3

#define UPPER_LED	PA1
#define LOWER_LED	PA0

#define UPPER_LED2	PB5		//	robo: the middle pin of the 3pin led header between the 3led (left) and 2led (right)
													//	VBATT!!! , PB5 , ? = simply blinking,


// Mosfet output, little onboard led
#define MOSFET_OUT	PC13

// Brushless Control DC (BLDC) defines
#define BLDC_GH PA10		// green	, Tommyboi2001 all bldc pins same as 2.0
#define BLDC_GL PB15		
#define BLDC_BH PA9			// blue
#define BLDC_BL PB14		
#define BLDC_YH PA8			// yellow
#define BLDC_YL PB13		
#define TIMER_BLDC_PULLUP	GPIO_PUPD_NONE	// robo: not sure if some boards indeed nned GPIO_PUPD_PULLUP like 2.2 or 2.3


// Timer BLDC short circuit emergency shutoff define
//#define TIMER_BLDC_EMERGENCY_SHUTDOWN	TODO_PIN

// Hall sensor defines

#define HALL_A	PB11
#define HALL_B	PC14
#define HALL_C	PF1




// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define USART0_TX	PB6
#define USART0_RX	PB7

// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4
#define USART1_TX		PA2
#define USART1_RX		PA3
	

//	so available for analog input: A5 A6 A7 B0 B1 	
	


// ADC defines
#define VBATT	PA4		// not PA4, autodetect suggests PB0 :-)
#define ADC_BATTERY_VOLT      0.0258320368	// robo newly gaged

#define CURRENT_DC	PA6

#define PHASE_CURRENT_X	PA7		// robo: maybe because some oscilating values

// Self hold defines
#define SELF_HOLD	PB2

// Button defines
#define BUTTON	PC15

#ifdef HAS_BUZZER
	// Buzzer defins
	#define BUZZER	PB10
#endif

#ifdef MASTER
	// Charge state defines
	#define CHARGE_STATE	PF0
#endif


// 2.3.3

// LED defines
//#define LED_GREEN PB3				
//#define LED_ORANGE PB4				
//#define LED_RED PB5
//#define UPPER_LED PF0
//#define LOWER_LED PF1		


//#define CURRENT_DC	PA6
//#define SELF_HOLD PB2
//#define BUTTON PA5
#define BUZZER PF0
//#define CHARGE_STATE PC15
//#define MOSFET_OUT TODO_PIN		

// Hall sensor defines
#define HALL_A PB10
#define HALL_B PB3
#define HALL_C PA15


// Brushless Control DC (BLDC) defines
#define TIMER_BLDC_PULLUP	GPIO_PUPD_NONE	// robo
// Channel G
#define BLDC_GH PB15		// PB15 - Hi Lo Swapped from 2.0
#define BLDC_GL PA10		// PA10 - Hi Lo Swapped from 2.0
// Channel B
#define BLDC_BH PB14		// PB14 - Hi Lo Swapped from 2.0
#define BLDC_BL PA9		// PA9 - Hi Lo Swapped from 2.0
// Channel Y
#define BLDC_YH PB13		// PB13 - Hi Lo Swapped from 2.0
#define BLDC_YL PA8		// PA8 - Hi Lo Swapped from 2.0

// Timer BLDC short circuit emergency shutoff define
// Is initialized here but never used somewhere else in code.
// setup.c:176	gpio_mode_set(TIMER_BLDC_EMERGENCY_SHUTDOWN_PORT , GPIO_MODE_AF, GPIO_PUPD_NONE, TIMER_BLDC_EMERGENCY_SHUTDOWN);  
//#define TIMER_BLDC_EMERGENCY_SHUTDOWN TODO_PIN //GPIO_PIN_12	// NC



#define USART0_TX	PB6	// vertical 2.54mm header to the left on slave with 100 Ohm: second lowest pin (gnd = third lowest pin)
#define USART0_RX	PB7	// vertical 2.54mm header to the left on slave with 100 Ohm: lowest pin 

#define USART1_TX		PA2		// second left pin on uart header
#define USART1_RX		PA3		// thir left pin on uart header

#define I2C0_SCL PB8		// IMU
#define I2C0_SDA PB9		// IMU


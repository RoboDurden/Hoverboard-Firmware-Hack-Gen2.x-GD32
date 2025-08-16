#ifndef DEFINES_2_1_20_H
#define DEFINES_2_1_20_H

#define LED_GREEN PB3
#define LED_ORANGE PA15
#define LED_RED PB4
#define UPPER_LED PB2
#define LOWER_LED PB5

#define HALL_A PC13
#define HALL_B PA1
#define HALL_C PC14

#define BUZZER PB9

#define VBATT PA4
#define ADC_BATTERY_VOLT      0.02507  	// V_Batt to V_BattMeasure = factor 30: ( (ADC-Data/4095) *3,3V *30 )

//#define CURRENT_DC P??		// this board does not have a shunt resistor !

#define SELF_HOLD	PB12
#define BUTTON PA12

// Brushless Control DC (BLDC) defines
#define BLDC_GH PA10		// green	, Tommyboi2001 all bldc pins same as 2.0
#define BLDC_GL PB15		
#define BLDC_BH PA9			// blue
#define BLDC_BL PB14		
#define BLDC_YH PA8			// yellow
#define BLDC_YL PB13		
#define TIMER_BLDC_PULLUP	GPIO_PUPD_NONE	// robo: not sure if some boards indeed nned GPIO_PUPD_PULLUP like 2.2 or 2.3

// GD32F130 USART0 TX/RX:	(PA9/PA10)AF1	, (PB6/PB7)AF0 , 	(PA2/PA3)AF1 , (PA14/PA15)AF1 GD32F130x4 only!
#define USART0_TX	PB6	// you have to solder to the tiny IMU: https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/90#issuecomment-2949589576
#define USART0_RX	PB7	// you have to solder to the tiny IMU: https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/90#issuecomment-2949589576

// GD32F130 USART1 GD32F130 TX/RX: (PA14/PA15)AF1 , (PA2,PA3)AF1	, (PA8/PB0)AlternateFunction4
#define USART1_TX		PA2
#define USART1_RX		PA3

#define I2C_PB6PB7	// IMU


#endif
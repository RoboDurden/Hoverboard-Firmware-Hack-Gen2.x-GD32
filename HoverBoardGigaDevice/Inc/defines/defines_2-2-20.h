#ifndef DEFINES_2_2_20_H
#define DEFINES_2_2_20_H

#define LED_GREEN PB3
#define LED_ORANGE PA15
#define LED_RED PB4
#define UPPER_LED PB2
#define LOWER_LED PB5

#define PHOTO_L PC15
#define PHOTO_R PA11


#define HALL_A PC13
#define HALL_B PA1
#define HALL_C PC14

#define BUZZER PB9

#define VBATT PA4
#define ADC_BATTERY_VOLT      0.02507  	// V_Batt to V_BattMeasure = factor 30: ( (ADC-Data/4095) *3,3V *30 )

//#define CURRENT_DC P??		// no DC bus shunt on this board
// Per-phase current sensing: 2x R004 (4mΩ) shunt resistors on low-side FETs,
// amplified by dual op-amp (~20x gain). No shunt on the third phase.
#define PHASE_A	PB0		// 4mΩ shunt + op-amp on phase A low-side
#define PHASE_B	PA0		// 4mΩ shunt + op-amp on phase B low-side
//#define PHASE_C	P??		// no shunt on phase C

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

// GD32F103 USART0 default pins: PA9/PA10. PB6/PB7 available via remap (see setup.c).
#define USART0_TX	PB6	// using AFIO_PCF0 USART0_REMAP -> PB6/PB7
#define USART0_RX	PB7

// GD32F103 USART1 pins: PA2/PA3 (master-slave 4-pin header on this board).
#define USART1_TX		PA2
#define USART1_RX		PA3

//#define MPU_6050		// Gemini tried for clones..
//#define MPU_6050old	// disabled on F103 — i2c.c lacks a PB6/PB7 path for TARGET=2
//#define I2C_PB6PB7	// IMU



#endif
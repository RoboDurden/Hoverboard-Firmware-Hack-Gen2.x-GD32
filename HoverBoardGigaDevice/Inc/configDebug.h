#ifndef CONFIG_H
#define CONFIG_H

//#define REMOTE_AUTODETECT
				// ONLY test with 1-2A constant current power supply !!!! The charger with 1.5A might also do :-)
				// will drive the motor without hall input to detect the hall pins..

#ifdef REMOTE_AUTODETECT
	#define REMOTE_USART				0 	// 	1 is usually PA2/PA3 and the original master-slave 4pin header
																	//	0 is usually PB6/PB7 and the empty header close to the flash-header
																	//	2 is usually PB10/PB11 on stm32f103 boards
#else

	// choose your target in the dropdown list to the top-right of the Keil IDE
	// and then set your layout below
	// Gen2-target-layout is included in defines.h
	#ifdef GD32F130		// TARGET = 1
		#define LAYOUT 4
		#define LAYOUT_SUB 1	// Layout 2.1.7 exisits as 2.1.7.0 and 2.1.7.1
	#elif GD32F103		// TARGET = 2
		#define LAYOUT 1
	#elif GD32E230		// TARGET = 3
		#define LAYOUT 1
	#elif MM32SPIN05	// TARGET = 4
		#define LAYOUT 1
	#endif
	
	//#define BLDC_BC			// old block commutation bldc control
	#define BLDC_SINE			// silent sine-pwm motor control, added 2025 by Robo Durden. 
													// not yet for target 2  = Gen2.2.x
	
	#define BAT_CELLS         	7        // battery number of cells. Normal Hoverboard battery: 10s
	//#define BATTERY_LOW_SHUTOFF		// will shut off the board below BAT_LOW_DEAD = BAT_CELLS * CELL_LOW_DEAD, 

	//#define MASTER		// uncomment for MASTER firmware.
	//#define SLAVE			// uncomment for SLAVE firmware.
	#define SINGLE			// uncomment if firmware is for single board and no master-slave dual board setup

	#if defined(MASTER) || defined(SINGLE)
		
		// choose only one 'remote' to control the motor
		//#define REMOTE_DUMMY
		#define REMOTE_UART
		//#define REMOTE_UARTBUS	// ESP32 as master and multiple boards as multiple slaves ESP.tx-Hovers.rx and ESP.rx-Hovers.tx
		//#define REMOTE_CRSF		// https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/26
		//#define REMOTE_ADC	// speed is PA2=TX and steer is PA3=RX of the masterslave header. Get 3.3V from the flash header
												// DO NOT use the 5V/15V pin of the masterslave header for the potentiometers !!!!!!!!!
												// SLAVE board has to be connected to the additional UART header, but 5V/15V coming from the masters masterslave header
												// for calibration, hold the on/off button until the startup melody restarts.
												// Then release the button and leave the joystick (the two potentiometers) in neutral position.
												// When the melody returns for 2 seconds, push speed to max.
												// After another 5 seconds + 2 seconds melody: push speed to min. Then steer to max. Finally steer to min
		
		
		#ifdef REMOTE_UARTBUS
			#define SLAVE_ID	0		// must be unique for all hoverboards connected to the bus
		#endif
		#ifdef REMOTE_DUMMY
			#define TEST_HALL2LED	// led the 3-led panel blink according to the hall sensors
		#else
			//#define TEST_HALL2LED	// led the 3-led panel blink according to the hall sensors
		#endif
		

		#define SPEED_COEFFICIENT   -1
		#define STEER_COEFFICIENT   1
		
		#define DISABLE_BUTTON	// this is the opposite of former CHECK_BUTTON define.
															// remove '//' if you use a slave board as master 
															// or if you turn the boards on/off by injecting a postive voltage into the input pin of the 2pin BUTTON header

		#define MASTER_OR_SINGLE
	#endif
	
	#if defined(MASTER) || defined(SLAVE)
		#define MASTERSLAVE_USART		2 	// 	1 is usually PA2/PA3 and the original master-slave 4pin header
																		//	0 is usually PB6/PB7 and the empty header close to the flash-header
																		//	2 is usually PB10/PB11 on stm32f103 boards
	#endif
	
	#if defined(REMOTE_UART) || defined(REMOTE_UARTBUS) || defined(REMOTE_CRSF)
		#define REMOTE_USART				0 	// 	1 is usually PA2/PA3 and the original master-slave 4pin header
																		//	0 is usually PB6/PB7 and the empty header close to the flash-header
																		//	2 is usually PB10/PB11 on stm32f103 boards
	#endif
	
	//#define DEBUG_LED		// uncomment to activate DEBUG_LedSet(bSet,iColor) macro. iCol: 0=green, 1=organge, 2=red
	
#endif



#ifdef MASTER_OR_SINGLE
	#define INACTIVITY_TIMEOUT 	8        	// Minutes of not driving until poweroff (not very precise)

	#define CELL_LOW_LVL1     3.5       // Gently beeps, show green battery symbol above this Level.
	#define CELL_LOW_LVL2     3.3       // Battery almost empty, show orange battery symbol above this Level. Charge now! 
	#define CELL_LOW_DEAD     3.0       // Undervoltage lockout, show red battery symbol above this Level.
#endif



#define DC_CUR_LIMIT     		15        // Motor DC current limit in amps
#define DEAD_TIME        		60        // PWM deadtime (60 = 1?s, measured by oscilloscope)
#define PWM_FREQ         		16000     // PWM frequency in Hz


#define FILTER_SHIFT 13 						// Low-pass filter for pwm, rank k=12
					// With PWM_FREQ = 1000, 12 will take over 4s to mostly adapt to a sudden change in input. So only 250 ms for 16 kHz !
					// 19 and 16 kHz would be 32 seconds for the motor to reach 63% of its new target speed (Gemini 2.5pro)


#define DELAY_IN_MAIN_LOOP 	5         // Delay in ms

#define TIMEOUT_MS          2000      // Time in milliseconds without steering commands before pwm emergency off

#endif		// CONFIG_H
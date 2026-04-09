#ifndef CONFIG_H
#define CONFIG_H

//#define REMOTE_AUTODETECT		// !! highly experimental 
				// ONLY test with 1-2A constant current power supply !!!!
				// will drive the motor without hall input to detect the hall pins..

#ifdef REMOTE_AUTODETECT
	
	#define HAS_USART0	// tx=PB6,rx=PB7	uncomment to connect via 19200 baud serial
	//#define HAS_USART1	// tx=PA2,rx=PA3	uncomment to connect via 19200 baud serial

	#define SINGLE
	#define MASTER_OR_SINGLE
	#define BAT_CELLS         	6        // battery number of cells. Normal Hoverboard battery: 10s
	#define SPEED_COEFFICIENT   -1
	#define STEER_COEFFICIENT   1
	//#define CHECK_BUTTON		// disable = add '//' if you use a slave board as master
#else

	// LAYOUT_2_X is used in defines.h
	#ifdef GD32F130		// TARGET = 1
		#define LAYOUT 20
		#define LAYOUT_SUB 1	// Layout 2.1.7 exisits as 2.1.7.0 and 2.1.7.1
	#elif GD32F103		// TARGET = 2
		#define LAYOUT 1
	#elif GD32E230		// TARGET = 3
		#define LAYOUT 1
	#elif MM32SPIN05	// TARGET = 4
		#define LAYOUT 1
	#endif

	//#define MASTER		// uncomment for MASTER firmware. Choose USART0_MASTERSLAVE or USART1_MASTERSLAVE in your defines_2-?.h file
	//#define SLAVE			// uncomment for SLAVE firmware. Choose USART0_MASTERSLAVE or USART1_MASTERSLAVE in your defines_2-?.h file
	#define SINGLE			// uncomment if firmware is for single board and no master-slave dual board setup

	#define BLDC_BC			// old block commutation bldc control
	//#define BLDC_SINE		// silent sine-pwm motor control
	#define FOC_ENABLED	// block commutation at startup, transitions to FOC at speed

	#define DRIVING_MODE 0	//  0=pwm, 1=speed, 2=torque, 3=odometer

	#define BAT_CELLS         	7       // battery number of cells. Normal Hoverboard battery: 10s
	//#define BATTERY_LOW_SHUTOFF
	#define BATTERY_LOW_BEEP
	//#define BEEP_BACKWARDS

	#define REMOTE_USART	1	// 0=PB6/PB7, 1=PA2/PA3

	#if defined(MASTER) || defined(SINGLE)
		#define MASTER_OR_SINGLE

		//#define REMOTE_DUMMY
		#define REMOTE_UART
		//#define REMOTE_UARTBUS
		//#define REMOTE_CRSF

		#ifdef REMOTE_UARTBUS
			#define SLAVE_ID	0
		#endif

		#ifdef REMOTE_DUMMY
			#define REMOTE_PERIOD 6
			#define TEST_HALL2LED
		#endif

		// RTT logging (uses ST-LINK SWD, independent of UART)
		#define RTT_REMOTE
		#define WINDOWS_RN

		#define SPEED_COEFFICIENT   -1
		#define STEER_COEFFICIENT   1

		//#define CHECK_BUTTON
		#define DISABLE_BUTTON	// startup waits for button release, so permanent jumper hangs firmware
	#endif
#endif

// ################################################################################

#define DC_CUR_LIMIT     		15        // Motor DC current limit in amps
#define DEAD_TIME        		60        // PWM deadtime (60 = 1�s, measured by oscilloscope)
#define PWM_FREQ         		16000     // PWM frequency in Hz

// ################################################################################

#define DELAY_IN_MAIN_LOOP 	5         // Delay in ms

#define TIMEOUT_MS          2000      // Time in milliseconds without steering commands before pwm emergency off

#ifdef MASTER_OR_SINGLE
	#define INACTIVITY_TIMEOUT 	8        	// Minutes of not driving until poweroff (not very precise)

	// ################################################################################


	#define CELL_LOW_LVL1     3.5       // Gently beeps, show green battery symbol above this Level.
	#define CELL_LOW_LVL2     3.3       // Battery almost empty, show orange battery symbol above this Level. Charge now! 
	#define CELL_LOW_DEAD     3.1       // Undervoltage lockout, show red battery symbol above this Level.

	#define BAT_LOW_LVL1     BAT_CELLS * CELL_LOW_LVL1
	#define BAT_LOW_LVL2     BAT_CELLS * CELL_LOW_LVL2
	#define BAT_LOW_DEAD     BAT_CELLS * CELL_LOW_DEAD

	/*
	#define BAT_LOW_LVL1     35.0       // Gently beeps, show green battery symbol above this Level.
	#define BAT_LOW_LVL2     33.0       // Battery almost empty, show orange battery symbol above this Level. Charge now! 
	#define BAT_LOW_DEAD     31.0       // Undervoltage lockout, show red battery symbol above this Level.
	// ONLY DEBUG-LEVEL!!!
	//#define BAT_LOW_LVL1     29.0
	//#define BAT_LOW_LVL2     28.0
	//#define BAT_LOW_DEAD     27.0
	*/

	// ################################################################################
#endif

#if defined(MASTER) || defined(SLAVE)
	#define MASTER_OR_SLAVE
#endif

// ###### ARMCHAIR ######
#define FILTER_SHIFT 12 						// Low-pass filter for pwm, rank k=12

#endif

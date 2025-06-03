/*
* This file is part of the hoverboard-firmware-hack-V2 project. The 
* firmware is used to hack the generation 2 board of the hoverboard.
* These new hoverboards have no mainboard anymore. They consist of 
* two Sensorboards which have their own BLDC-Bridge per Motor and an
* ARM Cortex-M3 processor GD32F130C8.
*
* Copyright (C) 2018 Florian Staeblein
* Copyright (C) 2018 Jakob Broemauer
* Copyright (C) 2018 Kai Liebich
* Copyright (C) 2018 Christoph Lehnert
*
* The program is based on the hoverboard project by Niklas Fauth. The 
* structure was tried to be as similar as possible, so that everyone 
* could find a better way through the code.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "../Inc/defines.h"
#include "../Inc/it.h"

#ifndef pinMode
void pinMode(uint32_t pin, uint32_t mode)
{
	gpio_mode_set(pin&0xffffff00U, mode, GPIO_PUPD_NONE,BIT(pin&0xfU) );
	gpio_output_options_set(pin&0xffffff00U, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, BIT(pin&0xfU));
}

void pinModePull(uint32_t pin, uint32_t mode, uint32_t pull)
{
	gpio_mode_set(pin&0xffffff00U, mode, pull,BIT(pin&0xfU) );
	gpio_output_options_set(pin&0xffffff00U, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, BIT(pin&0xfU));
}
#endif


#define TIMEOUT_FREQ  1000

// timeout timer parameter structs
timer_parameter_struct timeoutTimer_paramter_struct;

// PWM timer Parameter structs
timer_parameter_struct timerBldc_paramter_struct;	
timer_break_parameter_struct timerBldc_break_parameter_struct;
timer_oc_parameter_struct timerBldc_oc_parameter_struct;

// DMA (USART) structs
dma_parameter_struct dma_init_struct_usart;

//uint8_t usartMasterSlave_rx_buf[USART_MASTERSLAVE_RX_BUFFERSIZE];
//uint8_t usartSteer_COM_rx_buf[USART_STEER_COM_RX_BUFFERSIZE];

uint8_t usart0_rx_buf[1];
uint8_t usart1_rx_buf[1];


// DMA (ADC) structs
dma_parameter_struct dma_init_struct_adc;
extern adc_buf_t adc_buffer;

//----------------------------------------------------------------------------
// Initializes the interrupts
//----------------------------------------------------------------------------
void Interrupt_init(void)
{
  // Set IRQ priority configuration
	TARGET_nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
}

//----------------------------------------------------------------------------
// Initializes the watchdog
//----------------------------------------------------------------------------
ErrStatus Watchdog_init(void)
{
	// Check if the system has resumed from FWDGT reset
	if (RESET != rcu_flag_get(RCU_FLAG_FWDGTRST))
	{   
		// FWDGTRST flag set
		rcu_all_reset_flag_clear();
	}
	
	// Clock source is IRC40K (40 kHz)
	// Prescaler is 16
	// Reload value is 4096 (0x0FFF)
	// Watchdog fires after 1638.4 ms
	if (fwdgt_config(0x0FFF, FWDGT_PSC_DIV16) != SUCCESS ||
		fwdgt_window_value_config(0x0FFF) != SUCCESS)
	{
		return ERROR;
	}

	// Enable free watchdog timer
	fwdgt_enable();
	
	return SUCCESS;
}

//----------------------------------------------------------------------------
// Initializes the timeout timer
//----------------------------------------------------------------------------
void TimeoutTimer_init(void)
{
	// Enable timer clock
	rcu_periph_clock_enable(RCU_TIMER_TIMEOUT);
	
	// Initial deinitialize of the timer
	
	timer_deinit(TIMER_TIMEOUT);
	
	// Set up the basic parameter struct for the timer
	// Update event will be fired every 1ms
	timeoutTimer_paramter_struct.counterdirection 	= TIMER_COUNTER_UP;
	timeoutTimer_paramter_struct.prescaler 					= 0;
	timeoutTimer_paramter_struct.alignedmode 				= TIMER_COUNTER_CENTER_DOWN;
	timeoutTimer_paramter_struct.period							= 72000000 / 2 / TIMEOUT_FREQ;
	timeoutTimer_paramter_struct.clockdivision 			= TIMER_CKDIV_DIV1;
	timeoutTimer_paramter_struct.repetitioncounter 	= 0;
	timer_auto_reload_shadow_disable(TIMER_TIMEOUT);
	timer_init(TIMER_TIMEOUT, &timeoutTimer_paramter_struct);
	
	// Enable TIMER_INT_UP interrupt and set priority
	TARGET_nvic_irq_enable(TIMER_TIMEOUT_IRQn, 0, 0);
	timer_interrupt_enable(TIMER_TIMEOUT, TIMER_INT_UP);
	
	// Enable timer
	timer_enable(TIMER_TIMEOUT);
}

//----------------------------------------------------------------------------
// Initializes the GPIOs
//----------------------------------------------------------------------------
void GPIO_init(void)
{
	// Enable all GPIO clocks
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOF);
	
	#ifdef TIMER_BLDC_EMERGENCY_SHUTDOWN
		// Init emergency shutdown pin
		pinModeAF(TIMER_BLDC_EMERGENCY_SHUTDOWN,AF_TIMER0_BRKIN,GPIO_PUPD_NONE,GPIO_OSPEED_50MHZ)
	#endif
	
	// Init PWM output Pins
	// Configure: Alternate functions,  [Floating mode] / Pull-up / Pull-down
	// Configure: Push-Pull mode, Output max speed 2MHz
	pinModeAF(BLDC_GH, AF_TIMER0_BLDC, TIMER_BLDC_PULLUP, GPIO_OSPEED_2MHZ);
	pinModeAF(BLDC_GL, AF_TIMER0_BLDC, TIMER_BLDC_PULLUP, GPIO_OSPEED_2MHZ);
	pinModeAF(BLDC_BH, AF_TIMER0_BLDC, TIMER_BLDC_PULLUP, GPIO_OSPEED_2MHZ);
	pinModeAF(BLDC_BL, AF_TIMER0_BLDC, TIMER_BLDC_PULLUP, GPIO_OSPEED_2MHZ);
	pinModeAF(BLDC_YH, AF_TIMER0_BLDC, TIMER_BLDC_PULLUP, GPIO_OSPEED_2MHZ);
	pinModeAF(BLDC_YL, AF_TIMER0_BLDC, TIMER_BLDC_PULLUP, GPIO_OSPEED_2MHZ);


	
	#ifndef REMOTE_AUTODETECT
	
	
		#ifdef DEBUG_LED_PIN
			gpio_mode_set(DEBUG_LED_PORT , GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,DEBUG_LED_PIN);	
			gpio_output_options_set(DEBUG_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, DEBUG_LED_PIN);
		#endif


		#ifdef LED_GREEN
			pinMode(LED_GREEN,	GPIO_MODE_OUTPUT);
		#endif
		#ifdef LED_RED
			pinMode(LED_RED,		GPIO_MODE_OUTPUT);
		#endif
		#ifdef LED_ORANGE
			pinMode(LED_ORANGE,	GPIO_MODE_OUTPUT);
		#endif
		#ifdef UPPER_LED
			pinMode(UPPER_LED,	GPIO_MODE_OUTPUT);
		#endif
		#ifdef LOWER_LED
			pinMode(LOWER_LED,	GPIO_MODE_OUTPUT);
		#endif
		#ifdef MOSFET_OUT
			pinMode(MOSFET_OUT,	GPIO_MODE_OUTPUT);
		#endif


		#ifdef DEBUG_LED_PIN
			gpio_mode_set(DEBUG_LED_PORT , GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,DEBUG_LED_PIN);	
			gpio_output_options_set(DEBUG_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, DEBUG_LED_PIN);
		#endif
	
	
		// Init HAL input
		pinMode(HALL_A,	GPIO_MODE_INPUT);
		pinMode(HALL_B,	GPIO_MODE_INPUT);
		pinMode(HALL_C,	GPIO_MODE_INPUT);
	
		// Init ADC pins
		#ifdef VBATT
			pinMode(VBATT, GPIO_MODE_ANALOG);
		#endif
		#ifdef CURRENT_DC
			pinMode(CURRENT_DC, GPIO_MODE_ANALOG);
		#endif
		#ifdef REMOTE_ADC
			pinMode(PA2, GPIO_MODE_ANALOG);
			pinMode(PA3, GPIO_MODE_ANALOG);
		#endif


		// Init self hold
		#ifdef SELF_HOLD
			pinMode(SELF_HOLD,	GPIO_MODE_OUTPUT);
		#endif

		#ifdef BUZZER
			// Init buzzer
			pinModeSpeed(BUZZER,	GPIO_MODE_OUTPUT,GPIO_OSPEED_50MHZ);
			//gpio_mode_set(BUZZER_PORT , GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BUZZER_PIN);	
			//gpio_output_options_set(BUZZER_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BUZZER_PIN);
		#endif

		#ifdef MASTER_OR_SINGLE
		
			// Init button
			#ifdef BUTTON_PU
				pinModePull(BUTTON_PU,GPIO_MODE_INPUT,GPIO_PUPD_PULLUP);
			#elif defined(BUTTON)
				pinMode(BUTTON,	GPIO_MODE_INPUT);
			#endif
			
			#ifdef CHARGE_STATE_PIN
				// Init charge state
				pinModePull(CHARGE_STATE,GPIO_MODE_INPUT, GPIO_PUPD_PULLUP);
				//gpio_mode_set(CHARGE_STATE_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, CHARGE_STATE_PIN);
			#endif
		#endif
	#endif // 	#ifndef REMOTE_AUTODETECT

}
	
//----------------------------------------------------------------------------
// Initializes the PWM
//----------------------------------------------------------------------------
void PWM_init(void)
{
	// Enable timer clock
	rcu_periph_clock_enable(RCU_TIMER_BLDC);
	
	// Initial deinitialize of the timer
	timer_deinit(TIMER_BLDC);
	
	// Set up the basic parameter struct for the timer
	timerBldc_paramter_struct.counterdirection = TIMER_COUNTER_UP;
	timerBldc_paramter_struct.prescaler = 0;
	timerBldc_paramter_struct.alignedmode = TIMER_COUNTER_CENTER_DOWN;
	timerBldc_paramter_struct.period = BLDC_TIMER_PERIOD;
	timerBldc_paramter_struct.clockdivision = TIMER_CKDIV_DIV1;

	timerBldc_paramter_struct.repetitioncounter = 0;
	timer_auto_reload_shadow_disable(TIMER_BLDC);
	
	// Initialize timer with basic parameter struct
	timer_init(TIMER_BLDC, &timerBldc_paramter_struct);

	// Deactivate output channel fastmode
	timer_channel_output_fast_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, TIMER_OC_FAST_DISABLE);
	timer_channel_output_fast_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, TIMER_OC_FAST_DISABLE);
	timer_channel_output_fast_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, TIMER_OC_FAST_DISABLE);
	
	// Deactivate output channel shadow function
	timer_channel_output_shadow_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, TIMER_OC_SHADOW_DISABLE);
	timer_channel_output_shadow_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, TIMER_OC_SHADOW_DISABLE);
	timer_channel_output_shadow_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, TIMER_OC_SHADOW_DISABLE);
	
	// Set output channel PWM type to PWM1
	/*
	CH0COMCTL[2:0]
	110: PWM mode0.
	When counting up, OxCPRE is high when the counter is smaller than TIMER0_CHxCV, and low otherwise.
	When counting down, OxCPRE is low when the counter is larger than TIMER0_CHxCV, and high otherwise.
	111: PWM mode1.
	When counting up, OxCPRE is low when the counter is smaller than TIMER0_CHxCV, and high otherwise.
	When counting down, OxCPRE is high when the counter is larger than TIMER0_CHxCV, and low otherwise.
	*/
	timer_channel_output_mode_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, TIMER_OC_MODE_PWM1);
	timer_channel_output_mode_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, TIMER_OC_MODE_PWM1);
	timer_channel_output_mode_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, TIMER_OC_MODE_PWM1);

	// Initialize pulse length with value 0 (pulse duty factor = zero)
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, 0);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, 0);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, 0);
	
	// Set up the output channel parameter struct
	timerBldc_oc_parameter_struct.ocpolarity 		= TIMER_OC_POLARITY_HIGH; //HIGH: CHx_O is the same as OxCPRE , LOW: CHx_O is contrary to OxCPRE
	timerBldc_oc_parameter_struct.ocnpolarity 	= TIMER_OCN_POLARITY_LOW; //HIGH: CHx_ON is contrary to OxCPRE, LOW: CHx_O is the same as OxCPRE
	timerBldc_oc_parameter_struct.ocidlestate 	= TIMER_OC_IDLE_STATE_LOW;
	timerBldc_oc_parameter_struct.ocnidlestate 	= TIMER_OCN_IDLE_STATE_HIGH;
	
	// Configure all three output channels with the output channel parameter struct
	timer_channel_output_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, &timerBldc_oc_parameter_struct);
  timer_channel_output_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, &timerBldc_oc_parameter_struct);
	timer_channel_output_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, &timerBldc_oc_parameter_struct);

	// Set up the break parameter struct
	timerBldc_break_parameter_struct.runoffstate			= TIMER_ROS_STATE_ENABLE;
	timerBldc_break_parameter_struct.ideloffstate 		= TIMER_IOS_STATE_DISABLE;
	timerBldc_break_parameter_struct.protectmode			= TIMER_CCHP_PROT_OFF;
	timerBldc_break_parameter_struct.deadtime 				= DEAD_TIME;
	timerBldc_break_parameter_struct.breakstate				= TIMER_BREAK_DISABLE;		// Gen2.2 HarleyBob used TIMER_BREAK_DISABLE instead of TIMER_BREAK_ENABLE
	timerBldc_break_parameter_struct.breakpolarity		= TIMER_BREAK_POLARITY_LOW;
	timerBldc_break_parameter_struct.outputautostate 	= TIMER_OUTAUTO_ENABLE;
	
	// Configure the timer with the break parameter struct
	timer_break_config(TIMER_BLDC, &timerBldc_break_parameter_struct);

	// Disable until all channels are set for PWM output
	timer_disable(TIMER_BLDC);

	// Enable all three channels for PWM output
	timer_channel_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, TIMER_CCX_ENABLE);
	timer_channel_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, TIMER_CCX_ENABLE);
	timer_channel_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, TIMER_CCX_ENABLE);

	// Enable all three complemenary channels for PWM output
	timer_channel_complementary_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, TIMER_CCXN_ENABLE);
	timer_channel_complementary_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, TIMER_CCXN_ENABLE);
	timer_channel_complementary_output_state_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, TIMER_CCXN_ENABLE);
	
	// Enable TIMER_INT_UP interrupt and set priority
	TARGET_nvic_irq_enable(TIMER0_BRK_UP_TRG_COM_IRQn, 0, 0);
	timer_interrupt_enable(TIMER_BLDC, TIMER_INT_UP);
	
	// Enable the timer and start PWM
	timer_enable(TIMER_BLDC);
}

//----------------------------------------------------------------------------
// Initializes the ADC
//----------------------------------------------------------------------------
void ADC_init(void)
{
	// Enable ADC and DMA clock
	rcu_periph_clock_enable(RCU_ADC);
	rcu_periph_clock_enable(RCU_DMA);
	
  // Configure ADC clock (APB2 clock is DIV1 -> 72MHz, ADC clock is DIV6 -> 12MHz)
	rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
	
	// Interrupt channel 0 enable
	TARGET_nvic_irq_enable(DMA_Channel0_IRQn, 1, 0);
	
	// Initialize DMA channel 0 for ADC
	dma_deinit(DMA_CH0);
	
	uint16_t iCountAdc = sizeof(adc_buffer)/2;	// array of uint16_t
	//iCountAdc = 4;
	
	dma_init_struct_adc.direction = DMA_PERIPHERAL_TO_MEMORY;
	dma_init_struct_adc.memory_addr = (uint32_t)&adc_buffer;
	dma_init_struct_adc.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct_adc.memory_width = DMA_MEMORY_WIDTH_16BIT;
	dma_init_struct_adc.number = iCountAdc;
	
	dma_init_struct_adc.periph_addr = (uint32_t)&TARGET_ADC_RDATA;
	dma_init_struct_adc.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_init_struct_adc.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
	dma_init_struct_adc.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA_CH0, &dma_init_struct_adc);
	
	// Configure DMA mode
	dma_circulation_enable(DMA_CH0);
	dma_memory_to_memory_disable(DMA_CH0);
	
	// Enable DMA transfer complete interrupt
	dma_interrupt_enable(DMA_CH0, DMA_CHXCTL_FTFIE);
	
	// At least clear number of remaining data to be transferred by the DMA 
	dma_transfer_number_config(DMA_CH0, iCountAdc);		// 2
	
	// Enable DMA channel 0
	dma_channel_enable(DMA_CH0);
	
	
	#ifdef REMOTE_AUTODETECT
		adc_channel_length_config(ADC_REGULAR_CHANNEL, 1);
		adc_regular_channel_config(0, PIN_TO_CHANNEL(TODO_PIN), ADC_SAMPLETIME_13POINT5);
			// for some reason, the adc channel 1 used for VBat (3.3V) has to be set to TODO_PIN = PF4
	#else
		adc_channel_length_config(ADC_REGULAR_CHANNEL, iCountAdc);	// 2
		#ifdef VBATT
			adc_regular_channel_config(0, PIN_TO_CHANNEL(VBATT), ADC_SAMPLETIME_13POINT5);
		#endif
		#ifdef CURRENT_DC
			adc_regular_channel_config(1, PIN_TO_CHANNEL(CURRENT_DC), ADC_SAMPLETIME_13POINT5);
		#endif
		#ifdef REMOTE_ADC
			adc_regular_channel_config(2, PIN_TO_CHANNEL(PA2), ADC_SAMPLETIME_13POINT5);
			adc_regular_channel_config(3, PIN_TO_CHANNEL(PA3), ADC_SAMPLETIME_13POINT5);
		#endif
	#endif
	
	adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
	
	// Set trigger of ADC
	adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
	adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);

	// Disable the temperature sensor, Vrefint and vbat channel
	adc_tempsensor_vrefint_disable();
	#ifndef REMOTE_AUTODETECT
		TARGET_adc_vbat_disable();
	#endif
	
	// ADC analog watchdog disable
	adc_watchdog_disable();
	
	// Enable ADC (must be before calibration)
	adc_enable();
	
	// Calibrate ADC values
	adc_calibration_enable();
	
	// Enable DMA request
	adc_dma_mode_enable();
    
	// Set ADC to scan mode
	adc_special_function_config(ADC_SCAN_MODE, ENABLE);
}


void USART1_Init(uint32_t iBaud)
{
#ifdef HAS_USART1
	
	// Init USART1
	#if REMOTE_USART==1 && defined(REMOTE_UARTBUS)	// no pullup resistors with multiple boards on the UartBus - Esp32/Arduino (Serial.begin) have to setup pullups
		#define USART1_PUPD	GPIO_PUPD_NONE
	#else
		#define USART1_PUPD	GPIO_PUPD_PULLUP
	#endif
	pinModeAF(USART1_TX, AF_USART1_TX, USART1_PUPD, GPIO_OSPEED_50MHZ);	// // GD32F130: GPIO_AF_1 = USART
	pinModeAF(USART1_RX, AF_USART1_RX, USART1_PUPD, GPIO_OSPEED_50MHZ);	
	//gpio_mode_set(USART1_TX_PORT , GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART1_TX_PIN);	
	//gpio_mode_set(USART1_RX_PORT , GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART1_RX_PIN);
	//gpio_output_options_set(USART1_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, USART1_TX_PIN);
	//gpio_output_options_set(USART1_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, USART1_RX_PIN);	
	//gpio_af_set(USART1_TX_PORT, GPIO_AF_1, USART1_TX_PIN);	// GD32F130: GPIO_AF_1 = USART
	//gpio_af_set(USART1_RX_PORT, GPIO_AF_1, USART1_RX_PIN);
	
	
	// Enable ADC and DMA clock
	rcu_periph_clock_enable(RCU_USART1);
	rcu_periph_clock_enable(RCU_DMA);
	
	// Init USART for 115200 baud, 8N1
	usart_baudrate_set(USART1, iBaud);
	usart_parity_config(USART1, USART_PM_NONE);
	usart_word_length_set(USART1, USART_WL_8BIT);
	usart_stop_bit_set(USART1, USART_STB_1BIT);
	usart_oversample_config(USART1, USART_OVSMOD_16);
	
	// Enable both transmitter and receiver
	usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
	usart_receive_config(USART1, USART_RECEIVE_ENABLE);
	
	//syscfg_dma_remap_enable(SYSCFG_DMA_REMAP_USART0RX|SYSCFG_DMA_REMAP_USART0TX);

	// Enable USART
	usart_enable(USART1);
	
	// Interrupt channel 3/4 enable
	TARGET_nvic_irq_enable(DMA_Channel3_4_IRQn, 2, 0);
	
	// Initialize DMA channel 4 for USART_SLAVE RX
	dma_deinit(DMA_CH4);
	dma_init_struct_usart.direction = DMA_PERIPHERAL_TO_MEMORY;
	dma_init_struct_usart.memory_addr = (uint32_t)usart1_rx_buf;
	dma_init_struct_usart.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct_usart.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_init_struct_usart.number = 1;
	dma_init_struct_usart.periph_addr = USART1_DATA_RX_ADDRESS;
	dma_init_struct_usart.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_init_struct_usart.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_init_struct_usart.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA_CH4, &dma_init_struct_usart);
	
	// Configure DMA mode
	dma_circulation_enable(DMA_CH4);
	dma_memory_to_memory_disable(DMA_CH4);

	// USART DMA enable for transmission and receive
	usart_dma_receive_config(USART1, USART_DENR_ENABLE);
	
	// Enable DMA transfer complete interrupt
	dma_interrupt_enable(DMA_CH4, DMA_CHXCTL_FTFIE);
	
	// At least clear number of remaining data to be transferred by the DMA 
	dma_transfer_number_config(DMA_CH4, 1);
	
	// Enable dma receive channel
	dma_channel_enable(DMA_CH4);
#endif
}

void USART0_Init(uint32_t iBaud)
{
#ifdef HAS_USART0
	
	// Init USART0
	#if REMOTE_USART==0 && defined(REMOTE_UARTBUS)	// no pullup resistors with multiple boards on the UartBus - Esp32/Arduino (Serial.begin) have to setup pullups
		#define USART0_PUPD	GPIO_PUPD_NONE
	#else
		#define USART0_PUPD	GPIO_PUPD_PULLUP
	#endif
	pinModeAF(USART0_TX, AF_USART0_TX, USART0_PUPD,GPIO_OSPEED_50MHZ);	// // GD32F130: GPIO_AF_0 = USART, GPIO_AF_1 = I2C
	pinModeAF(USART0_RX, AF_USART0_RX, USART0_PUPD,GPIO_OSPEED_50MHZ);	
	//gpio_mode_set(USART0_TX_PORT , GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART0_TX_PIN);	
	//gpio_mode_set(USART0_RX_PORT , GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART0_RX_PIN);
	//gpio_output_options_set(USART0_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, USART0_TX_PIN);
	//gpio_output_options_set(USART0_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, USART0_RX_PIN);	
	//gpio_af_set(USART0_TX_PORT, GPIO_AF_0, USART0_TX_PIN);	// GD32F130: GPIO_AF_0 = USART, GPIO_AF_1 = I2C
	//gpio_af_set(USART0_RX_PORT, GPIO_AF_0, USART0_RX_PIN);
	
		// Enable ADC and DMA clock
	rcu_periph_clock_enable(RCU_USART0);
	rcu_periph_clock_enable(RCU_DMA);
	
	// Init USART for USART0_BAUD baud, 8N1
	usart_baudrate_set(USART0, iBaud);
	usart_parity_config(USART0, USART_PM_NONE);
	usart_word_length_set(USART0, USART_WL_8BIT);
	usart_stop_bit_set(USART0, USART_STB_1BIT);
	usart_oversample_config(USART0, USART_OVSMOD_16);
	
	
	// Enable both transmitter and receiver
	usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
	//usart_invert_config(USART0, USART_SWAP_ENABLE);	// HarleyBob
	usart_receive_config(USART0, USART_RECEIVE_ENABLE);
	
	// Enable USART
	usart_enable(USART0);
	
	// Interrupt channel 1/2 enable
	TARGET_nvic_irq_enable(DMA_Channel1_2_IRQn, 2, 0);
	
	// Initialize DMA channel 2 for USART0 RX
	dma_deinit(DMA_CH2);
	dma_init_struct_usart.direction = DMA_PERIPHERAL_TO_MEMORY;
	dma_init_struct_usart.memory_addr = (uint32_t)usart0_rx_buf;
	dma_init_struct_usart.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct_usart.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_init_struct_usart.number = 1;
	dma_init_struct_usart.periph_addr = USART0_DATA_RX_ADDRESS;
	dma_init_struct_usart.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_init_struct_usart.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_init_struct_usart.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA_CH2, &dma_init_struct_usart);
	
	// Configure DMA mode
	dma_circulation_enable(DMA_CH2);
	dma_memory_to_memory_disable(DMA_CH2);

	// USART DMA enable for transmission and receive
	usart_dma_receive_config(USART0, USART_DENR_ENABLE);
	
	// Enable DMA transfer complete interrupt
	dma_interrupt_enable(DMA_CH2, DMA_CHXCTL_FTFIE);
	
	// At least clear number of remaining data to be transferred by the DMA 
	dma_transfer_number_config(DMA_CH2, 1);
	
	// Enable dma receive channel
	dma_channel_enable(DMA_CH2);
	
#endif
}


# define TRUE												0x01
# define FALSE												0x00

/* Clears a page of microprocessor memory */
void flashErase(uint32_t address) {
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR);
	fmc_page_erase(address);
	fmc_lock();
}

/* Reads 4 bytes from microprocessor memory */
uint32_t flashRead(uint32_t address) {
	return *(uint32_t*)address;
}

/* Writes 4 bytes to microprocessor memory */
uint8_t flashWrite(uint32_t address, uint32_t data) {
	uint8_t fflash = FALSE;
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR); 
	if (fmc_word_program(address, data) == FMC_READY) fflash = TRUE; 
	fmc_lock(); 
	return fflash;
}

/* Write a memory buffer to the microprocessor memory */
void flashWriteBuffer(uint32_t address, uint32_t pbuffer, uint8_t len) { 
	int16_t icount = 0; 
	while (1) { 
		if (flashWrite(address+icount, *(uint32_t*)(pbuffer + icount)) == TRUE) { 
		if (icount > len) break; 
		icount = icount + 4; 
		}
	}
}

/* Read into the microprocessor memory buffer */
void flashReadBuffer(uint32_t address, uint32_t pbuffer, uint8_t len) {
	int16_t icount = 0;
	while (1) {
		*(uint32_t*)(pbuffer + icount) = *(uint32_t*)(address+icount);
		if (icount > len) break;
		icount = icount + 4;
	}
}

/* Number of Flash Memory */
# define FLASH_MAX 0xFA00
/* Start Address of Flash Memory */
# define FLASH_ADRESS 0x08000000


#define EEPROM_VERSION 1
//#define STATE_InverterOn  1
ConfigData oConfig;	// = {EEPROM_VERSION,0,2048,2048,4096,0,4096,0};

void ConfigReset(void) 
{
	oConfig.iVersion = EEPROM_VERSION;
	oConfig.wState = 0;
	oConfig.iSpeedNeutral = 2048;
	oConfig.iSteerNeutral = 2048;
	oConfig.iSpeedMax = 4096;
	oConfig.iSpeedMin = 0;
	oConfig.iSteerMax = 4096;
	oConfig.iSteerMin = 0;
}


void ConfigWrite(void) 
{
	flashErase((FLASH_ADRESS + FLASH_MAX) - (sizeof(oConfig) + 4));
	flashWriteBuffer((FLASH_ADRESS + FLASH_MAX) - (sizeof(oConfig) + 4), (uint32_t)&oConfig, sizeof(oConfig) - 4);
}

void ConfigRead(void) 
{
	if (flashRead((FLASH_ADRESS + FLASH_MAX) - (sizeof(oConfig) + 4)) == 0xFFFFFFFF) 
	{
		ConfigReset();
		ConfigWrite();
	}
	flashReadBuffer((FLASH_ADRESS + FLASH_MAX) - (sizeof(oConfig) + 4), (uint32_t)&oConfig, sizeof(oConfig) - 4);
}

void confErase(void) 
{
	flashErase((FLASH_ADRESS + FLASH_MAX) - (sizeof(oConfig) + 4));
}


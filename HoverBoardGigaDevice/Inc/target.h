#ifdef GD32E230
	#include "gd32e23x.h"
	#define TARGET_nvic_irq_enable(a, b, c){nvic_irq_enable(a, b);}
	#define TARGET_nvic_priority_group_set(a)	// that function does not exist for this target = not needed ?
	#define TARGET_adc_vbat_disable()
	#define TARGET_ADC_RDATA ADC_RDATA
	
	#define TARGET_fwdgt_window_value_config(a) fwdgt_window_value_config(a)

		// it.c
	#define TARGET_DMA_Channel3_4_IRQHandler DMA_Channel3_4_IRQHandler
	#define TARGET_DMA_Channel1_2_IRQn DMA_Channel1_2_IRQn	
	#define TARGET_TIMER0_BRK_UP_TRG_COM_IRQHandler TIMER0_BRK_UP_TRG_COM_IRQHandler
	#define TARGET_adc_software_trigger_enable(a) adc_software_trigger_enable(a)		
	#define TARGET_DMA_Channel0_IRQHandler DMA_Channel0_IRQHandler	
	#define TARGET_DMA_Channel1_2_IRQHandler DMA_Channel1_2_IRQHandler
	
	#define TARGET_DMA_CH4 DMA_CH4
	#define TARGET_DMA_Channel3_4_IRQn	DMA_Channel3_4_IRQn
	#define TARGET_DMA_CH2 DMA_CH2
	#define TARGET_DMA_Channel1_2_IRQHandler DMA_Channel1_2_IRQHandler

	// setup.c
	
	#define TARGET_usart_oversample_config(a,b)	usart_oversample_config(a,b)
	#define TARGET_dma_interrupt_flag_get(a,b) dma_interrupt_flag_get(a,b)
	#define TARGET_dma_interrupt_flag_clear(a,b) dma_interrupt_flag_clear(a,b)
	#define TARGET_ADC_RDATA ADC_RDATA
	#define TARGET_adc_dma_mode_enable()	adc_dma_mode_enable()
	#define TARGET_dma_deinit(a) dma_deinit(a)
	#define TARGET_dma_init(a,b) dma_init(a,b)
	#define TARGET_dma_circulation_enable(a) dma_circulation_enable(a)
	#define TARGET_dma_memory_to_memory_disable(a) dma_memory_to_memory_disable(a)
	#define TARGET_dma_interrupt_enable(a,b) dma_interrupt_enable(a,b)
	#define TARGET_dma_transfer_number_config(a,b) dma_transfer_number_config(a,b)
	#define TARGET_dma_channel_enable(a) dma_channel_enable(a)
	#define TARGET_adc_channel_length_config(a,b) adc_channel_length_config(a,b)
	#define TARGET_adc_regular_channel_config(a,b,c)	adc_regular_channel_config(a,b,c)
	#define TARGET_adc_data_alignment_config(a)	adc_data_alignment_config(a)
	#define TARGET_adc_external_trigger_config(a,b)	adc_external_trigger_config(a,b)
	#define TARGET_adc_external_trigger_source_config(a,b)	adc_external_trigger_source_config(a,b)
	#define TARGET_adc_watchdog_disable() adc_watchdog_disable()
	#define TARGET_adc_enable()	adc_enable()
	#define TARGET_adc_calibration_enable()	adc_calibration_enable()
	#define TARGET_adc_special_function_config(a,b)	adc_special_function_config(a,b)

	#define USART0_DATA_RX_ADDRESS (uint32_t)&USART_RDATA(USART0)
	#define USART1_DATA_RX_ADDRESS (uint32_t)&USART_RDATA(USART1)
	//#define USART0_DATA_RX_ADDRESS ((uint32_t)0x40013824)
	//#define USART1_DATA_RX_ADDRESS ((uint32_t)0x40004424)

#elif defined MM32SPIN05	
	#include "mm32_device.h"
	#include "hal_conf.h"
	void TIM2_IRQHandler(void)	// just for testing
	{
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		// fine, now TIM_IT_Update is defined
	}
	#include "hal_device.h"
	#include "hal_rcc.h"
	#define ErrStatus ErrorStatus
	
	#define timer_interrupt_flag_clear TIM_ClearITPendingBit
	#define TIMER_INT_UP TIM_IT_Update

	#define dma_interrupt_flag_get(DMA_CH0, DMA_INT_FLAG_FTF) 1
	#define dma_interrupt_flag_clear(DMA_CH0, DMA_INT_FLAG_FTF) DMA_ClearITPendingBit(DMA1_IT_TC1)
	
	/* GPIO pin definitions */
	#define GPIO_PIN_0                 BIT(0)                /*!< GPIO pin 0 */
	#define GPIO_PIN_1                 BIT(1)                /*!< GPIO pin 1 */
	#define GPIO_PIN_2                 BIT(2)                /*!< GPIO pin 2 */
	#define GPIO_PIN_3                 BIT(3)                /*!< GPIO pin 3 */
	#define GPIO_PIN_4                 BIT(4)                /*!< GPIO pin 4 */
	#define GPIO_PIN_5                 BIT(5)                /*!< GPIO pin 5 */
	#define GPIO_PIN_6                 BIT(6)                /*!< GPIO pin 6 */
	#define GPIO_PIN_7                 BIT(7)                /*!< GPIO pin 7 */
	#define GPIO_PIN_8                 BIT(8)                /*!< GPIO pin 8 */
	#define GPIO_PIN_9                 BIT(9)                /*!< GPIO pin 9 */
	#define GPIO_PIN_10                BIT(10)               /*!< GPIO pin 10 */
	#define GPIO_PIN_11                BIT(11)               /*!< GPIO pin 11 */
	#define GPIO_PIN_12                BIT(12)               /*!< GPIO pin 12 */
	#define GPIO_PIN_13                BIT(13)               /*!< GPIO pin 13 */
	#define GPIO_PIN_14                BIT(14)               /*!< GPIO pin 14 */
	#define GPIO_PIN_15                BIT(15)               /*!< GPIO pin 15 */
	#define GPIO_PIN_ALL               BITS(0,15)            /*!< GPIO pin all */

	#define TARGET_ADC_RDATA ADC_RDATA

#elif defined GD32F103
	#include "gd32f10x.h"

	/* GD32F103
		#define GPIO_PIN_15 	BIT(15)		// same as gd32f130
	
		#define GPIOA                      (GPIO_BASE + 0x00000000U)
		#define GPIOB                      (GPIO_BASE + 0x00000400U)
		#define GPIOC                      (GPIO_BASE + 0x00000800U)	// ??????
		#define GPIOD                      (GPIO_BASE + 0x00000C00U)
		#define GPIOE                      (GPIO_BASE + 0x00001000U)
		#define GPIOF                      (GPIO_BASE + 0x00001400U)
		#define GPIOG                      (GPIO_BASE + 0x00001800U)

		#define GPIO_BASE             (APB2_BUS_BASE + 0x00000800U)	// ??????
		and not #define GPIO_BASE     (AHB2_BUS_BASE + 0x00000000U) // GD32F130

		#define APB2_BUS_BASE         ((uint32_t)0x40010000U)        
		and not #define AHB2_BUS_BASE ((uint32_t)0x48000000U)        // GD32F130
	*/
	
	// GD32F130 has 10 channels PA0..PA7 = 0..7 and PB0,PB1 = 8,9 . Only 64 pin MCU has further adc on GPIOC
	#define PIN_TO_CHANNEL(pin) ((pin&0xffffff00U) ==  GPIOA ? (pin&0xfU) : ((pin&0xfU)+8) )


	/*
		void gpio_init(uint32_t gpio_periph, uint32_t mode, uint32_t speed, uint32_t pin)
	    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,E,F,G) 
			\param[in]  mode: gpio pin mode, only one parameter can be selected which is shown as below:
				\arg        GPIO_MODE_AIN: analog input mode
				\arg        GPIO_MODE_IN_FLOATING: floating input mode
				\arg        GPIO_MODE_IPD: pull-down input mode
				\arg        GPIO_MODE_IPU: pull-up input mode
				\arg        GPIO_MODE_OUT_OD: GPIO output with open-drain
				\arg        GPIO_MODE_OUT_PP: GPIO output with push-pull
				\arg        GPIO_MODE_AF_OD: AFIO output with open-drain
				\arg        GPIO_MODE_AF_PP: AFIO output with push-pull
			\param[in]  speed: gpio output max speed value, only one parameter can be selected which is shown as below:
				\arg        GPIO_OSPEED_10MHZ: output max speed 10MHz
				\arg        GPIO_OSPEED_2MHZ: output max speed 2MHz
				\arg        GPIO_OSPEED_50MHZ: output max speed 50MHz
			\param[in]  pin: GPIO pin, one or more parameters can be selected which are shown as below:
				\arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
	*/

	#define GPIO_MODE_INPUT GPIO_MODE_IN_FLOATING
	#define GPIO_MODE_OUTPUT GPIO_MODE_OUT_PP
	// maybe GPIO_MODE_OUT_PP instead of GPIO_MODE_OUT_OD
	#define GPIO_MODE_ANALOG	GPIO_MODE_AIN
	#define pinMode(pin,mode) \
	{\
		gpio_init(pin&0xffffff00U, mode, GPIO_OSPEED_50MHZ, BIT(pin&0xfU));\
	}
		//	gpio_init(pin&0xffffff00U, mode, GPIO_OSPEED_10MHZ, BIT(pin&0xfU))

	#define pinModeSpeed(pin,mode,speed) \
	{\
		gpio_init(pin&0xffffff00U, mode, speed, BIT(pin&0xfU));\
	}

	/* GPIO_MODE_AIN: analog input mode
		GPIO_MODE_IN_FLOATING: floating input mode
		GPIO_MODE_IPD: pull-down input mode
		GPIO_MODE_IPU: pull-up input mode
		GPIO_MODE_OUT_OD: GPIO output with open-drain
		GPIO_MODE_OUT_PP: GPIO output with push-pull
		GPIO_MODE_AF_OD: AFIO output with open-drain
		GPIO_MODE_AF_PP: AFIO output with push-pull */
	#define GPIO_PUPD_PULLUP GPIO_MODE_IPU
	#define GPIO_PUPD_PULLDOWN GPIO_MODE_IPD
	#define GPIO_PUPD_NONE GPIO_MODE_IN_FLOATING
	#define pinModePull(pin,mode,pull) \
	{\
		gpio_init(pin&0xffffff00U, pull , GPIO_OSPEED_50MHZ, BIT(pin&0xfU));\
	}

	//     gpio_init(TIMER_BLDC_GH_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, TIMER_BLDC_GH_PIN);    

	#define pinModeAF(pin, AF, pullUpDown,speed) \
	{\
		 gpio_init(pin&0xffffff00U, pullUpDown, GPIO_OSPEED_50MHZ, BIT(pin&0xfU)); 	\
	}

	
	
	#define digitalWrite(pin,set) gpio_bit_write(pin&0xffffff00U,  (BIT(pin&0xfU) ), set)
	#define digitalRead(pin) 			gpio_input_bit_get(pin&0xffffff00U, BIT(pin&0xfU))

	// setup.c
	#define TARGET_nvic_priority_group_set nvic_priority_group_set
	#define TARGET_fwdgt_window_value_config(a) SUCCESS
	
	#define TIMER_TIMEOUT TIMER3
	#define TIMEOUT_IrqHandler TIMER3_IRQHandler
	#define RCU_TIMER_TIMEOUT	RCU_TIMER3
	#define TIMER_TIMEOUT_IRQn TIMER3_IRQn
	// bug:-(  #define rcu_periph_clock_enable(RCU_TIMER_TIMEOUT) rcu_periph_clock_enable(RCU_TIMER3)

	//setup.c::USART init
	
	#define TARGET_usart_oversample_config(a,b)	//JMA no oversampling in F103 usart_oversample_config(USART_STEER_COM, USART_OVSMOD_16);
	#define DMA_Channel1_2_IRQn DMA0_Channel2_IRQn
	#define TARGET_DMA_Channel3_4_IRQn	DMA0_Channel5_IRQn
	#define TARGET_DMA_CH4 DMA_CH5
	#define USART1_DATA_RX_ADDRESS (uint32_t)&USART_DATA(USART1)
	
	

	//setup.c::Adc_init()
	#define RCU_ADC RCU_ADC0
	#define RCU_DMA RCU_DMA0
	#define RCU_ADCCK_APB2_DIV6 RCU_CKADC_CKAPB2_DIV6
	#define TARGET_dma_deinit(a) dma_deinit(DMA0, a)
	#define TARGET_ADC_RDATA ADC_RDATA(ADC0)
	#define TARGET_dma_init(a,b) dma_init(DMA0,a,b)
	#define TARGET_dma_circulation_enable(a) dma_circulation_enable(DMA0,a)
	#define TARGET_dma_memory_to_memory_disable(a) dma_memory_to_memory_disable(DMA0,a)
	
	#define TARGET_dma_interrupt_enable(a,b) dma_interrupt_enable(DMA0,a,b)
	#define TARGET_dma_transfer_number_config(a,b) dma_transfer_number_config(DMA0,a,b)
	#define TARGET_dma_channel_enable(a) dma_channel_enable(DMA0,a)
	#define TARGET_adc_channel_length_config(a,b) adc_channel_length_config(ADC0,a,b)
	#define TARGET_adc_regular_channel_config(a,b,c)	adc_regular_channel_config(ADC0,a,b,c)
	#define TARGET_adc_data_alignment_config(a)	adc_data_alignment_config(ADC0,a)
	#define TARGET_adc_external_trigger_config(a,b)	adc_external_trigger_config(ADC0,a,b)
	#define TARGET_adc_external_trigger_source_config(a,b)	adc_external_trigger_source_config(ADC0,a,b)
	#define ADC_EXTTRIG_REGULAR_NONE ADC0_1_2_EXTTRIG_REGULAR_NONE
	#define TARGET_adc_vbat_disable adc_tempsensor_vrefint_disable
	#define TARGET_adc_watchdog_disable() adc_watchdog_disable(ADC0)
	#define TARGET_adc_enable()	adc_enable(ADC0)
	#define TARGET_adc_calibration_enable()	adc_calibration_enable(ADC0)
	#define TARGET_adc_dma_mode_enable()	adc_dma_mode_enable(ADC0)
	#define TARGET_adc_special_function_config(a,b)	adc_special_function_config(ADC0,a,b)

	#define DMA_Channel0_IRQn DMA0_Channel0_IRQn
	#define TIMER0_BRK_UP_TRG_COM_IRQn TIMER0_UP_IRQn
	#define TARGET_nvic_irq_enable(a, b, c){nvic_irq_enable(a, b, c);}
	
	// usart0
	#define FMC_FLAG_END FMC_FLAG_BANK0_END 
	#define FMC_FLAG_WPERR FMC_FLAG_BANK0_WPERR
	#define TARGET_DMA_CH2 DMA_CH4
	#define TARGET_DMA_Channel1_2_IRQn DMA0_Channel4_IRQn
	#define USART0_DATA_RX_ADDRESS (uint32_t)&USART_DATA(USART0)
	
	
	// it.c
	#define TARGET_DMA_Channel3_4_IRQHandler	DMA0_Channel5_IRQHandler
	#define TARGET_DMA_Channel1_2_IRQHandler DMA0_Channel4_IRQHandler
	#define TARGET_DMA_Channel0_IRQHandler DMA0_Channel0_IRQHandler
	#define TARGET_TIMER0_BRK_UP_TRG_COM_IRQHandler TIMER0_UP_IRQHandler
	#define TARGET_adc_software_trigger_enable(a) adc_software_trigger_enable(ADC0,a)
	#define TARGET_dma_interrupt_flag_get(a,b)	dma_interrupt_flag_get(DMA0,a,b)
	#define TARGET_dma_interrupt_flag_clear(a,b)	dma_interrupt_flag_clear(DMA0,a,b)
	
	// inserted adc
	#define TARGET_ADC_IDATA0 ADC_IDATA0(ADC0)
	#define TARGET_ADC_IDATA1 ADC_IDATA1(ADC0)
	#define TARGET_adc_interrupt_flag_get(a) adc_interrupt_flag_get(ADC0,a)
					
	#define TARGET_ADC_EXTTRIG_INSERTED_T0_TRGO	ADC0_1_EXTTRIG_INSERTED_T0_TRGO
	#define TARGET_adc_external_trigger_source_config(a,b)	adc_external_trigger_source_config(ADC0,a,b)
	#define TARGET_adc_external_trigger_config(a,b)	adc_external_trigger_config(ADC0,a,b)
	#define TARGET_adc_channel_length_config(a,b)	adc_channel_length_config(ADC0,a,b)
	#define TARGET_adc_inserted_channel_config(a,b,c)	adc_inserted_channel_config(ADC0,a,b,c)
	#define TARGET_adc_inserted_channel_offset_config(a,b)	adc_inserted_channel_offset_config(ADC0,a,b)
	#define TARGET_adc_interrupt_flag_clear(a)	adc_interrupt_flag_clear(ADC0,a);
	#define TARGET_adc_interrupt_enable(a)	adc_interrupt_enable(ADC0,a)
	#define TARGET_ADC_CMP_IRQn	ADC0_1_IRQn
	#define TARGET_ADC_CMP_IRQHandler ADC0_1_IRQHandler

	
#else
	#include "gd32f1x0.h"
	#include "gd32f1x0_gpio.h"
	#include "gd32f1x0_exti.h"
	#include "gd32f1x0_rcu.h"
	// it.c
	#define TARGET_DMA_Channel0_IRQHandler DMA_Channel0_IRQHandler
	#define TARGET_TIMER0_BRK_UP_TRG_COM_IRQHandler TIMER0_BRK_UP_TRG_COM_IRQHandler
	
	// setup.c
	
	// setup.c::usart init
	#define TARGET_usart_oversample_config(a,b)	usart_oversample_config(a,b)
	
	#define TARGET_dma_interrupt_flag_get(a,b) dma_interrupt_flag_get(a,b)
	#define TARGET_dma_interrupt_flag_clear(a,b) dma_interrupt_flag_clear(a,b)
	#define TARGET_adc_software_trigger_enable(a) adc_software_trigger_enable(a)	
	#define TARGET_fwdgt_window_value_config(a) fwdgt_window_value_config(a)
	#define TARGET_nvic_irq_enable(a, b, c){nvic_irq_enable(a, b, c);}
	#define TARGET_nvic_priority_group_set(a){nvic_priority_group_set(a);}
	#define TARGET_adc_vbat_disable(){adc_vbat_disable();}
	#define TARGET_ADC_RDATA ADC_RDATA
	#define TARGET_adc_dma_mode_enable()	adc_dma_mode_enable()
	#define TARGET_dma_deinit(a) dma_deinit(a)
	#define TARGET_dma_init(a,b) dma_init(a,b)
	#define TARGET_dma_circulation_enable(a) dma_circulation_enable(a)
	#define TARGET_dma_memory_to_memory_disable(a) dma_memory_to_memory_disable(a)
	#define TARGET_dma_interrupt_enable(a,b) dma_interrupt_enable(a,b)
	#define TARGET_dma_transfer_number_config(a,b) dma_transfer_number_config(a,b)
	#define TARGET_dma_channel_enable(a) dma_channel_enable(a)
	#define TARGET_adc_channel_length_config(a,b) adc_channel_length_config(a,b)
	#define TARGET_adc_regular_channel_config(a,b,c)	adc_regular_channel_config(a,b,c)
	#define TARGET_adc_data_alignment_config(a)	adc_data_alignment_config(a)
	#define TARGET_adc_external_trigger_config(a,b)	adc_external_trigger_config(a,b)
	#define TARGET_adc_external_trigger_source_config(a,b)	adc_external_trigger_source_config(a,b)
	#define TARGET_adc_watchdog_disable() adc_watchdog_disable()
	#define TARGET_adc_enable()	adc_enable()
	#define TARGET_adc_calibration_enable()	adc_calibration_enable()
	#define TARGET_adc_special_function_config(a,b)	adc_special_function_config(a,b)
	#define TARGET_nvic_irq_enable(a, b, c){nvic_irq_enable(a, b, c);}
	
	// usart 0
	#define TARGET_DMA_Channel1_2_IRQn DMA_Channel1_2_IRQn
	#define TARGET_DMA_CH2 DMA_CH2
	#define TARGET_DMA_Channel1_2_IRQHandler DMA_Channel1_2_IRQHandler

	// usart1
	#define TARGET_DMA_Channel3_4_IRQn	DMA_Channel3_4_IRQn
	#define TARGET_DMA_CH4	DMA_CH4
	#define TARGET_DMA_Channel3_4_IRQHandler	DMA_Channel3_4_IRQHandler

	#define USART0_DATA_RX_ADDRESS (uint32_t)&USART_RDATA(USART0)
	#define USART1_DATA_RX_ADDRESS (uint32_t)&USART_RDATA(USART1)
	//#define USART0_DATA_RX_ADDRESS ((uint32_t)0x40013824)
	//#define USART1_DATA_RX_ADDRESS ((uint32_t)0x40004424)

	// inserted adc
	#define TARGET_ADC_IDATA0 ADC_IDATA0
	#define TARGET_ADC_IDATA1 ADC_IDATA1
	#define TARGET_adc_interrupt_flag_get(a) adc_interrupt_flag_get(a)
					
	#define TARGET_ADC_EXTTRIG_INSERTED_T0_TRGO	ADC_EXTTRIG_INSERTED_T0_TRGO
	#define TARGET_adc_external_trigger_source_config(a,b)	adc_external_trigger_source_config(a,b)
	#define TARGET_adc_external_trigger_config(a,b)	adc_external_trigger_config(a,b)
	#define TARGET_adc_channel_length_config(a,b)	adc_channel_length_config(a,b)
	#define TARGET_adc_inserted_channel_config(a,b,c)	adc_inserted_channel_config(a,b,c)
	#define TARGET_adc_inserted_channel_offset_config(a,b)	adc_inserted_channel_offset_config(a,b)
	#define TARGET_adc_interrupt_flag_clear(a)	adc_interrupt_flag_clear(a);
	#define TARGET_adc_interrupt_enable(a)	adc_interrupt_enable(a)
	#define TARGET_ADC_CMP_IRQn	ADC_CMP_IRQn
	#define TARGET_ADC_CMP_IRQHandler ADC_CMP_IRQHandler
	
	
#endif



#ifndef pinMode
	
	/* GD32F130 and GD32E230
		#define GPIO_PIN_15 	BIT(15)
	
		#define GPIOA         (GPIO_BASE + 0x00000000U)
		#define GPIOB					(GPIO_BASE + 0x00000400U)
		#define GPIOC         (GPIO_BASE + 0x00000800U)
		#define GPIOD         (GPIO_BASE + 0x00000C00U)
		#define GPIOF         (GPIO_BASE + 0x00001400U)
	
		#define GPIO_BASE     (AHB2_BUS_BASE + 0x00000000U)  // !< GPIO base address
		#define AHB2_BUS_BASE ((uint32_t)0x48000000U)        //!< ahb2 base address
	*/
	
	// GD32F130 has 10 channels PA0..PA7 = 0..7 and PB0,PB1 = 8,9 . Only 64 pin MCU has further adc on GPIOC
	#define PIN_TO_CHANNEL(pin) ((pin&0xffffff00U) ==  GPIOA ? (pin&0xfU) : ((pin&0xfU)+8) )

/* function in setup.c instead of define saves memory
	#define pinMode(pin,mode) \
	{\
		gpio_mode_set(pin&0xffffff00U, mode, GPIO_PUPD_NONE,BIT(pin&0xfU) );	\
		gpio_output_options_set(pin&0xffffff00U, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, BIT(pin&0xfU));\
	}
	#define pinModePull(pin,mode,pull) \
	{\
		gpio_mode_set(pin&0xffffff00U, mode, pull,BIT(pin&0xfU) );	\
		gpio_output_options_set(pin&0xffffff00U, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, BIT(pin&0xfU));\
	}
*/
	#define pinModeSpeed(pin,mode,speed) \
	{\
		gpio_mode_set(pin&0xffffff00U, mode, GPIO_PUPD_NONE,BIT(pin&0xfU) );	\
		gpio_output_options_set(pin&0xffffff00U, GPIO_OTYPE_PP, speed, BIT(pin&0xfU));\
	}
	
	// AF = AlternateFunction
	#define pinModeAF(pin, AF, pullUpDown,speed) \
	{\
		gpio_mode_set(pin&0xffffff00U , GPIO_MODE_AF, pullUpDown, BIT(pin&0xfU));	\
		gpio_output_options_set(pin&0xffffff00U, GPIO_OTYPE_PP, speed, BIT(pin&0xfU));	\
		gpio_af_set(pin&0xffffff00U, AF(pin), BIT(pin&0xfU));		\
	}
	//GD32F130xx Datasheet	2.6.7. GD32F130xx pin alternate function	
	#define AF_TIMER0_BLDC(pin)	GPIO_AF_2	// GD32F130: all TIMER0 AF are AF2
	#define AF_TIMER0_BRKIN(pin)	(pin==PA6 ? GPIO_AF_2 : GPIO_AF_2)	// GD32F130: AF2 = PA6 or PB12
	#define AF_USART0_TX(pin)	(pin==PB6 ? GPIO_AF_0 : GPIO_AF_1)			// GD32F130: AF0 = PB6 , AF1 = PA2 or PA9 or PA14
	#define AF_USART0_RX(pin)	(pin==PB7 ? GPIO_AF_0 : GPIO_AF_1)			// GD32F130: AF0 = PB7 , AF1 = PA3 or PA15 
	#define AF_USART1_TX(pin)	(pin==PA8 ? GPIO_AF_4 : GPIO_AF_1)		// GD32F130: AF4 = PA8 , AF1 = PA2 or PA14
	#define AF_USART1_RX(pin)	(pin==PB0 ? GPIO_AF_4 : GPIO_AF_1)		// GD32F130: AF4 = PB0 , AF1 = PA3 or PA15
	// usart1 and 1 same for GD32F230: GD32E230xxDatasheet_Rev2.0.pdf page 33ff

	#define digitalWrite(pin,set) gpio_bit_write(pin&0xffffff00U,  (BIT(pin&0xfU) ), set)
	#define digitalRead(pin) 			gpio_input_bit_get(pin&0xffffff00U, BIT(pin&0xfU))

	

#endif	
	
#define PB1i	17	// 0x10 + 1
#define PB2i	18	// 0x10 + 2
#define PB11i	27	// 0x10 + 11
#define PC14i	62	// 0x30 + 14
#define PF1i	81		// 0x50 + 1
	
	#define PA15	( (uint32_t)GPIOA | 15 )
	#define PA14	( (uint32_t)GPIOA | 14 )
	#define PA13	( (uint32_t)GPIOA | 13 )
	#define PA12	( (uint32_t)GPIOA | 12 )
	#define PA11	( (uint32_t)GPIOA | 11 )
	#define PA10	( (uint32_t)GPIOA | 10 )
	#define PA9		( (uint32_t)GPIOA | 9 )
	#define PA8		( (uint32_t)GPIOA | 8 )
	#define PA7		( (uint32_t)GPIOA | 7 )
	#define PA6		( (uint32_t)GPIOA | 6 )
	#define PA5		( (uint32_t)GPIOA | 5 )
	#define PA4		( (uint32_t)GPIOA | 4 )
	#define PA3		( (uint32_t)GPIOA | 3 )
	#define PA2		( (uint32_t)GPIOA | 2 )
	#define PA1		( (uint32_t)GPIOA | 1 )
	#define PA0		( (uint32_t)GPIOA | 0 )

	#define PB15	( (uint32_t)GPIOB | 15 )
	#define PB14	( (uint32_t)GPIOB | 14 )
	#define PB13	( (uint32_t)GPIOB | 13 )
	#define PB12	( (uint32_t)GPIOB | 12 )
	#define PB11	( (uint32_t)GPIOB | 11 )
	#define PB10	( (uint32_t)GPIOB | 10 )
	#define PB9		( (uint32_t)GPIOB | 9 )
	#define PB8		( (uint32_t)GPIOB | 8 )
	#define PB7		( (uint32_t)GPIOB | 7 )
	#define PB6		( (uint32_t)GPIOB | 6 )
	#define PB5		( (uint32_t)GPIOB | 5 )
	#define PB4		( (uint32_t)GPIOB | 4 )
	#define PB3		( (uint32_t)GPIOB | 3 )
	#define PB2		( (uint32_t)GPIOB | 2 )
	#define PB1		( (uint32_t)GPIOB | 1 )
	#define PB0		( (uint32_t)GPIOB | 0 )

	#define PC15	( (uint32_t)GPIOC | 15 )
	#define PC14	( (uint32_t)GPIOC | 14 )
	#define PC13	( (uint32_t)GPIOC | 13 )
	#define PC12	( (uint32_t)GPIOC | 12 )
	#define PC11	( (uint32_t)GPIOC | 11 )
	#define PC10	( (uint32_t)GPIOC | 10 )
	#define PC9	( (uint32_t)GPIOC | 9 )
	#define PC8	( (uint32_t)GPIOC | 8 )
	#define PC7	( (uint32_t)GPIOC | 7 )
	#define PC6	( (uint32_t)GPIOC | 6 )
	#define PC5	( (uint32_t)GPIOC | 5 )
	#define PC4	( (uint32_t)GPIOC | 4 )
	#define PC3	( (uint32_t)GPIOC | 3 )
	#define PC2	( (uint32_t)GPIOC | 2 )
	#define PC1	( (uint32_t)GPIOC | 1 )
	#define PC0	( (uint32_t)GPIOC | 0 )

	#define PF7	( GPIOF | 7 )
	#define PF6	( GPIOF | 6 )
	#define PF4	( GPIOF | 4 )
	#define PF1	( GPIOF | 1 )
	#define PF0	( GPIOF | 0 )


/* GD32F130 48pin possible IO pins: 
	C13 C14 C15 F0 F1 A0 A1 A2 
	A3 A4 A5 A6 A7 B0 B1 B2 B10 B11
	B12 B13 B14 B15 A8 A9 A10 A11 A12 A13 F6 F7
	A14 A15 B3 B4 B5 B6 B7 B8 B9 	*/ 

	
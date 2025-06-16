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
#include "../Inc/bldc.h"
#include "../Inc/led.h"
#include "../Inc/commsMasterSlave.h"

//#include "../Inc/commsSteering.h"

#include "../Inc/commsBluetooth.h"

uint32_t msTicks;
uint32_t timeoutCounter_ms = 0;
FlagStatus timedOut = RESET;

#ifdef SLAVE
uint32_t hornCounter_ms = 0;
#endif

extern int32_t steer;
extern int32_t speed;
extern FlagStatus activateWeakening;
extern FlagStatus beepsBackwards;

//----------------------------------------------------------------------------
// SysTick_Handler
//----------------------------------------------------------------------------
void SysTick_Handler(void)
{
  msTicks++;
}

//----------------------------------------------------------------------------
// Resets the timeout to zero
//----------------------------------------------------------------------------
void ResetTimeout(void)
{
  timeoutCounter_ms = 0;
}

//----------------------------------------------------------------------------
// Timer13_Update_Handler
// Is called when upcouting of TIMER_TIMEOUT (timer13) is finished and the UPDATE-flag is set
// -> period of timer13 running with 1kHz -> interrupt every 1ms
//----------------------------------------------------------------------------
void TIMEOUT_IrqHandler(void)
{	
	if (timeoutCounter_ms > TIMEOUT_MS)
	{
		// First timeout reset all process values
		if (timedOut == RESET)	// robo: had been RESET = bug ?
		{
#ifdef MASTER
			steer = 0;
			speed = 0;
			beepsBackwards = RESET;
#else
			SetPWM(0);
#endif
		}
		timedOut = SET;
	}
	else
	{
		timedOut = RESET;
		timeoutCounter_ms++;
	}

#ifdef SLAVE
	if (hornCounter_ms >= 2000)
	{
		// Avoid horn to be activated longer than 2 seconds
		SetUpperLEDMaster(RESET);
	}
	else if (hornCounter_ms < 2000)
	{
		hornCounter_ms++;
	}
	
	// Update LED program
	CalculateLEDProgram();
#endif
	
	// Clear timer update interrupt flag
	timer_interrupt_flag_clear(TIMER_TIMEOUT, TIMER_INT_UP);
}

//----------------------------------------------------------------------------
// Timer0_Update_Handler
// Is called when upcouting of timer0 is finished and the UPDATE-flag is set
// AND when downcouting of timer0 is finished and the UPDATE-flag is set
// -> pwm of timer0 running with 16kHz -> interrupt every 31,25us
//----------------------------------------------------------------------------
extern uint32_t steerCounter;								// Steer counter for setting update rate
uint32_t iPwmTicks = 0, iPwmTicks0 = 0, iPwmCounter = 0;
uint32_t iAdcTicks = 0, iAdcTicks0 = 0, iAdcCounter = 0;
#define COUNT_Irqs 1000

//void TIMER0_UP_IRQHandler(void)	//JMA must match the name in startup_gd32f10x_hd.s
#ifndef TARGET_TIMER0_BRK_UP_TRG_COM_IRQHandler
	#error "TIMER0_BRK_UP_TRG_COM_IRQHandler not defined for active target in target.h"
#endif
void TARGET_TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
{
	if (COUNT_Irqs == ++iPwmCounter)
	{
		iPwmTicks = msTicks - iPwmTicks0;
		iPwmTicks0 = msTicks;
		iPwmCounter = 0;
	}
	// Start ADC conversion
	TARGET_adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
	//adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL); //jma: ADC0 added for GD32F103
		
	// Clear timer update interrupt flag
	timer_interrupt_flag_clear(TIMER_BLDC, TIMER_INT_UP);
}

//----------------------------------------------------------------------------
// This function handles DMA_Channel0_IRQHandler interrupt
// Is called, when the ADC scan sequence is finished
// -> ADC is triggered from timer0-update-interrupt -> every 31,25us
//----------------------------------------------------------------------------
#ifndef TARGET_DMA_Channel0_IRQHandler
	#error "TARGET_DMA_Channel0_IRQHandler not defined for active target in target.h"
#endif
void TARGET_DMA_Channel0_IRQHandler(void)
{
	if (COUNT_Irqs == ++iAdcCounter)
	{
		iAdcTicks = msTicks - iAdcTicks0;
		iAdcTicks0 = msTicks;
		iAdcCounter = 0;
	}
	CalculateBLDC(); //moved behind flag_clear by Deepseek, Safe: NVIC blocks re-entrancy

	if (TARGET_dma_interrupt_flag_get(DMA_CH0, DMA_INT_FLAG_FTF))
	{
			TARGET_dma_interrupt_flag_clear(DMA_CH0, DMA_INT_FLAG_FTF);
	}	
}


uint32_t iCounterUsart0 = 0;
uint32_t iCounter2Usart0 = 0;

#ifdef HAS_USART0
	// Is asynchronously called when USART0 RX finished
	#ifndef TARGET_DMA_Channel1_2_IRQHandler
		#error "TARGET_DMA_Channel1_2_IRQHandler not defined for active target in target.h"
	#endif
	void TARGET_DMA_Channel1_2_IRQHandler(void)
	{
		iCounterUsart0++;
		//DEBUG_LedSet(	(steerCounter%20) < 10	,0)
		// USART steer/bluetooth RX
		if (TARGET_dma_interrupt_flag_get(TARGET_DMA_CH2, DMA_INT_FLAG_FTF))
		{
			iCounter2Usart0++;
			//DEBUG_LedSet(	(iCounter2Usart0++%10) < 5	,0)
			#if (REMOTE_USART==0) && defined(MASTER_OR_SINGLE)
					RemoteCallback();
			#elif (MASTERSLAVE_USART==0) && defined(MASTER_OR_SLAVE)
					UpdateUSARTMasterSlaveInput();
					// Update USART bluetooth input mechanism
					//UpdateUSARTBluetoothInput();
			#endif
			TARGET_dma_interrupt_flag_clear(TARGET_DMA_CH2, DMA_INT_FLAG_FTF);        
		}
	}
#endif

#ifdef HAS_USART1
	//----------------------------------------------------------------------------
	// This function handles DMA_Channel3_4_IRQHandler interrupt
	// Is asynchronously called when USART_SLAVE RX finished
	//----------------------------------------------------------------------------
	void DMA_Channel3_4_IRQHandler(void)
	{
		//DEBUG_LedSet(	(steerCounter%10) < 5	,0)
		// USART master slave RX
		if (dma_interrupt_flag_get(DMA_CH4, DMA_INT_FLAG_FTF))
		{
			#if (REMOTE_USART==1) && defined(MASTER_OR_SINGLE)
					RemoteCallback();
			#elif (MASTERSLAVE_USART==1) && defined(MASTER_OR_SLAVE)
					UpdateUSARTMasterSlaveInput();
					// Update USART bluetooth input mechanism
					//UpdateUSARTBluetoothInput();
			#endif
			
			dma_interrupt_flag_clear(DMA_CH4, DMA_INT_FLAG_FTF);        
		}
	}
#endif

uint32_t iCounterUsart2 = 0;
uint32_t iCounter2Usart2 = 0;
#if TARGET==2 && defined(HAS_USART2)
	//----------------------------------------------------------------------------
	// This function handles DMA_Channel3_4_IRQHandler interrupt
	// Is asynchronously called when USART_SLAVE RX finished
	//----------------------------------------------------------------------------
	void DMA0_Channel2_IRQHandler(void)		// DMA_Channel3_4_IRQHandler
	{
		iCounterUsart2++;
		//DEBUG_LedSet(	(steerCounter%10) < 5	,0)
		// USART master slave RX
		if (TARGET_dma_interrupt_flag_get(DMA_CH2, DMA_INT_FLAG_FTF))
		{
			iCounter2Usart2++;
			#if (REMOTE_USART==2) && defined(MASTER_OR_SINGLE)
					RemoteCallback();
			#elif (MASTERSLAVE_USART==2) && defined(MASTER_OR_SLAVE)
					UpdateUSARTMasterSlaveInput();
					// Update USART bluetooth input mechanism
					//UpdateUSARTBluetoothInput();
			#endif
			
			TARGET_dma_interrupt_flag_clear(DMA_CH2, DMA_INT_FLAG_FTF);        
		}
	}
#endif


//----------------------------------------------------------------------------
// Returns number of milliseconds since system start
//----------------------------------------------------------------------------
uint32_t millis()
{
	return msTicks;
}

//----------------------------------------------------------------------------
// Delays number of tick Systicks (happens every 10 ms)
//----------------------------------------------------------------------------
void Delay (uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks)
	{
		__NOP();
	}
}

//----------------------------------------------------------------------------
// This function handles Non maskable interrupt.
//----------------------------------------------------------------------------
void NMI_Handler(void)
{
}

//----------------------------------------------------------------------------
// This function handles Hard fault interrupt.
//----------------------------------------------------------------------------
void HardFault_Handler(void)
{
  while(1) {}
}

//----------------------------------------------------------------------------
// This function handles Memory management fault.
//----------------------------------------------------------------------------
void MemManage_Handler(void)
{
  while(1) {}
}

//----------------------------------------------------------------------------
// This function handles Prefetch fault, memory access fault.
//----------------------------------------------------------------------------
void BusFault_Handler(void)
{
  while(1) {}
}

//----------------------------------------------------------------------------
// This function handles Undefined instruction or illegal state.
//----------------------------------------------------------------------------
void UsageFault_Handler(void)
{
  while(1) {}
}

//----------------------------------------------------------------------------
// This function handles System service call via SWI instruction.
//----------------------------------------------------------------------------
void SVC_Handler(void)
{
}

//----------------------------------------------------------------------------
// This function handles Debug monitor.
//----------------------------------------------------------------------------
void DebugMon_Handler(void)
{
}

//----------------------------------------------------------------------------
// This function handles Pendable request for system service.
//----------------------------------------------------------------------------
void PendSV_Handler(void)
{
}

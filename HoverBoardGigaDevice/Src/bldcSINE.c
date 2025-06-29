#include "../Inc/bldcSINE.h"
#include "../Inc/it.h"

#ifdef BLDC_SINE
#include <math.h>


// Precomputed sine table for 0-360° (1° steps) in Q15 format
#define SIN_TABLE_SIZE 360
static const int16_t sine_table[SIN_TABLE_SIZE] = {
0, 572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126, 5690, 6252, 
6813, 7371, 7927, 8481, 9032, 9580, 10126, 10668, 11207, 11743, 
12275, 12803, 13328, 13848, 14364, 14876, 15383, 15886, 16384, 
16876, 17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621, 
21062, 21497, 21925, 22347, 22762, 23170, 23571, 23964, 24351, 
24730, 25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 
27788, 28087, 28377, 28659, 28932, 29196, 29451, 29697, 29934, 
30162, 30381, 30591, 30791, 30982, 31163, 31335, 31498, 31651, 
31794, 31928, 32052, 32166, 32270, 32365, 32449, 32524, 32588, 
32643, 32688, 32723, 32748, 32763, 32767, 32763, 32748, 32723, 
32688, 32643, 32588, 32524, 32449, 32365, 32270, 32166, 32052, 
31928, 31794, 31651, 31498, 31335, 31163, 30982, 30791, 30591, 
30381, 30162, 29934, 29697, 29451, 29196, 28932, 28659, 28377, 
28087, 27788, 27481, 27165, 26841, 26509, 26169, 25821, 25465, 
25101, 24730, 24351, 23964, 23571, 23170, 22762, 22347, 21925, 
21497, 21062, 20621, 20173, 19720, 19260, 18794, 18323, 17846, 
17364, 16876, 16384, 15886, 15383, 14876, 14364, 13848, 13328, 
12803, 12275, 11743, 11207, 10668, 10126, 9580, 9032, 8481, 
7927, 7371, 6813, 6252, 5690, 5126, 4560, 3993, 3425, 2856, 
2286, 1715, 1144, 572, 0, -572, -1144, -1715, -2286, -2856, 
-3425, -3993, -4560, -5126, -5690, -6252, -6813, -7371, -7927, 
-8481, -9032, -9580, -10126, -10668, -11207, -11743, -12275, 
-12803, -13328, -13848, -14364, -14876, -15383, -15886, -16384, 
-16876, -17364, -17846, -18323, -18794, -19260, -19720, -20173, 
-20621, -21062, -21497, -21925, -22347, -22762, -23170, -23571, 
-23964, -24351, -24730, -25101, -25465, -25821, -26169, -26509, 
-26841, -27165, -27481, -27788, -28087, -28377, -28659, -28932, 
-29196, -29451, -29697, -29934, -30162, -30381, -30591, -30791, 
-30982, -31163, -31335, -31498, -31651, -31794, -31928, -32052, 
-32166, -32270, -32365, -32449, -32524, -32588, -32643, -32688, 
-32723, -32748, -32763, -32767, -32763, -32748, -32723, -32688, 
-32643, -32588, -32524, -32449, -32365, -32270, -32166, -32052, 
-31928, -31794, -31651, -31498, -31335, -31163, -30982, -30791, 
-30591, -30381, -30162, -29934, -29697, -29451, -29196, -28932, 
-28659, -28377, -28087, -27788, -27481, -27165, -26841, -26509, 
-26169, -25821, -25465, -25101, -24730, -24351, -23964, -23571, 
-23170, -22762, -22347, -21925, -21497, -21062, -20621, -20173, 
-19720, -19260, -18794, -18323, -17846, -17364, -16876, -16384, 
-15886, -15383, -14876, -14364, -13848, -13328, -12803, -12275, 
-11743, -11207, -10668, -10126, -9580, -9032, -8481, -7927, 
-7371, -6813, -6252, -5690, -5126, -4560, -3993, -3425, -2856, 
-2286, -1715, -1144, -572
};

// Sector mapping table (hall position to sector)
static const uint8_t hall_to_sector[8] = {
	0,	// ilegal 0
	1,	// 0b100	1 idx
	5,	// 0b010	2
	0,	// 0b110	3
	3,	// 0b001	4
	2,	// 0b101	5
	4,	// 0b011	6
	0		// 0b111 = illegal
};


extern uint16_t buzzerTimer;	// counter in CalculateBLDC() running at PWM_FREQ (16 kHz)
extern volatile uint8_t hall;        // gpio hall state
volatile uint16_t hall_time_step;	// time in buzzerTimer ticks between to hall interrupts/changes
volatile uint16_t hall_time_last;  // Timestamp of last edge
volatile uint8_t hall_last;        // previous hall state
volatile uint8_t bInterrupt;

// Phase shift constants (120° apart)
#define PHASE_B_OFFSET 120
#define PHASE_C_OFFSET 240

static int8_t sectorLast = 0;
int8_t sectorChange = 0;
int8_t sector = 0;
int8_t sector10x = 0;
float fDT = 0;
float angle_deg = 0;
float angle_deg_add = 0;
float angle_deg_final = 0;
uint16_t angle_idx = 0;
int pwmGo = 0;

#if TARGET == 22

#include "gd32f10x.h"
#include "gd32f10x_gpio.h"
#include "gd32f10x_exti.h"
#include "gd32f10x_rcu.h"

uint32_t InitEXTI(uint32_t iPinArduino) 
{
    uint8_t  iPin  = iPinArduino & 0xFF;
    uint32_t iPort = iPinArduino & 0xFFFFFF00; 
    
	  // Configure GPIO pin as input with pull-up
    gpio_init(iPort, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, BIT(iPin));	// GPIO_MODE_IPU
	
    uint32_t iPinEXTI = BIT(iPin);
    uint8_t iPortEXTI_SOURCE = (iPort & 0xFFFF) / 0x400; // EXTI_SOURCE_GPIOA=0, GPIOB=1, etc.
    
    gpio_exti_source_select(iPortEXTI_SOURCE, iPin); // Changed function
    exti_init(iPinEXTI, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_flag_clear(iPinEXTI);
    
    // Configure NVIC with highest priority (0)
    if (iPin == 0)       nvic_irq_enable(EXTI0_IRQn, 0, 1);
    else if (iPin == 1)  nvic_irq_enable(EXTI1_IRQn, 0, 1);
    else if (iPin == 2)  nvic_irq_enable(EXTI2_IRQn, 0, 1);
    else if (iPin == 3)  nvic_irq_enable(EXTI3_IRQn, 0, 1);
    else if (iPin == 4)  nvic_irq_enable(EXTI4_IRQn, 0, 1);
    else if (iPin < 10)  nvic_irq_enable(EXTI5_9_IRQn, 0, 1);
    else                 nvic_irq_enable(EXTI10_15_IRQn, 0, 1);    
   return iPinEXTI;
}

uint32_t aHallEXTI[3];
uint32_t aHallCountEXTI[3] = {0,0,0};


void InitBldc() {
    //rcu_periph_clock_enable(RCU_GPIOA);	allready done in GPIO_init()
    rcu_periph_clock_enable(RCU_AF); // Required for EXTI configuration
	
	
    aHallEXTI[0] = InitEXTI(HALL_A);
    aHallEXTI[1] = InitEXTI(HALL_B);
    aHallEXTI[2] = InitEXTI(HALL_C);
}

int8_t iIrqLast = -1;
uint32_t iIrqCount = 0;

// _HandleEXTI remains identical to original
void _HandleEXTI(int8_t iIrq) 
{
	iIrqLast = iIrq;
	iIrqCount++;
	uint8_t i=0;for (; i<3; i++)
	if (exti_interrupt_flag_get(aHallEXTI[i])!= RESET)
	{
		bInterrupt = 1;
		hall_time_step = buzzerTimer>hall_time_last ? buzzerTimer-hall_time_last : buzzerTimer + (0x00010000-hall_time_last);
		hall_time_last = buzzerTimer;  // PWM_FREQ (16 kHz)
		hall_last = hall;
		hall = digitalRead(HALL_A) + digitalRead(HALL_B)*2 + digitalRead(HALL_C)*4;		
		aHallCountEXTI[i]++;
		exti_interrupt_flag_clear(aHallEXTI[i]);
		//exti_flag_clear(aHallEXTI[i]);
	}
}

void EXTI0_IRQHandler(void) { _HandleEXTI(0); }
void EXTI1_IRQHandler(void) { _HandleEXTI(1); }
void EXTI2_IRQHandler(void) { _HandleEXTI(2); }
void EXTI3_IRQHandler(void) { _HandleEXTI(3); }
void EXTI4_IRQHandler(void) { _HandleEXTI(4); }

// Grouped EXTI handlers
void EXTI5_9_IRQHandler(void)   { _HandleEXTI(5); }
void EXTI10_15_IRQHandler(void) { _HandleEXTI(10); }
#endif

#if TARGET == 2

uint32_t iExti = 0;
uint16_t iExti0 = 0;
uint16_t iExti1 = 0;
uint16_t iExti2 = 0;


void GPIO_Config(void)
{
	// Enable GPIO clocks
	//rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOA);

	// Configure PB3 (LED) as push-pull output
	//gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

	// Configure PA1 (EXTI source) as input with pull-down
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
}

void EXTI_Config(void)
{
	// Connect EXTI1 line to PA1
	gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_0);
	gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_1);
	gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_2);

	// Configure EXTI for PA1 (both edges)
	exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	exti_interrupt_flag_clear(EXTI_0);  // Clear pending bit
	exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	exti_interrupt_flag_clear(EXTI_1);  // Clear pending bit
	exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	exti_interrupt_flag_clear(EXTI_2);  // Clear pending bit
}

void NVIC_Config(void)
{
	// Enable EXTI1 interrupt
	nvic_irq_enable(EXTI0_IRQn, 0, 1);	// first 0 = can interrupt other 1+ interrupts but not bldc=(0,0) which also comes first if both are pending
	nvic_irq_enable(EXTI1_IRQn, 0, 1);
	nvic_irq_enable(EXTI2_IRQn, 0, 1);
}

void _HandleEXTI()
{
	bInterrupt = 1;
	hall_time_step = buzzerTimer>hall_time_last ? buzzerTimer-hall_time_last : buzzerTimer + (0x00010000-hall_time_last);
	hall_time_last = buzzerTimer;  // PWM_FREQ (16 kHz)
	hall_last = hall;
	hall = digitalRead(HALL_A) + digitalRead(HALL_B)*2 + digitalRead(HALL_C)*4;		
}


void EXTI0_IRQHandler(void)	// EXTI0 Interrupt Service Routine
{
	if(RESET != exti_interrupt_flag_get(EXTI_0)) 
	{
		iExti0++;
		_HandleEXTI();
		exti_interrupt_flag_clear(EXTI_0);
	}
}
void EXTI1_IRQHandler(void)	// EXTI1 Interrupt Service Routine
{
	if(RESET != exti_interrupt_flag_get(EXTI_1)) 
	{
		iExti1++;
		_HandleEXTI();
		exti_interrupt_flag_clear(EXTI_1);
	}
}
void EXTI2_IRQHandler(void)	// EXTI2 Interrupt Service Routine
{
	if(RESET != exti_interrupt_flag_get(EXTI_2)) 
	{
		iExti2++;
		_HandleEXTI();
		exti_interrupt_flag_clear(EXTI_2);
	}
}

void InitBldc()
{
	// System configuration
	rcu_periph_clock_enable(RCU_AF);
	GPIO_Config();
	EXTI_Config();
	NVIC_Config();
}


#else

uint32_t InitEXTI(uint32_t iPinArduino)		// init EXTernal Interrupt
{
	uint8_t		iPin	= iPinArduino & 0xFF ;
	uint32_t 	iPort = iPinArduino & 0xFFFFFF00; 
	
	//EXTI_11; EXTI_SOURCE_GPIOA;
	uint32_t iPinEXTI = BIT(iPin);
	uint8_t iPortEXTI_SOURCE = (iPort&0xFFFF)/0x400;		// EXTI_SOURCE_GPIOA=0, 1, 2, EXTI_SOURCE_GPIOD=3, EXTI_SOURCE_GPIOF=5
	
	syscfg_exti_line_config(iPortEXTI_SOURCE , iPin);
	exti_init( iPinEXTI, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	exti_flag_clear(iPinEXTI);
	
	// Configure NVIC with highest priority (0)
	if 			(iPin<2)	TARGET_nvic_irq_enable(EXTI0_1_IRQn, 0, 1)      // PF1 for example
	else if	(iPin<4)	TARGET_nvic_irq_enable(EXTI2_3_IRQn, 0, 1)      // PB2 for example
	else 	TARGET_nvic_irq_enable(EXTI4_15_IRQn, 0, 1)      // PC14 for example
	
	return iPinEXTI;
}

uint32_t aHallEXTI[3];
void InitBldc()
{
	rcu_periph_clock_enable(RCU_CFGCMP);  // Combines AF and SYSCFG functionality
	aHallEXTI[0] = InitEXTI(HALL_A);
	aHallEXTI[1] = InitEXTI(HALL_B);
	aHallEXTI[2] = InitEXTI(HALL_C);		// PA2 on 2.1.4 !
}

void _HandleEXTI()
{
	uint8_t i=0;for (; i<3; i++)
	if (exti_interrupt_flag_get(aHallEXTI[i]) != RESET)
	{
		bInterrupt = 1;
		hall_time_step = buzzerTimer>hall_time_last ? buzzerTimer-hall_time_last : buzzerTimer + (0x00010000-hall_time_last);
		hall_time_last = buzzerTimer;  // PWM_FREQ (16 kHz)
		hall_last = hall;
		hall = digitalRead(HALL_A) + digitalRead(HALL_B)*2 + digitalRead(HALL_C)*4;		
		exti_flag_clear(aHallEXTI[i]);
	}
}
void EXTI0_1_IRQHandler(void) {	_HandleEXTI();}
void EXTI2_3_IRQHandler(void) {	_HandleEXTI();}
void EXTI4_15_IRQHandler(void){	_HandleEXTI();}

#endif

uint32_t hall_time_step_LPR,hall_time_step_LP;
float fGradient = 0, fGradientX = 0;
uint16_t iDT = 0;

void bldc_get_pwm(int pwm, int pos, int *y, int *b, int *g) 	// pos is not used but hall_to_sector mapping :-/
{
	pwmGo = -pwm;
	uint32_t safe_hall_time_last,safe_hall_time_step;
	uint8_t iRetries = 10;
	do	// Get current sector (0-5) and other interrupt set data
	{
		bInterrupt = 0;
		sector = hall_to_sector[hall];
		sectorLast = hall_to_sector[hall_last];
		safe_hall_time_last = hall_time_last;  // Timestamp of last edge. last_edge and pos might change any time by gpio interrupt! 
		safe_hall_time_step = hall_time_step;
	} while (bInterrupt && iRetries--);	 // during last three mappings, a hall interrupt might have occured

	sector10x = 10* sector;
	
	angle_deg = sector * 60.0f;	// Calculate base angle for this sector (0-60° range)
	angle_deg_add = 0;
	int16_t iOffset = 0;
	
	// Calculate low-pass filter for pwm value
	#define RANK_hts 9
	hall_time_step_LPR = hall_time_step_LPR - (hall_time_step_LPR >> RANK_hts) + safe_hall_time_step;
	
	if (ABS(pwmGo) > 50)
	{
		if (sector != sectorLast) 
		{
			sectorChange = (sector - sectorLast + 6) % 6;	// normaly +1 or -1 but could be more if a hall position is skipped because of PWM_FREQ sampling frequency
			if (sectorChange > 3) sectorChange -= 6;  // Handle reverse direction
		}
		
		if (	(safe_hall_time_step > 0) && (safe_hall_time_step < (PWM_FREQ/20))	)	// >50ms for one hall change is to slow to interpolate
		{
			hall_time_step_LP = hall_time_step_LPR >> RANK_hts;
			
			iDT = buzzerTimer>safe_hall_time_last ? buzzerTimer-safe_hall_time_last : buzzerTimer + (0x00010000-safe_hall_time_last);
			fGradient = 1.0/hall_time_step_LP;
			fGradientX = fGradient * 40000;
			fDT = (float)iDT * fGradient;		// at constant speed, fDT == 1 would be right before a new sector gets detected by the hall inputs
			//fDT = (float)iDT /safe_hall_time_step;		// at constant speed, fDT == 1 would be right before a new sector gets detected by the hall inputs
			if (fDT < 1.5)	// not for heavy breaking
			{
				angle_deg_add = sectorChange * (60.0f  * fDT);
				iOffset = sectorChange < 0 ? 390 : 330;		// +-30° +360° to prevent negative angle
			}
		}
	}
	angle_deg_final  = angle_deg + angle_deg_add + iOffset;
	

	angle_idx = (uint16_t)angle_deg_final % 360;	// Convert to integer lookup index (0-359)
	// Calculate phase offsets
	uint16_t angle_idx_b = (angle_idx + PHASE_B_OFFSET) % 360;
	uint16_t angle_idx_c = (angle_idx + PHASE_C_OFFSET) % 360;

	// Apply sine table (Q15 multiplication)
	*y = (pwmGo * (int32_t)sine_table[angle_idx]) >> 15;
	*b = (pwmGo * (int32_t)sine_table[angle_idx_b]) >> 15;
	*g = (pwmGo * (int32_t)sine_table[angle_idx_c]) >> 15;
}


#endif
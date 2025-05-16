#include "config.h"

#if COMMUTATION_MODE == SVM_COMMUTATION

#include <stdio.h>

#include "commutation.h"
#include "mathDefines.h"

static const uint8_t HALL_A_MASK = 0x4;
static const uint8_t HALL_B_MASK = 0x2;
static const uint8_t HALL_C_MASK = 0x1;

static const int16_t sine_q15[61] = {
       0, 572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126, 5690, 6252, 6813, 7371, 7927, 8481, 9032, 9580, 10126, 10668, 11207, 11743, 12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886, 16384, 16877, 17364, 17847, 18324, 18795, 19261, 19720, 20174, 20622, 21063, 21498, 21926, 22348, 22763, 23170, 23571, 23965, 24351, 24730, 25102, 25466, 25822, 26170, 26510, 26842, 27166, 27482, 27789, 28088, 28378
};

// Uses Jantzen Lee convention https://www.youtube.com/watch?v=XCzfHDnt6G4
uint8_t getPosition(uint8_t hall)
{
	switch (hall)
	{
		case 0b100:
			return 0;
		case 0b101:
			return 1;
		case 0b001:
			return 2;
		case 0b011:
			return 3;
		case 0b010:
			return 4;
		case 0b110:
			return 5;
		default:
			return 0; // Not sure what a good convention for invalid state is.  Maybe the MCU should panic?
					  // If we're field weakning, crashes can cause back EMF that blows stuff up. // https://www.youtube.com/watch?v=5eQyoVMz1dY
	}
}

void svmPWM(int sector, int t0, int t1, int t2,
            uint32_t *phaseA, uint32_t *phaseB, uint32_t *phaseC)
{
    // exact integer half of the zero vector time
    uint32_t h = (uint32_t)t0 >> 1;

    switch (sector)
    {
    case 0:
        *phaseA = h + (uint32_t)t2;
        *phaseB = h;
        *phaseC = h + (uint32_t)t1 + (uint32_t)t2;
        break;
    case 1:
        *phaseA = h + (uint32_t)t1 + (uint32_t)t2;
        *phaseB = h;
        *phaseC = h + (uint32_t)t1;
        break;
    case 2:
        *phaseA = h + (uint32_t)t1 + (uint32_t)t2;
        *phaseB = h + (uint32_t)t2;
        *phaseC = h;
        break;
    case 3:
        *phaseA = h + (uint32_t)t1;
        *phaseB = h + (uint32_t)t1 + (uint32_t)t2;
        *phaseC = h;
        break;
    case 4:
        *phaseA = h;
        *phaseB = h + (uint32_t)t1 + (uint32_t)t2;
        *phaseC = h + (uint32_t)t2;
        break;
    case 5:
        *phaseA = h;
        *phaseB = h + (uint32_t)t2;
        *phaseC = h + (uint32_t)t1 + (uint32_t)t2;
        break;
    default:
        // fault condition: all off
        *phaseA = 0;
        *phaseB = 0;
        *phaseC = 0;
        break;
    }
}


int32_t calculateT(int16_t angle, int16_t modIndexPermille) {
    // 1) clamp inputs
    if (angle < 0)      angle = 0;
    if (angle > 61)     angle = 61;

    // 2) convert –DUTY_CYCLE_100_PERCENT…+DUTY_CYCLE_100_PERCENT → Q15 (–1.0…+1.0)
    //    scale by 32767, add ±500 for rounding, then divide by DUTY_CYCLE_100_PERCENT
    int32_t M_Q15 = (int32_t)modIndexPermille * 32767;
    M_Q15 += (modIndexPermille >= 0 ?  500 : -500);
    M_Q15 /= DUTY_CYCLE_100_PERCENT;                     // now in Q15

    // 3) look up sin(angle) in Q15
    int32_t s_Q15 = sine_q15[angle];

    // 4) prod = M_Q15 * s_Q15 → Q30
    int64_t prod30 = (int64_t)M_Q15 * s_Q15;

    // 5) divide out sin(60°) (Q15): Q30 / Q15 → Q15
    const int32_t sin60_Q15 = sine_q15[60];
    // add half for rounding
    int32_t t_Q15 = (int32_t)((prod30 + (sin60_Q15 >> 1)) / sin60_Q15);

    // 6) scale Q15 → ticks: (Q15 * PERIOD) >> 15, with rounding
    int64_t scaled = (int64_t)t_Q15 * BLDC_TIMER_PERIOD;
    int32_t ticks  = (int32_t)((scaled + (1LL << 14)) >> 15);

    return ticks;
}

uint8_t get_sector(uint8_t hall_a, uint8_t hall_b, uint8_t hall_c)
{
  uint8_t hall = hall_a * HALL_A_MASK + hall_b * HALL_B_MASK + hall_c * HALL_C_MASK;
  return getPosition(hall);
}

void get_pwm(int dutyCycle, uint8_t sector, uint32_t *pulse_a, uint32_t *pulse_b, uint32_t *pulse_c)
{
    int angle = 30 + PHASE_ADVANCE_AT_MAX_PWM_DEGREES * dutyCycle / DUTY_CYCLE_100_PERCENT;
    // This is based on Jantzen Lee's equations https://youtu.be/oHEVdXucSJs?t=510

    if (dutyCycle < 0) {
        sector = (sector + 3) % 6; // Go backwards
    }
    int32_t t1 = calculateT(60 - angle, ABS(dutyCycle));
    int32_t t2 = calculateT(angle, ABS(dutyCycle));

	int32_t t0 = BLDC_TIMER_PERIOD - t1 - t2;
    if (t0 < 0) {
        t0 = 0;
    }

    uint32_t phaseA, phaseB, phaseC;
    svmPWM(sector, t0, t1, t2, &phaseA, &phaseB, &phaseC);
    *pulse_a = CLAMP(phaseA, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE);
    *pulse_b = CLAMP(phaseB, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE);
    *pulse_c = CLAMP(phaseC, BLDC_TIMER_MIN_VALUE, BLDC_TIMER_MAX_VALUE);
}

#endif
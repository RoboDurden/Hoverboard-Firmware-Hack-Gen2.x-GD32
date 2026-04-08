#include "../Inc/foc.h"

// Sector start angles (uint16_t: 0..65535 = 0..360 degrees electrical)
// pos 1 starts at 0°, each sector is 60°
// This table may need adjustment based on motor/hall alignment
const uint16_t sector_start_angle[7] = {
	0,            // [0] unused
	0,            // [1] pos 1: 0°
	ANGLE_60DEG,  // [2] pos 2: 60°
	ANGLE_60DEG * 2,  // [3] pos 3: 120°
	ANGLE_60DEG * 3,  // [4] pos 4: 180°
	ANGLE_60DEG * 4,  // [5] pos 5: 240°
	ANGLE_60DEG * 5,  // [6] pos 6: 300°
};

void foc_angle_init(FOC_Angle *a) {
	a->electrical_angle = 0;
	a->sector = 0;
	a->last_sector = 0;
	a->transition_tick = 0;
	a->sector_ticks = 1000;  // ~62ms at 16kHz, safe default (slow speed)
	a->tick = 0;
}

void foc_angle_update(FOC_Angle *a, uint8_t pos) {
	a->tick++;

	if (pos < 1 || pos > 6) return;

	// Detect hall transition
	if (pos != a->sector) {
		uint32_t now = a->tick;
		uint32_t elapsed = now - a->transition_tick;

		// Only update speed if the elapsed time is reasonable
		// (filter out glitches: minimum 2 ticks, maximum 10000 ticks ~625ms)
		if (elapsed >= 2 && elapsed <= 10000) {
			a->sector_ticks = elapsed;
		}

		a->last_sector = a->sector;
		a->sector = pos;
		a->transition_tick = now;
	}

	// Interpolate angle within current sector
	uint32_t ticks_in_sector = a->tick - a->transition_tick;

	// Fraction through current sector: ticks_in_sector / sector_ticks
	// Clamp to 1.0 (don't extrapolate beyond one sector)
	uint16_t frac;
	if (ticks_in_sector >= a->sector_ticks) {
		frac = ANGLE_60DEG;  // full sector
	} else {
		// frac = (ticks_in_sector * ANGLE_60DEG) / sector_ticks
		frac = (uint16_t)((uint32_t)ticks_in_sector * ANGLE_60DEG / a->sector_ticks);
	}

	a->electrical_angle = sector_start_angle[a->sector] + frac;
}

void foc_current_update(FOC_Current *c, uint16_t adc_y, uint16_t adc_b,
                        uint16_t offset_y, uint16_t offset_b) {
	c->iy = (int16_t)adc_y - (int16_t)offset_y;
	c->ib = (int16_t)adc_b - (int16_t)offset_b;
	c->ig = -(c->iy + c->ib);  // Kirchhoff: Iy + Ib + Ig = 0
}

// 256-entry Q15 sine table (one full period, 0..360°)
// sin_table[i] = sin(i * 2*PI / 256) * 32767
static const int16_t sin_table[256] = {
	     0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
	  6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
	 12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
	 18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
	 23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
	 27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
	 30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
	 32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
	 32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
	 32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
	 30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
	 27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
	 23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
	 18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
	 12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
	  6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
	     0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
	 -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
	-12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
	-18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
	-23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
	-27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
	-30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
	-32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
	-32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
	-32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
	-30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
	-27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
	-23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
	-18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
	-12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
	 -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804,
};

int16_t foc_sin(uint16_t angle) {
	// Map 16-bit angle to 8-bit table index
	return sin_table[angle >> 8];
}

int16_t foc_cos(uint16_t angle) {
	// cos(x) = sin(x + 90°) = sin(x + 16384)
	return sin_table[(uint8_t)((angle + 16384) >> 8)];
}

// Clarke transform: 3-phase (Y,B,G) → alpha-beta stationary frame
// Using yellow as phase A:
//   Iα = Iy
//   Iβ = (Iy + 2*Ib) / √3
// For integer math: 1/√3 ≈ 18919/32768 (Q15)
#define INV_SQRT3_Q15  18919

void foc_clarke(const FOC_Current *in, FOC_AlphaBeta *out) {
	out->alpha = in->iy;
	// beta = (iy + 2*ib) / sqrt(3)
	out->beta = (int16_t)(((int32_t)(in->iy + 2 * in->ib) * INV_SQRT3_Q15) >> 15);
}

// Park transform: alpha-beta → d-q rotating frame
//   Id =  Iα * cos(θ) + Iβ * sin(θ)
//   Iq = -Iα * sin(θ) + Iβ * cos(θ)
void foc_park(const FOC_AlphaBeta *in, FOC_DQ *out, uint16_t angle) {
	int16_t sin_val = foc_sin(angle);
	int16_t cos_val = foc_cos(angle);
	out->d = (int16_t)(((int32_t)in->alpha * cos_val + (int32_t)in->beta * sin_val) >> 15);
	out->q = (int16_t)(((int32_t)in->beta * cos_val - (int32_t)in->alpha * sin_val) >> 15);
}

// Inverse Park: d-q → alpha-beta
//   Vα = Vd * cos(θ) - Vq * sin(θ)
//   Vβ = Vd * sin(θ) + Vq * cos(θ)
void foc_inverse_park(const FOC_DQ *in, FOC_AlphaBeta *out, uint16_t angle) {
	int16_t sin_val = foc_sin(angle);
	int16_t cos_val = foc_cos(angle);
	out->alpha = (int16_t)(((int32_t)in->d * cos_val - (int32_t)in->q * sin_val) >> 15);
	out->beta  = (int16_t)(((int32_t)in->d * sin_val + (int32_t)in->q * cos_val) >> 15);
}

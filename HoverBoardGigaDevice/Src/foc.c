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

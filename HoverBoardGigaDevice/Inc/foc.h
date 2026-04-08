#ifndef FOC_H
#define FOC_H

#include <stdint.h>

// Electrical angle is represented as 0..65535 (uint16_t) mapping to 0..2*PI
// This avoids float and wraps naturally on overflow
#define ANGLE_60DEG   10922   // 65536 / 6
#define ANGLE_120DEG  21845
#define ANGLE_180DEG  32768
#define ANGLE_360DEG  65536   // wraps to 0

typedef struct {
	uint16_t electrical_angle;     // 0..65535 = 0..2*PI
	uint8_t  sector;               // current hall sector (1-6)
	uint8_t  last_sector;          // previous hall sector
	uint32_t transition_tick;      // ISR tick at last hall transition
	uint32_t sector_ticks;         // ticks for one sector (speed estimate)
	uint32_t tick;                 // running ISR tick counter
} FOC_Angle;

// Sector start angles: electrical angle at the leading edge of each sector (1-6)
// These depend on motor/hall alignment and may need calibration.
// Default assumes pos 1 starts at 0 degrees.
extern const uint16_t sector_start_angle[7];  // index 0 unused, 1-6

void foc_angle_init(FOC_Angle *a);
void foc_angle_update(FOC_Angle *a, uint8_t pos);

// Phase currents after offset removal
typedef struct {
	int16_t iy;   // yellow phase current (ADC - offset)
	int16_t ib;   // blue phase current
	int16_t ig;   // green phase (derived: -(iy + ib))
} FOC_Current;

void foc_current_update(FOC_Current *c, uint16_t adc_y, uint16_t adc_b,
                        uint16_t offset_y, uint16_t offset_b);

#endif

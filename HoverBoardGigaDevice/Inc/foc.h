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

// Alpha-beta (stationary frame) and d-q (rotating frame) currents
typedef struct {
	int16_t alpha;
	int16_t beta;
} FOC_AlphaBeta;

typedef struct {
	int16_t d;   // flux component (should be driven to 0)
	int16_t q;   // torque component
} FOC_DQ;

// Clarke transform: 3-phase → alpha-beta (stationary frame)
// Uses yellow as phase A reference
void foc_clarke(const FOC_Current *in, FOC_AlphaBeta *out);

// Park transform: alpha-beta → d-q (rotating frame)
void foc_park(const FOC_AlphaBeta *in, FOC_DQ *out, uint16_t angle);

// Inverse Park: d-q → alpha-beta
void foc_inverse_park(const FOC_DQ *in, FOC_AlphaBeta *out, uint16_t angle);

// Q15 sin/cos from lookup table. Input: uint16_t angle (0..65535)
// Output: Q15 value (-32768..32767 representing -1.0..+1.0)
int16_t foc_sin(uint16_t angle);
int16_t foc_cos(uint16_t angle);

#endif

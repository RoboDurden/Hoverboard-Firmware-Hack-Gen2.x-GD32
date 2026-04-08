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
	uint16_t electrical_angle;     // 0..65535 = 0..2*PI (includes offset)
	uint16_t angle_offset;         // hall-to-electrical angle offset (tune this!)
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

// PI controller
#define PI_INTEGRAL_SHIFT 10  // integrator accumulates at 1/1024 rate

typedef struct {
	int16_t kp;        // proportional gain
	int16_t ki;        // integral gain
	int32_t integral;  // accumulated integral (shifted)
	int16_t limit;     // symmetric output clamp
} FOC_PI;

void foc_pi_init(FOC_PI *pi, int16_t kp, int16_t ki, int16_t limit);
int16_t foc_pi_update(FOC_PI *pi, int16_t error);
void foc_pi_reset(FOC_PI *pi);

// Inverse Clarke: alpha-beta → 3-phase voltages (Y, B, G)
typedef struct {
	int16_t y;
	int16_t b;
	int16_t g;
} FOC_Phase;

void foc_inverse_clarke(const FOC_AlphaBeta *in, FOC_Phase *out);

// Full FOC state for the control loop
typedef struct {
	FOC_PI pi_d;       // flux controller (drives Id → 0)
	FOC_PI pi_q;       // torque controller (drives Iq → reference)
	int16_t iq_ref;    // torque reference (set from speed/pwm input)
	int16_t id_ref;    // flux reference (normally 0)
} FOC_Controller;

void foc_controller_init(FOC_Controller *ctrl);

// Calibrate current offsets by sampling ADC with motor off
// Call at startup before enabling motor. Blocks for ~200ms.
void foc_calibrate_offsets(uint16_t *offset_y, uint16_t *offset_b,
                           volatile uint16_t *adc_y, volatile uint16_t *adc_b);

// Align rotor to a known angle and determine the hall-to-electrical angle offset.
// Applies a voltage at 0° electrical, waits for rotor to settle, reads halls.
// Returns the angle_offset to use. Blocks for ~1s. Call at startup.
uint16_t foc_align_rotor(uint8_t *hall_to_pos_table);

// Run one FOC iteration: currents + angle → phase voltages
void foc_controller_update(FOC_Controller *ctrl,
                           const FOC_Current *current,
                           uint16_t angle,
                           FOC_Phase *voltage_out);

#endif

#include "../Inc/bldcFOC.h"

#ifdef BLDC_FOC

#if !defined(PHASE_CURRENT_A) || !defined(PHASE_CURRENT_B)
	#error "BLDC_FOC requires PHASE_CURRENT_A and PHASE_CURRENT_B. Add them to your board's Inc/defines/defines_X-Y-Z.h (see defines_2-1-20.h for an example)."
#endif

#include <stdio.h>
#ifdef RTT_REMOTE
#include "SEGGER_RTT.h"
#endif

// ── Internal types and state ──────────────────────────────────────────
// All FOC types and globals are file-local; only foc_sensor_update is
// exposed via bldcFOC.h.

// Electrical angle: 0..65535 maps to 0..2*PI (avoids float, wraps naturally)
#define ANGLE_60DEG   10922   // 65536 / 6
#define ANGLE_120DEG  21845
#define ANGLE_180DEG  32768
#define ANGLE_360DEG  65536   // wraps to 0

typedef struct {
	uint16_t electrical_angle;     // 0..65535 = 0..2*PI (includes offset)
	uint16_t angle_offset;         // hall-to-electrical angle offset
	uint8_t  sector;               // current hall sector (1-6)
	uint8_t  last_sector;          // previous hall sector
	int8_t   direction;            // +1 = forward, -1 = reverse
	uint32_t transition_tick;      // ISR tick at last hall transition
	uint32_t sector_ticks;         // ticks for one sector (speed estimate)
	uint32_t tick;                 // running ISR tick counter
	int32_t  pll_angle;            // Q16: top 16 bits = uint16_t electrical angle
	int32_t  pll_velocity;         // Q16: angle units per ISR tick
	uint32_t sector_tick_sum[6];   // per-sector calibration sums
	uint16_t sector_tick_cnt[6];
} FOC_Angle;

typedef struct {
	int16_t ia, ib, ic;            // phase A/B currents (ic derived: -(ia+ib))
} FOC_Current;

typedef struct {
	int16_t alpha, beta;
} FOC_AlphaBeta;

typedef struct {
	int16_t d;   // flux component (driven to 0)
	int16_t q;   // torque component
} FOC_DQ;

typedef struct {
	int16_t a, b, c;
} FOC_Phase;

#define PI_INTEGRAL_SHIFT 10  // integrator accumulates at 1/1024 rate

typedef struct {
	int16_t kp, ki;
	int32_t integral;
	int16_t limit;
} FOC_PI;

typedef struct {
	FOC_PI pi_d, pi_q;
	int16_t iq_ref, id_ref;
} FOC_Controller;

// Back-EMF observer: uses Id as angle error signal (in open-loop voltage FOC,
// Id is non-zero only when the assumed angle is misaligned with the rotor).
typedef struct {
	int32_t angle;     // Q16
	int32_t velocity;  // Q16: angle units per ISR tick
	int8_t  sign;      // sign of Id→angle correction
} FOC_Observer;

static FOC_Angle foc_angle;
static FOC_Current foc_current;
static FOC_AlphaBeta foc_ab;
static FOC_DQ foc_dq;
static FOC_Controller foc_ctrl;
static FOC_Observer foc_obs;
static uint16_t foc_offset_a = 2000;  // calibrated at startup
static uint16_t foc_offset_b = 2000;

// ISR-rate averaging for telemetry
static int32_t foc_id_sum = 0, foc_iq_sum = 0;
static int32_t foc_ia_sum = 0, foc_ib_sum = 0;
static uint16_t foc_avg_count = 0;
static int16_t foc_id_avg = 0, foc_iq_avg = 0;
static int16_t foc_ia_avg = 0, foc_ib_avg = 0;

// Runtime mode: 0=block commutation, 1=FOC. Updated by bldc_get_pwm().
static uint8_t foc_mode = 0;

// Get observer angle as uint16_t
static inline uint16_t foc_observer_angle(const FOC_Observer *obs) {
	return (uint16_t)(obs->angle >> 16);
}

// Sector start angles (default: 6 evenly-spaced 60° sectors).
// For better accuracy on a specific motor, calibrate per-sector widths
// using foc_angle->sector_tick_sum/cnt at constant speed and replace
// these uniform values with measured ratios.
const uint16_t sector_start_angle[7] = {
	0,                // [0] unused
	0,                // [1] s1: 0°
	ANGLE_60DEG,      // [2] s2: 60°
	ANGLE_60DEG * 2,  // [3] s3: 120°
	ANGLE_60DEG * 3,  // [4] s4: 180°
	ANGLE_60DEG * 4,  // [5] s5: 240°
	ANGLE_60DEG * 5,  // [6] s6: 300°
};

// Sector widths — uniform 60° each by default
static const uint16_t sector_width[7] = {
	0,
	ANGLE_60DEG,
	ANGLE_60DEG,
	ANGLE_60DEG,
	ANGLE_60DEG,
	ANGLE_60DEG,
	ANGLE_60DEG,
};

void foc_angle_init(FOC_Angle *a) {
	a->electrical_angle = 0;
	// Angle offset depends on hall sensor placement (motor design, not per-unit).
	// For this hoverboard sensorboard: 27307 = 150° (verified by bench tuning).
	// For other motors, calibrate via foc_align_rotor() or joystick trim.
	a->angle_offset = 27307;
	a->sector = 0;
	a->last_sector = 0;
	a->direction = 1;
	a->transition_tick = 0;
	a->sector_ticks = 1000;  // ~62ms at 16kHz, safe default (slow speed)
	a->tick = 0;
	a->pll_angle = 0;
	a->pll_velocity = 0;
	for (int i = 0; i < 6; i++) {
		a->sector_tick_sum[i] = 0;
		a->sector_tick_cnt[i] = 0;
	}
}

// Determine direction from sector transition
// Forward: 1→2→3→4→5→6→1, Reverse: 1→6→5→4→3→2→1
static int8_t sector_direction(uint8_t from, uint8_t to) {
	if (from == 0 || to == 0) return 1;
	int8_t diff = (int8_t)to - (int8_t)from;
	// Handle wraparound: 6→1 is forward (+1), 1→6 is reverse (-1)
	if (diff == 1 || diff == -5) return 1;
	if (diff == -1 || diff == 5) return -1;
	return 1;  // skip or glitch, keep current direction
}

void foc_angle_update(FOC_Angle *a, uint8_t pos) {
	a->tick++;

	if (pos < 1 || pos > 6) return;

	// PLL: advance angle by velocity each ISR cycle
	a->pll_angle += a->pll_velocity;

	// Detect hall transition
	if (pos != a->sector) {
		uint32_t now = a->tick;
		uint32_t elapsed = now - a->transition_tick;

		// Only update speed if the elapsed time is reasonable
		if (elapsed >= 2 && elapsed <= 10000) {
			a->sector_ticks = elapsed;

			// Calibration: accumulate elapsed time for the sector we just LEFT
			if (a->sector >= 1 && a->sector <= 6) {
				uint8_t s = a->sector - 1;
				if (a->sector_tick_cnt[s] < 60000) {
					a->sector_tick_sum[s] += elapsed;
					a->sector_tick_cnt[s]++;
				}
			}
		}

		// Detect rotation direction
		a->direction = sector_direction(a->sector, pos);

		a->last_sector = a->sector;
		a->sector = pos;
		a->transition_tick = now;

		// PLL correction at hall transition: rotor is at the leading edge of the new sector
		uint16_t measured;
		if (a->direction >= 0) {
			measured = sector_start_angle[a->sector];
		} else {
			measured = sector_start_angle[a->sector] + sector_width[a->sector];
		}

		// Compute error in 16-bit signed wrap (handles 0/65535 correctly)
		int16_t pll_angle_16 = (int16_t)(a->pll_angle >> 16);
		int16_t error_16 = (int16_t)((uint16_t)measured - (uint16_t)pll_angle_16);
		int32_t error_32 = (int32_t)error_16 << 16;

		// PI correction: Kp = 1/2 (position, faster so FOC doesn't "kick"
		// from stale PLL state when mode changes), Ki = 1/64 (velocity,
		// also faster for better tracking during BC so FOC engages clean)
		a->pll_angle += error_32 >> 1;
		a->pll_velocity += error_32 >> 6;
	}

	// Output angle = PLL angle + offset
	a->electrical_angle = (uint16_t)(a->pll_angle >> 16) + a->angle_offset;
}

void foc_current_update(FOC_Current *c, uint16_t adc_a, uint16_t adc_b,
                        uint16_t offset_a, uint16_t offset_b) {
	c->ia = (int16_t)adc_a - (int16_t)offset_a;
	c->ib = (int16_t)adc_b - (int16_t)offset_b;
	c->ic = -(c->ia + c->ib);
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
	// cos(x) = sin(x + 90°); table index offset of 64 = 90°
	return sin_table[(uint8_t)((angle >> 8) + 64)];
}

// Clarke transform: 3-phase → alpha-beta stationary frame
//   Iα = Ia
//   Iβ = (Ia + 2*Ib) / √3
// For integer math: 1/√3 ≈ 18919/32768 (Q15)
#define INV_SQRT3_Q15  18919

void foc_clarke(const FOC_Current *in, FOC_AlphaBeta *out) {
	out->alpha = in->ia;
	// beta = (ia + 2*ib) / sqrt(3)
	out->beta = (int16_t)(((int32_t)(in->ia + 2 * in->ib) * INV_SQRT3_Q15) >> 15);
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

// Back-EMF observer: uses Id as angle error signal
// (d-axis current is non-zero when assumed angle ≠ rotor angle in open-loop FOC)
void foc_observer_init(FOC_Observer *obs) {
	obs->angle = 0;
	obs->velocity = 0;
	obs->sign = 1;  // start positive, flip if observer diverges
}

void foc_observer_set(FOC_Observer *obs, uint16_t angle, int32_t velocity) {
	obs->angle = (int32_t)angle << 16;
	obs->velocity = velocity;
}

// Observer update at ISR rate.
// Each call advances angle by velocity, then nudges based on Id error.
// Very conservative gains — observer needs many cycles to converge.
void foc_observer_update(FOC_Observer *obs, int16_t id) {
	// Always advance angle by current velocity estimate
	obs->angle += obs->velocity;

	// Use Id (raw ADC counts) as the error signal.
	int32_t error = (int32_t)id * obs->sign;

	// Kp: position correction — much smaller than before (was 1, now 1/256)
	// In Q16 units: error * 256 = nudge angle by 256 units per ADC count of Id
	obs->angle += error * 256;
	// Ki: velocity correction (slower)
	obs->velocity += error;
}

// PI controller
void foc_pi_init(FOC_PI *pi, int16_t kp, int16_t ki, int16_t limit) {
	pi->kp = kp;
	pi->ki = ki;
	pi->integral = 0;
	pi->limit = limit;
}

void foc_pi_reset(FOC_PI *pi) {
	pi->integral = 0;
}

// Q11 fixed-point gains (matching reference FOC project)
// Effective gain = stored_value / 2048
// Reference uses kp=1229 → ~0.6, ki=1229 → ~0.6 (Iq loop)
//                kp=819  → ~0.4, ki=737  → ~0.36 (Id loop)
#define PI_KP_SHIFT 11

int16_t foc_pi_update(FOC_PI *pi, int16_t error) {
	// Accumulate integral (Q11 scaled by additional PI_INTEGRAL_SHIFT for precision)
	pi->integral += (int32_t)error * pi->ki;

	// Anti-windup: clamp integral
	int32_t int_limit = (int32_t)pi->limit << (PI_INTEGRAL_SHIFT + PI_KP_SHIFT);
	if (pi->integral > int_limit) pi->integral = int_limit;
	if (pi->integral < -int_limit) pi->integral = -int_limit;

	// Compute output: P + I, with Q11 gains
	int32_t output = ((int32_t)error * pi->kp + (pi->integral >> PI_INTEGRAL_SHIFT)) >> PI_KP_SHIFT;

	// Clamp output
	if (output > pi->limit) return pi->limit;
	if (output < -pi->limit) return -pi->limit;
	return (int16_t)output;
}

// Inverse Clarke: alpha-beta → 3-phase
// Va = Vα
// Vb = (-Vα + √3*Vβ) / 2
// Vc = (-Vα - √3*Vβ) / 2
// √3/2 in Q15 = 28378
#define SQRT3_OVER_2_Q15 28378

void foc_inverse_clarke(const FOC_AlphaBeta *in, FOC_Phase *out) {
	out->a = in->alpha;
	int32_t sqrt3_beta = ((int32_t)in->beta * SQRT3_OVER_2_Q15) >> 15;
	out->b = (int16_t)(-in->alpha / 2 + sqrt3_beta);
	out->c = (int16_t)(-in->alpha / 2 - sqrt3_beta);
}

// Calibrate current sensor offsets (zero-current ADC values)
// Samples ADC 1000 times over ~200ms with motor off
void foc_calibrate_offsets(uint16_t *offset_a, uint16_t *offset_b,
                           volatile uint16_t *adc_a, volatile uint16_t *adc_b) {
	uint32_t sum_a = 0, sum_b = 0;
	for (int i = 0; i < 1000; i++) {
		sum_a += *adc_a;
		sum_b += *adc_b;
		// Small delay — the DMA refreshes ADC values continuously
		for (volatile int d = 0; d < 500; d++);
	}
	*offset_a = (uint16_t)(sum_a / 1000);
	*offset_b = (uint16_t)(sum_b / 1000);
}

// Align rotor to determine the electrical angle offset.
// Applies a d-axis voltage at 0° to lock the rotor, then reads the hall position.
#define ALIGN_VOLTAGE 150  // PWM counts (~13% of mid value) — keep current low
#define ALIGN_SETTLE_MS 500

uint16_t foc_align_rotor(uint8_t *hall_to_pos_table) {
	uint16_t mid = BLDC_TIMER_PERIOD / 2;

	// Disable BLDC ISR so it doesn't overwrite our PWM
	nvic_irq_disable(DMA_Channel0_IRQn);
	timer_automatic_output_enable(TIMER_BLDC);

	// Apply voltage at 0° electrical:
	// Inverse Park at θ=0: Vα = Vd, Vβ = 0
	// Inverse Clarke: Vy = Vα = ALIGN_VOLTAGE
	//                 Vb = (-Vα) / 2 = -ALIGN_VOLTAGE/2
	//                 Vg = (-Vα) / 2 = -ALIGN_VOLTAGE/2
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, mid + ALIGN_VOLTAGE);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, mid - ALIGN_VOLTAGE / 2);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, mid - ALIGN_VOLTAGE / 2);

	// Wait for rotor to settle
	for (int i = 0; i < ALIGN_SETTLE_MS; i++) {
		Delay(1);
		fwdgt_counter_reload();
		ResetTimeout();
	}

	// Read hall sensors
	uint8_t ha = digitalRead(HALL_A);
	uint8_t hb = digitalRead(HALL_B);
	uint8_t hc = digitalRead(HALL_C);
	uint8_t hall = ha * 1 + hb * 2 + hc * 4;
	uint8_t sector = hall_to_pos_table[hall];

	// Turn off outputs
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_Y, mid);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_B, mid);
	timer_channel_output_pulse_value_config(TIMER_BLDC, TIMER_BLDC_CHANNEL_G, mid);
	timer_automatic_output_disable(TIMER_BLDC);

	// Re-enable BLDC ISR
	nvic_irq_enable(DMA_Channel0_IRQn, 1, 0);

	// The rotor has aligned to 0° electrical.
	// The halls report sector s, whose center is sector_start_angle[s] + 30°.
	// offset = applied_angle - measured_angle = 0 - (sector_start + 30°)
	if (sector >= 1 && sector <= 6) {
		uint16_t measured = sector_start_angle[sector] + ANGLE_60DEG / 2;  // center of sector
		return (uint16_t)(0 - measured);  // wraps naturally
	}
	return 0;  // invalid hall state, no offset
}

// FOC controller
void foc_controller_init(FOC_Controller *ctrl) {
	// Very conservative starting gains — tune upward from here
	// Kp=2: 2 PWM counts per ADC count of current error
	// Ki=1: slow integral, shifted by PI_INTEGRAL_SHIFT (1/1024)
	// Limit: ±500 (well below max of ±1125)
	// Q11 gains from hoverboard-firmware-hack-FOC reference project
	foc_pi_init(&ctrl->pi_d, 819,  737,  400);  // flux: Kp~0.4, Ki~0.36
	foc_pi_init(&ctrl->pi_q, 1229, 1229, 400);  // torque: Kp~0.6, Ki~0.6
	ctrl->iq_ref = 0;
	ctrl->id_ref = 0;
}

void foc_controller_update(FOC_Controller *ctrl,
                           const FOC_Current *current,
                           uint16_t angle,
                           FOC_Phase *voltage_out) {
	// Clarke: 3-phase → alpha-beta
	FOC_AlphaBeta ab;
	foc_clarke(current, &ab);

	// Park: alpha-beta → d-q
	FOC_DQ dq;
	foc_park(&ab, &dq, angle);

	// PI controllers
	FOC_DQ vdq;
	vdq.d = foc_pi_update(&ctrl->pi_d, ctrl->id_ref - dq.d);
	vdq.q = foc_pi_update(&ctrl->pi_q, ctrl->iq_ref - dq.q);

	// Inverse Park: d-q → alpha-beta
	FOC_AlphaBeta vab;
	foc_inverse_park(&vdq, &vab, angle);

	// Inverse Clarke: alpha-beta → 3-phase voltages
	foc_inverse_clarke(&vab, voltage_out);
}

// ── Per-ISR sensor update ──────────────────────────────────────────────
// Pulls in phase currents, runs Clarke/Park, updates observer, and
// accumulates averages used for RTT telemetry.
void foc_sensor_update(uint8_t pos, volatile adc_buf_t *adc) {
	foc_angle_update(&foc_angle, pos);

	// Ib offset compensation: startup calibration consistently reads
	// ~33 counts too high on PB1 (persistent across sessions). Correcting
	// here keeps the DC bias out of the Clarke/Park transforms.
	foc_current_update(&foc_current, adc->phase_current_a, adc->phase_current_b,
	                   foc_offset_a, foc_offset_b + 33);
	foc_clarke(&foc_current, &foc_ab);
	foc_park(&foc_ab, &foc_dq, foc_angle.electrical_angle);

	// Back-EMF observer: seed from halls once motor is spinning, then let it run free.
	// Re-seed only if observer is way off from hall (catches drift).
	static uint8_t obs_seeded = 0;
	int16_t obs_hall_diff = (int16_t)(foc_angle.electrical_angle - foc_observer_angle(&foc_obs));
	if (!obs_seeded || obs_hall_diff > 5461 || obs_hall_diff < -5461) {  // > 30°
		if (foc_angle.sector_ticks > 0 && foc_angle.sector_ticks < 5000) {
			// hall velocity = ANGLE_60DEG (10923) per sector_ticks ISR cycles, in Q16
			int32_t hall_velocity = ((int32_t)10923 << 16) / (int32_t)foc_angle.sector_ticks;
			if (foc_angle.direction < 0) hall_velocity = -hall_velocity;
			foc_observer_set(&foc_obs, foc_angle.electrical_angle, hall_velocity);
			obs_seeded = 1;
		}
	}
	foc_observer_update(&foc_obs, foc_dq.d);

	// Accumulate for ISR-rate averaging (compute avg every 1000 cycles = ~62ms)
	foc_id_sum += foc_dq.d;
	foc_iq_sum += foc_dq.q;
	foc_ia_sum += foc_current.ia;
	foc_ib_sum += foc_current.ib;
	foc_avg_count++;
	if (foc_avg_count >= 1000) {
		foc_id_avg = foc_id_sum / 1000;
		foc_iq_avg = foc_iq_sum / 1000;
		foc_ia_avg = foc_ia_sum / 1000;
		foc_ib_avg = foc_ib_sum / 1000;
		foc_id_sum = foc_iq_sum = foc_ia_sum = foc_ib_sum = 0;
		foc_avg_count = 0;
	}
}

// ── Periodic RTT telemetry ────────────────────────────────────────────
static void foc_log_rtt(void) {
	#ifdef RTT_REMOTE
		static uint16_t rtt_log_count = 0;
		if (++rtt_log_count < 3000) return;
		rtt_log_count = 0;

		extern int32_t steer;
		extern uint8_t wState;
		uint16_t off_deg = (uint32_t)foc_angle.angle_offset * 360 / 65536;

		char s[120];
		sprintf(s, "wS:0x%02X foc:%d m:%d dir:%+d Id:%+4d Iq:%+4d stk:%u off:%3u str:%+5ld\r\n",
			wState, (wState & 1) ? 1 : 0, foc_mode,
			(int)foc_angle.direction,
			foc_id_avg, foc_iq_avg,
			(unsigned)foc_angle.sector_ticks, off_deg, (long)steer);
		SEGGER_RTT_WriteString(0, s);
	#endif
}

// ── Block commutation ↔ open-loop voltage FOC state machine ───────────

// Thresholds in sector_ticks (ISR cycles per hall sector).
// Lower ticks = higher RPM. At 16 kHz with 6 sectors × 15 pole pairs:
//   RPM = 60 × 16000 / (sector_ticks × 90) = 10666 / sector_ticks
#define FOC_BLDC_ENGAGE_TICKS  200    // engage FOC above ~53 RPM
#define FOC_BLDC_DISENGAGE_TICKS 400  // fall back to block below ~27 RPM
#define FOC_BLDC_WARMUP_TICKS  48000  // 3 s block commutation after startup

// FOC angle offset (empirically tuned post-Clarke fix). Base is 161°.
// Trim adjusts ±180° around it for tuning.
#define FOC_ANGLE_OFFSET_BASE  29305  // 161° in Q16

// 6-step block commutation table (one phase at +pwm, one at -pwm, one floating)
static void foc_block_pwm(int16_t pwm, uint8_t pos, int *y, int *b, int *g)
{
	switch (pos) {
		case 1: *y = 0;    *b = pwm;  *g = -pwm; break;
		case 2: *y = -pwm; *b = pwm;  *g = 0;    break;
		case 3: *y = -pwm; *b = 0;    *g = pwm;  break;
		case 4: *y = 0;    *b = -pwm; *g = pwm;  break;
		case 5: *y = pwm;  *b = -pwm; *g = 0;    break;
		case 6: *y = pwm;  *b = 0;    *g = -pwm; break;
		default: *y = 0;   *b = 0;    *g = 0;    break;
	}
}

extern const uint16_t sector_start_angle[7];

static uint8_t foc_bldc_step(uint8_t pos, int16_t pwm_cmd, int32_t trim,
                             uint8_t foc_enable,
                             int *y, int *b, int *g)
{
	static uint8_t was_foc_enabled = 0;
	if (!foc_enable) {
		was_foc_enabled = 0;
		foc_block_pwm(pwm_cmd, pos, y, b, g);
		return 0;
	}

	// Hard reset PLL on BC → FOC transition. Testing whether this
	// eliminates the "stuck state" where FOC fails to engage after
	// certain BC sequences. The "kick" is the cost.
	if (!was_foc_enabled && pos >= 1 && pos <= 6) {
		uint16_t centre = sector_start_angle[pos] + (ANGLE_60DEG / 2);
		foc_angle.pll_angle = (int32_t)centre << 16;
		foc_angle.pll_velocity = 0;
		was_foc_enabled = 1;
	}

	// Open-loop voltage FOC. Trim adjusts angle offset around 161° with
	// ±60° range — covers field weakening exploration without being
	// overly sensitive.
	foc_angle.angle_offset =
		(uint16_t)(FOC_ANGLE_OFFSET_BASE + (trim * 109) / 10);

	FOC_DQ vdq;
	vdq.d = 0;
	vdq.q = -pwm_cmd;

	FOC_AlphaBeta vab;
	foc_inverse_park(&vdq, &vab, foc_angle.electrical_angle);

	FOC_Phase voltage;
	foc_inverse_clarke(&vab, &voltage);

	// SVPWM centering
	int16_t vmax = voltage.a;
	if (voltage.b > vmax) vmax = voltage.b;
	if (voltage.c > vmax) vmax = voltage.c;
	int16_t vmin = voltage.a;
	if (voltage.b < vmin) vmin = voltage.b;
	if (voltage.c < vmin) vmin = voltage.c;
	int16_t v_offset = -(vmax + vmin) / 2;

	*y = voltage.a + v_offset;
	*b = voltage.b + v_offset;
	*g = voltage.c + v_offset;
	return 1;
}

// FOC owns bldc_get_pwm() and InitBldc(); bldcBC.c and bldcSINE.c are excluded.
// FOC on/off is driven by wStateMaster bit 0 from the joystick controller
// (see joystick's -f option). Block commutation runs when FOC is off.
void bldc_get_pwm(int pwm, int pos, int *y, int *b, int *g) {
	extern uint8_t wState;
	extern int32_t steer;
	foc_mode = foc_bldc_step((uint8_t)pos, (int16_t)pwm, steer,
	                         (wState & 0x01) ? 1 : 0,
	                         y, b, g);
	foc_log_rtt();
}

void InitBldc(void) {
	extern adc_buf_t adc_buffer;

	foc_angle_init(&foc_angle);
	foc_controller_init(&foc_ctrl);
	foc_observer_init(&foc_obs);

	foc_calibrate_offsets(&foc_offset_a, &foc_offset_b,
	                      &adc_buffer.phase_current_a, &adc_buffer.phase_current_b);
}

#endif // BLDC_FOC

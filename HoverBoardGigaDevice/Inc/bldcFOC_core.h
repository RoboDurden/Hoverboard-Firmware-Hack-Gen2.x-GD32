#ifndef BLDCFOC_CORE_H
#define BLDCFOC_CORE_H

#include <stdint.h>

/* ============ Q15 constants ============ */
#define Q15_INV_SQRT3      18919    /* 1/sqrt(3) * 2^15 */
#define Q15_TWO_OVER_SQRT3 37837    /* 2/sqrt(3) * 2^15, doesn't fit int16 */
#define Q15_HALF           16384    /* 0.5 * 2^15 */
#define Q15_SQRT3_OVER_2   28378    /* sqrt(3)/2 * 2^15 */

/* ============ FOC tunables ============ */
#define FOC_VQ_MAX             20000
#define FOC_ANGLE_OFFSET_BASE  156
#define FOC_OUT_SHIFT          4
#define FOC_IQ_MAX             500
#define FOC_RTT_LOG_DIV        160
#define FOC_AVG_SHIFT          10

/* PLL */
#define FOC_PLL_VEL_CLAMP          3000
#define FOC_PLL_NOMINAL_WIDTH_Q8   15360   /* 60 deg << 8 */
#define FOC_PLL_ANCHOR_Q8          38400   /* (sector_centre_deg[0]=180 << 8) - FOC_PLL_NOMINAL_WIDTH_Q8/2 */

/* PI gains (Phase 2 uses these; Phase 1 declares pi_step only) */
#define FOC_IQ_KP_SHIFT 0
#define FOC_IQ_KI_SHIFT 10
#define FOC_ID_KP_SHIFT 0
#define FOC_ID_KI_SHIFT 10
#define FOC_AW_SHIFT    2

/* DTC — FOC_DTC_MAX mirrors (DEAD_TIME / 2) from Inc/defines.h.
 * Kept as a literal here so bldcFOC_core.c is host-compilable without
 * pulling the MCU headers. bldcFOC.c asserts DEAD_TIME == 60 at target
 * compile. */
#define FOC_DTC_MAX   30
#define FOC_DTC_SHIFT 2

/* Hall learner */
#define FOC_HALL_LEARN_MIN_VEL      144
#define FOC_HALL_LEARN_WINDOW_REVS  10
#define FOC_HALL_LEARN_EMA_SHIFT    3
#define FOC_HALL_LEARN_PWM_TOL      50
#define FOC_HALL_LEARN_VEL_TOL      64

/* Steer trim */
#define FOC_STEER_LPF_SHIFT         10
#define FOC_STEER_TRIM_RANGE_DEG    30
#define FOC_STEER_TRIM_Q8_PER_COUNT ((FOC_STEER_TRIM_RANGE_DEG * 256 + 500) / 1000)

/* Calibration */
#define FOC_CALIB_DISCARD 32
#define FOC_CALIB_SAMPLES 1024

/* ============ Frame types ============ */
typedef struct { int16_t iy,    ib;   } iph_t;     /* phase-frame currents (yellow, blue), ADC counts */
typedef struct { int16_t alpha, beta; } ab_i_t;    /* stator-frame currents, ADC counts */
typedef struct { int16_t alpha, beta; } ab_v_t;    /* stator-frame voltages, internal units */
typedef struct { int16_t alpha, beta; } ab_pwm_t;  /* stator-frame voltages, PWM-delta units */
typedef struct { int16_t d,     q;    } dq_i_t;    /* rotor-frame currents */
typedef struct { int16_t d,     q;    } dq_v_t;    /* rotor-frame voltages */

typedef struct {
    iph_t   i_phase;
    ab_i_t  i_stator;
    dq_i_t  i_rotor;
    int16_t theta;
    uint8_t mode_req;
} foc_state_t;

typedef struct {
    int8_t kp_shift;
    int8_t ki_shift;
    int8_t aw_shift;
} pi_gains_t;

typedef struct {
    int32_t integrator_raw;
} pi_state_t;

typedef struct {
    int32_t  angle_accum_q8;
    int16_t  velocity_q8;
    int8_t   prev_pos;
    uint16_t sector_dwell;
} pll_state_t;

typedef enum {
    PLL_EDGE_VALID  = 0,
    PLL_EDGE_SEED   = 1,
    PLL_EDGE_GLITCH = 2
} pll_edge_kind_t;

typedef struct {
    uint16_t        consumed_dwell;
    int8_t          consumed_sector;
    pll_edge_kind_t kind;
} pll_edge_t;

/* ============ Inline helpers ============ */

extern const int16_t sine_q15_half[180];

static inline int16_t wrap_angle(int16_t a) {
    while (a >= 360) a -= 360;
    while (a < 0)    a += 360;
    return a;
}

static inline int16_t sin_q15(int16_t a) {
    a = wrap_angle(a);
    return (a < 180) ? sine_q15_half[a]
                     : (int16_t)(-sine_q15_half[a - 180]);
}

static inline int16_t cos_q15(int16_t a) { return sin_q15((int16_t)(a + 90)); }

/* signed_shift: sign-aware rounding shift used by pi_step's Kp/Ki/AW paths.
 * Right-shift rounds half-away-from-zero to avoid ASR's negative bias on
 * symmetric errors — exact multiples return exactly, halves round outward.
 * Left-shift is plain. */
static inline int32_t signed_shift(int32_t v, int8_t s) {
    if (s <= 0) return v << (-s);
    int32_t half = (int32_t)1 << (s - 1);
    return (v >= 0) ?  ((v + half) >> s)
                    : -(((-v) + half) >> s);
}

/* One-pole IIR. state is int32_t; equilibrium for constant sample is
 * state == sample << shift. Time constant ~= 2^shift samples. */
static inline int32_t ema_step(int32_t state, int16_t sample, uint8_t shift) {
    return state + (int32_t)sample - (state >> shift);
}

static inline int16_t steer_to_trim_q8(int16_t s) {
    return (int16_t)((int32_t)s * FOC_STEER_TRIM_Q8_PER_COUNT);
}

/* Soft-limiter slope for dead-time compensation. Returns PWM-delta
 * correction that saturates at +/-FOC_DTC_MAX. */
static inline int16_t dtc_term(int16_t i_counts) {
    int32_t t = (int32_t)i_counts >> FOC_DTC_SHIFT;
    if (t >  FOC_DTC_MAX) t =  FOC_DTC_MAX;
    if (t < -FOC_DTC_MAX) t = -FOC_DTC_MAX;
    return (int16_t)t;
}

static inline ab_pwm_t ab_scale_for_pwm(ab_v_t v) {
    ab_pwm_t out;
    out.alpha = (int16_t)(v.alpha >> FOC_OUT_SHIFT);
    out.beta  = (int16_t)(v.beta  >> FOC_OUT_SHIFT);
    return out;
}

/* ============ Out-of-line helpers ============ */

ab_i_t clarke(iph_t i);
dq_i_t park(ab_i_t i, int16_t theta_deg);
ab_v_t ipark(dq_v_t v, int16_t theta_deg);
void   iclarke_svpwm(ab_pwm_t v, int16_t *y, int16_t *b, int16_t *g);
void   dtc_apply(iph_t i, int16_t *y, int16_t *b, int16_t *g);

void       pll_advance(pll_state_t *state);
pll_edge_t pll_edge   (pll_state_t *state, int8_t pos, const uint16_t *sector_width_q8);

void hall_widths_update(uint16_t widths[6],
                        const uint32_t learn_sum[6],
                        uint8_t ema_shift);

void get_pwm_bc(int pwm, int pos, int16_t *y, int16_t *b, int16_t *g);

/* Phase 2 — declaration only; body lands with TRQ mode. */
int16_t pi_step(pi_state_t *state, int16_t err, int16_t clamp, pi_gains_t gains);

#endif /* BLDCFOC_CORE_H */

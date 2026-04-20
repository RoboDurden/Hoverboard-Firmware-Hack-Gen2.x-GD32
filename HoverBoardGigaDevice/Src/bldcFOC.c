#include "../Inc/defines.h"
#include "../Inc/bldc.h"
#include "../Inc/bldcFOC.h"
#include "../Inc/bldcFOC_core.h"

#ifdef BLDC_FOC

/* FOC_DTC_MAX in bldcFOC_core.h is a literal mirror of DEAD_TIME/2.
 * If DEAD_TIME ever moves off 60, the mirror needs to follow. */
#if DEAD_TIME != 60
#error "FOC_DTC_MAX in bldcFOC_core.h mirrors DEAD_TIME=60; update mirror if DEAD_TIME changes"
#endif

#ifdef RTT_REMOTE
#include <stdio.h>
#endif

/* ============ Project globals ============ */
extern uint8_t          wState;
extern volatile uint8_t hall;   /* unused here; decoded → pos inside bldc.c */
extern adc_buf_t        adc_buffer;

#ifdef MASTER_OR_SINGLE
extern int32_t steer;           /* live joystick axis; serves as FOC θ trim */
#else
static const int32_t steer = 0; /* slave builds: trim stays at zero */
#endif

/* ============ File-local state ============ */

/* Hall PLL + angle accumulator. */
static pll_state_t pll;

#if defined(PHASE_CURRENT_A) && defined(PHASE_CURRENT_B)
static int16_t  offset_A, offset_B;
static int32_t  sum_A, sum_B;
static uint16_t sample_count;
#endif

/* calib_done is NOT #if-gated — dispatch reads it every build. Sensorless
 * forces it to 1 in InitBldc so the dispatch gate passes forever. */
static uint8_t calib_done;

/* EMAs of Id/Iq for display. int32_t because sample<<FOC_AVG_SHIFT overflows int16. */
static int32_t i_d_ema, i_q_ema;

/* Mode tracking for PI integrator reset on transition. */
static uint8_t prev_mode;

/* PI integrators (Phase 2 populates pi_step body; kept here for symmetry with
 * the declared state boundary). */
static pi_state_t iq_pi, id_pi;
static volatile int16_t foc_trq_v_max = 3000;

/* Diagnostics + bench overrides. */
static int16_t           adc_raw_a, adc_raw_b;
static int16_t           vq_cmd;
static volatile uint8_t  pos_force = 0;

/* Hall width table + learner window state. */
static volatile uint16_t sector_width_q8[6] = {
    FOC_PLL_NOMINAL_WIDTH_Q8, FOC_PLL_NOMINAL_WIDTH_Q8, FOC_PLL_NOMINAL_WIDTH_Q8,
    FOC_PLL_NOMINAL_WIDTH_Q8, FOC_PLL_NOMINAL_WIDTH_Q8, FOC_PLL_NOMINAL_WIDTH_Q8
};
static uint32_t learn_sum[6];
static uint16_t learn_revs;
static int16_t  learn_pwm_ref, learn_vel_ref;
static volatile uint8_t foc_hall_learn_en = 1;

/* DTC gate — flipped via OpenOCD after VLT alignment is dialled in. */
static volatile uint8_t foc_dtc_en = 0;

/* Steer trim LPF accumulator. Pre-warmed in InitBldc. */
static int32_t steer_ema;

#ifdef RTT_REMOTE
typedef struct {
    int8_t   mode;
    int16_t  adc_raw_a, adc_raw_b;
    int16_t  iy, ib, ig;
    int16_t  i_alpha, i_beta;
    int16_t  i_d, i_q;
    int16_t  i_d_avg, i_q_avg;
    int16_t  theta;
    int16_t  rpm;
    int16_t  vq_cmd;
    int8_t   pos;
    uint8_t  pos_force;
    int16_t  steer_trim_q8;
} log_snapshot_t;

static log_snapshot_t   log_snapshot;    /* plain — log_pending orders access */
static volatile uint8_t log_pending;
static uint16_t         log_counter;
#endif

/* Silence "defined but not used" in slave builds where iq_pi/id_pi/foc_trq_v_max
 * haven't been wired into any dispatch path yet — Phase 2 lands TRQ. */
static inline void foc_phase1_silence_unused(void) {
    (void)iq_pi; (void)id_pi; (void)foc_trq_v_max;
}

/* ============ InitBldc ============ */
void InitBldc(void) {
    prev_mode    = 0xFF;   /* sentinel: first real dispatch forces integrator reset */
    pll.prev_pos = 0;      /* pll_edge seeds on the first non-zero pos */

#if !(defined(PHASE_CURRENT_A) && defined(PHASE_CURRENT_B))
    calib_done = 1;        /* sensorless: nothing to calibrate */
#endif

    /* Pre-warm steer LPF so its 64 ms time constant applies from ISR #1,
     * regardless of stick-at-boot position. */
    steer_ema = (int32_t)steer << FOC_STEER_LPF_SHIFT;

    foc_phase1_silence_unused();
}

/* ============ Sense half — foc_sample ============ */
static foc_state_t foc_sample(int pwm, int pos) {
    foc_state_t st = {
        .i_phase  = { 0, 0 },
        .i_stator = { 0, 0 },
        .i_rotor  = { 0, 0 },
        .theta    = 0,
        .mode_req = 0
    };

    /* 1. Latch wState atomically (M3 LDRB). */
    uint8_t ws = wState;
    st.mode_req = (uint8_t)(ws & 0x03u);

    /* 2. Read raw ADCs (sensored only) + always-on steer LPF. */
#if defined(PHASE_CURRENT_A) && defined(PHASE_CURRENT_B)
    adc_raw_a = (int16_t)adc_buffer.phase_current_a;
    adc_raw_b = (int16_t)adc_buffer.phase_current_b;
#endif
    steer_ema = ema_step(steer_ema, (int16_t)steer, FOC_STEER_LPF_SHIFT);
    int16_t steer_smoothed = (int16_t)(steer_ema >> FOC_STEER_LPF_SHIFT);
    int16_t trim_q8 = steer_to_trim_q8(steer_smoothed);

    /* 3 + 4. Calibration (discard + accumulate) → offset subtract. */
#if defined(PHASE_CURRENT_A) && defined(PHASE_CURRENT_B)
    if (!calib_done) {
        if (sample_count >= FOC_CALIB_DISCARD) {
            sum_A += adc_raw_a;
            sum_B += adc_raw_b;
        }
        sample_count++;
        if (sample_count >= (FOC_CALIB_DISCARD + FOC_CALIB_SAMPLES)) {
            offset_A = (int16_t)(sum_A / FOC_CALIB_SAMPLES);
            offset_B = (int16_t)(sum_B / FOC_CALIB_SAMPLES);
            calib_done = 1;
        }
        /* i_phase stays zero until calib completes. */
    } else {
        /* Channel A = PB0 = BLUE wire; Channel B = PA0 = YELLOW wire. */
        st.i_phase.iy = (int16_t)(adc_raw_b - offset_B);
        st.i_phase.ib = (int16_t)(adc_raw_a - offset_A);
    }
#endif

    /* 5. PLL advance + conditional edge. */
    int8_t prev_for_learner = pll.prev_pos;
    pll_advance(&pll);
    int edge_called = (pos != 0 && (int8_t)pos != pll.prev_pos);
    pll_edge_t edge = { 0, -1, PLL_EDGE_SEED };
    if (edge_called) {
        edge = pll_edge(&pll, (int8_t)pos, (const uint16_t *)sector_width_q8);
    }

    /* Online Hall-width learner. Gated by calib_done, learn-enable flag, and
     * adjacency (handled by edge.kind). Steady-state gate: vel > min, pwm/vel
     * within tolerance of refs captured on window's first edge. */
    if (foc_hall_learn_en && calib_done && edge_called) {
        if (edge.kind == PLL_EDGE_GLITCH) {
            for (int i = 0; i < 6; i++) learn_sum[i] = 0;
            learn_revs = 0;
        } else if (edge.kind == PLL_EDGE_VALID) {
            int16_t v  = pll.velocity_q8;
            int16_t av = (int16_t)((v < 0) ? -v : v);
            int speed_ok = (av > FOC_HALL_LEARN_MIN_VEL);

            /* First edge of a fresh window: all sums zero and revs zero. */
            int first_of_window = (learn_revs == 0);
            if (first_of_window) {
                for (int i = 0; i < 6; i++) {
                    if (learn_sum[i] != 0) { first_of_window = 0; break; }
                }
            }

            if (speed_ok) {
                if (first_of_window) {
                    learn_pwm_ref = (int16_t)pwm;
                    learn_vel_ref = v;
                }
                int32_t pwm_delta = (int32_t)pwm - learn_pwm_ref;
                if (pwm_delta < 0) pwm_delta = -pwm_delta;
                int32_t vel_delta = (int32_t)v - learn_vel_ref;
                if (vel_delta < 0) vel_delta = -vel_delta;
                int steady = (pwm_delta < FOC_HALL_LEARN_PWM_TOL)
                          && (vel_delta < FOC_HALL_LEARN_VEL_TOL);

                if (steady) {
                    learn_sum[edge.consumed_sector] += edge.consumed_dwell;
                    int forward_wrap = (prev_for_learner == 6 && pos == 1);
                    int reverse_wrap = (prev_for_learner == 1 && pos == 6);
                    if (forward_wrap || reverse_wrap) learn_revs++;

                    if (learn_revs >= FOC_HALL_LEARN_WINDOW_REVS) {
                        hall_widths_update((uint16_t *)sector_width_q8,
                                           learn_sum, FOC_HALL_LEARN_EMA_SHIFT);
                        for (int i = 0; i < 6; i++) learn_sum[i] = 0;
                        learn_revs = 0;
                    }
                } else {
                    for (int i = 0; i < 6; i++) learn_sum[i] = 0;
                    learn_revs = 0;
                }
            } else {
                for (int i = 0; i < 6; i++) learn_sum[i] = 0;
                learn_revs = 0;
            }
        }
        /* PLL_EDGE_SEED: nothing to attribute; leave window untouched. */
    }

    /* 6. θ composition. lead_q8 compensates sample-to-apply half-PWM latency. */
    int16_t electrical_angle = (int16_t)(pll.angle_accum_q8 >> 8);
    int16_t lead_q8          = (int16_t)(pll.velocity_q8 >> 1);
    int16_t theta = wrap_angle((int16_t)(
        electrical_angle
        + FOC_ANGLE_OFFSET_BASE
        + ((trim_q8 + 128) >> 8)
        + ((lead_q8 + 128) >> 8)));
    st.theta = theta;

    /* 7 + 8. Clarke, Park, EMAs — skipped while !calib_done so EMAs don't
     * ingest zeroed currents. */
#if defined(PHASE_CURRENT_A) && defined(PHASE_CURRENT_B)
    if (calib_done) {
        st.i_stator = clarke(st.i_phase);
        st.i_rotor  = park(st.i_stator, theta);
        i_d_ema = ema_step(i_d_ema, st.i_rotor.d, FOC_AVG_SHIFT);
        i_q_ema = ema_step(i_q_ema, st.i_rotor.q, FOC_AVG_SHIFT);
    }
#endif

    /* 9. Deferred RTT log: populate snapshot, set pending flag. No sprintf
     * in ISR — RttMainPoll formats and emits from the main loop. */
#ifdef RTT_REMOTE
    if (++log_counter >= FOC_RTT_LOG_DIV) {
        log_counter = 0;
        if (!log_pending) {
            log_snapshot.mode      = (int8_t)st.mode_req;
            log_snapshot.adc_raw_a = adc_raw_a;
            log_snapshot.adc_raw_b = adc_raw_b;
            log_snapshot.iy        = st.i_phase.iy;
            log_snapshot.ib        = st.i_phase.ib;
            log_snapshot.ig        = (int16_t)(-(st.i_phase.iy + st.i_phase.ib));
            log_snapshot.i_alpha   = st.i_stator.alpha;
            log_snapshot.i_beta    = st.i_stator.beta;
            log_snapshot.i_d       = st.i_rotor.d;
            log_snapshot.i_q       = st.i_rotor.q;
            log_snapshot.i_d_avg   = (int16_t)(i_d_ema >> FOC_AVG_SHIFT);
            log_snapshot.i_q_avg   = (int16_t)(i_q_ema >> FOC_AVG_SHIFT);
            log_snapshot.theta     = theta;
            log_snapshot.rpm       = (int16_t)(((int32_t)pll.velocity_q8 * 25) / 36);
            log_snapshot.vq_cmd    = vq_cmd;
            log_snapshot.pos       = (int8_t)pos;
            log_snapshot.pos_force = pos_force;
            log_snapshot.steer_trim_q8 = trim_q8;
            log_pending = 1;
        }
    }
#endif

    return st;
}

/* ============ bldc_get_pwm — dispatch ============ */
void bldc_get_pwm(int pwm, int pos, int *y, int *b, int *g) {
    int16_t y16 = 0, b16 = 0, g16 = 0;

    foc_state_t state = foc_sample(pwm, pos);

    /* Calibration gate: short-circuit zero while offsets are being sampled.
     * Leaves prev_mode untouched so the 0xFF sentinel still forces integrator
     * reset on the first post-calib dispatch. */
    if (!calib_done) {
        *y = 0; *b = 0; *g = 0;
        return;
    }

    /* Bench override: pos_force routes to BC at the forced pos regardless of
     * requested mode. Poisons prev_mode so post-force TRQ/SPD re-enters fresh. */
    if (pos_force != 0) {
        get_pwm_bc(pwm, pos_force, &y16, &b16, &g16);
        prev_mode = 0xFF;
        *y = y16; *b = b16; *g = g16;
        return;
    }

    /* Reset current-loop integrators on any mode transition; stops wind-up
     * from the previous mode kicking the first tick. */
    if (state.mode_req != prev_mode) {
        iq_pi.integrator_raw = 0;
        id_pi.integrator_raw = 0;
    }

    switch (state.mode_req) {
    case 0: /* BC */
        get_pwm_bc(pwm, pos, &y16, &b16, &g16);
        vq_cmd = 0;
        break;

    case 1: { /* VLT — open-loop Vq, Vd=0 */
        int32_t vq = ((int32_t)pwm * FOC_VQ_MAX) / BLDC_TIMER_MID_VALUE;
        if (vq >  FOC_VQ_MAX) vq =  FOC_VQ_MAX;
        if (vq < -FOC_VQ_MAX) vq = -FOC_VQ_MAX;
        vq_cmd = (int16_t)vq;

        dq_v_t   v_dq  = { .d = 0, .q = vq_cmd };
        ab_v_t   v_ab  = ipark(v_dq, state.theta);
        ab_pwm_t v_pwm = ab_scale_for_pwm(v_ab);
        iclarke_svpwm(v_pwm, &y16, &b16, &g16);
        if (foc_dtc_en) dtc_apply(state.i_phase, &y16, &b16, &g16);
        break;
    }

    case 2: /* SPD — Phase 3 */
    case 3: /* TRQ — Phase 2 */
    default:
        /* Phase 1 falls through to BC so the modes are reachable from the UI
         * without behaving as silent no-ops. */
        get_pwm_bc(pwm, pos, &y16, &b16, &g16);
        vq_cmd = 0;
        break;
    }

    prev_mode = state.mode_req;

    *y = y16; *b = b16; *g = g16;
}

/* ============ RttMainPoll — main-loop log emitter ============ */
#ifdef RTT_REMOTE
static const char *mode_str(int8_t m) {
    static const char *names[4] = { "BC", "VLT", "SPD", "TRQ" };
    return (m >= 0 && m <= 3) ? names[m] : "??";
}
#endif

void RttMainPoll(void) {
#ifdef RTT_REMOTE
    if (!log_pending) return;
    log_snapshot_t local = log_snapshot;   /* copy out while flag is still set */
    log_pending = 0;                       /* release to ISR */

    char buf[220];
    int n = snprintf(buf, sizeof(buf),
        "m:%s adc_a=%d adc_b=%d Ia:%d Ib:%d Ig:%d "
        "Ialpha:%d Ibeta:%d Id:%d Iq:%d Idavg:%d Iqavg:%d "
        "ang:%d rpm:%d Vq:%d p:%d f:%d trim:%d"
#ifdef WINDOWS_RN
        "\r\n"
#else
        "\n"
#endif
        ,
        mode_str(local.mode), local.adc_raw_a, local.adc_raw_b,
        local.iy, local.ib, local.ig,
        local.i_alpha, local.i_beta, local.i_d, local.i_q,
        local.i_d_avg, local.i_q_avg,
        local.theta, local.rpm, local.vq_cmd,
        (int)local.pos, (int)local.pos_force, local.steer_trim_q8);
    if (n > 0) {
        if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
        SEGGER_RTT_Write(0, buf, (unsigned)n);
    }
#endif
}

#endif /* BLDC_FOC */

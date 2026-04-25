#include "test_helpers.h"
#include "../Inc/bldcFOC_core.h"

/* Phase 1: signed_shift + ema_step. Phase 2 adds pi_step body coverage. */

static void test_signed_shift_positive(void) {
    EXPECT_EQ(signed_shift(1000, 3), 125);      /* 1000/8 = 125 exact */
    EXPECT_EQ(signed_shift(-1000, 3), -125);
    EXPECT_EQ(signed_shift(0, 3), 0);
    /* half-away-from-zero rounding: 500/8 = 62.5 → 63; -500/8 = -62.5 → -63 */
    EXPECT_EQ(signed_shift(500, 3), 63);
    EXPECT_EQ(signed_shift(-500, 3), -63);
}

static void test_signed_shift_zero_shift(void) {
    EXPECT_EQ(signed_shift(1000, 0), 1000);
    EXPECT_EQ(signed_shift(-1000, 0), -1000);
    EXPECT_EQ(signed_shift(0, 0), 0);
}

static void test_signed_shift_negative_shift(void) {
    EXPECT_EQ(signed_shift(100, -3), 800);      /* 100 << 3 */
    EXPECT_EQ(signed_shift(-100, -3), -800);
}

static void test_signed_shift_roundtrip_symmetry(void) {
    /* +err and -err must produce equal-magnitude opposite-sign output for
     * every shift; this is what kills the ASR-bias drift in pi_step's
     * integrator. */
    for (long err = -1000; err <= 1000; err += 17) {
        for (int s = 1; s <= 10; s++) {
            long p = signed_shift((int32_t)err, (int8_t)s);
            long n = signed_shift((int32_t)(-err), (int8_t)s);
            EXPECT_EQ(p, -n);
        }
    }
}

static void test_ema_settle(void) {
    int32_t st = 0;
    int16_t sample = 1000;
    uint8_t shift = 4;
    for (int i = 0; i < 500; i++) st = ema_step(st, sample, shift);
    /* equilibrium = sample << shift = 16000 */
    EXPECT_WITHIN(st, 16000, 2);
}

static void test_ema_holds_dc(void) {
    int32_t st = 16000;
    for (int i = 0; i < 100; i++) st = ema_step(st, 1000, 4);
    EXPECT_WITHIN(st, 16000, 2);
}

static void test_ema_time_constant(void) {
    /* After 2^shift ticks of step input from rest, state should be ~63% of equilibrium. */
    int32_t st = 0;
    uint8_t shift = 6;                         /* τ = 64 samples */
    int16_t sample = 100;
    for (int i = 0; i < (1 << shift); i++) st = ema_step(st, sample, shift);
    int32_t eq = (int32_t)sample << shift;      /* 6400 */
    /* 1 - 1/e ≈ 0.632; accept 55%-75% for integer slop */
    EXPECT_TRUE(st > eq * 55 / 100);
    EXPECT_TRUE(st < eq * 75 / 100);
}

/* ================= pi_step tests (Phase 2) ================= */

static void test_pi_const_err_growth(void) {
    /* With Kp=1 (kp_shift=0), Ki_shift=10: each tick integrator += err.
     * After 1024 ticks at err=100: integrator_raw = 102400, ki_term = 100,
     * kp_term = 100 → output = 200. */
    pi_state_t st = { 0 };
    pi_gains_t g  = { 0, 10, 2 };
    int16_t out = 0;
    for (int n = 0; n < 1024; n++) out = pi_step(&st, 100, 10000, g);
    EXPECT_WITHIN(out, 200, 1);
}

static void test_pi_small_err_no_dead_zone(void) {
    /* |err| < 2^ki_shift: naive "shift-on-write" would drop the increment.
     * Full-precision accumulator integrates fine. err=1, ki_shift=10: after
     * 1024 ticks, ki_term = 1 (+ kp_term = 1) → output = 2. */
    pi_state_t st = { 0 };
    pi_gains_t g  = { 0, 10, 2 };
    int16_t out = 0;
    for (int n = 0; n < 1024; n++) out = pi_step(&st, 1, 10000, g);
    EXPECT_WITHIN(out, 2, 1);

    /* Another 1024 ticks → ki_term=2, output=3. */
    for (int n = 0; n < 1024; n++) out = pi_step(&st, 1, 10000, g);
    EXPECT_WITHIN(out, 3, 1);
}

static void test_pi_saturates_at_clamp(void) {
    pi_state_t st = { 0 };
    pi_gains_t g  = { 0, 10, 2 };
    int16_t clamp = 50;
    int16_t out = 0;
    for (int n = 0; n < 10000; n++) out = pi_step(&st, 500, clamp, g);
    EXPECT_EQ(out, clamp);

    /* Negative-rail saturation too. */
    pi_state_t st2 = { 0 };
    int16_t out2 = 0;
    for (int n = 0; n < 10000; n++) out2 = pi_step(&st2, -500, clamp, g);
    EXPECT_EQ(out2, -clamp);
}

static void test_pi_antiwindup_bounds_integrator(void) {
    /* Under prolonged saturation, AW must keep the integrator from running
     * away. Without AW, 500·100000 = 5×10^7 grows toward int32 limits.
     * With AW, it settles at ~-(err-clamp)·2^ki_shift ≈ -460800 — the level
     * where ki_term cancels kp_term's overshoot. Bound at 10× expected. */
    pi_state_t st = { 0 };
    pi_gains_t g  = { 0, 10, 2 };
    int16_t clamp = 50;
    for (int n = 0; n < 100000; n++) pi_step(&st, 500, clamp, g);
    EXPECT_TRUE(st.integrator_raw <  5000000);
    EXPECT_TRUE(st.integrator_raw > -5000000);
}

static void test_pi_antiwindup_releases_quickly(void) {
    /* After saturation, when err flips sign, output should de-saturate
     * within a few ticks — not hang at the rail forever. */
    pi_state_t st = { 0 };
    pi_gains_t g  = { 0, 10, 2 };
    int16_t clamp = 50;
    for (int n = 0; n < 10000; n++) pi_step(&st, 500, clamp, g);
    /* Now flip err sign; a P-only response of −500 should drop out below +clamp
     * on the very next tick. */
    int16_t out = pi_step(&st, -500, clamp, g);
    EXPECT_TRUE(out < clamp);
}

static void test_pi_zero_err_holds_output(void) {
    pi_state_t st = { 0 };
    pi_gains_t g  = { 0, 10, 2 };
    /* Wind up integrator. */
    for (int n = 0; n < 512; n++) pi_step(&st, 100, 10000, g);
    /* integrator_raw = 51200; ki_term = signed_shift(51200, 10) = 50 */
    int16_t baseline = pi_step(&st, 0, 10000, g);
    EXPECT_EQ(baseline, 50);
    /* Many more zero-err ticks: output holds. */
    int16_t held = 0;
    for (int n = 0; n < 1000; n++) held = pi_step(&st, 0, 10000, g);
    EXPECT_EQ(held, 50);
}

static void test_pi_reset_returns_zero(void) {
    pi_state_t st = { 0 };
    pi_gains_t g  = { 0, 10, 2 };
    for (int n = 0; n < 500; n++) pi_step(&st, 100, 10000, g);
    st.integrator_raw = 0;              /* caller-side reset on mode transition */
    int16_t out = pi_step(&st, 0, 10000, g);
    EXPECT_EQ(out, 0);
}

static void test_pi_sign_symmetric_integration(void) {
    /* Alternating +err, -err should bring integrator back to exactly zero;
     * signed_shift's half-away-from-zero rounding is what makes this work. */
    pi_state_t st = { 0 };
    pi_gains_t g  = { 0, 10, 2 };
    for (int n = 0; n < 100; n++) {
        pi_step(&st,  100, 10000, g);
        pi_step(&st, -100, 10000, g);
    }
    EXPECT_EQ(st.integrator_raw, 0);
}

static void test_pi_kp_gain_greater_than_one(void) {
    /* kp_shift = -2 → Kp = 4. P-only (first tick): output = err << 2 = 4·err. */
    pi_state_t st = { 0 };
    pi_gains_t g  = { -2, 10, 2 };
    int16_t out = pi_step(&st, 10, 10000, g);
    /* After 1 tick: integrator_raw=10, ki_term=signed_shift(10, 10)=0;
     * kp_term=10<<2=40 → output=40. */
    EXPECT_EQ(out, 40);
}

int run_pi_tests(void) {
    test_signed_shift_positive();
    test_signed_shift_zero_shift();
    test_signed_shift_negative_shift();
    test_signed_shift_roundtrip_symmetry();
    test_ema_settle();
    test_ema_holds_dc();
    test_ema_time_constant();
    test_pi_const_err_growth();
    test_pi_small_err_no_dead_zone();
    test_pi_saturates_at_clamp();
    test_pi_antiwindup_bounds_integrator();
    test_pi_antiwindup_releases_quickly();
    test_pi_zero_err_holds_output();
    test_pi_reset_returns_zero();
    test_pi_sign_symmetric_integration();
    test_pi_kp_gain_greater_than_one();
    return 0;
}

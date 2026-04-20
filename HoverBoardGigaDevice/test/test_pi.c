#include "test_helpers.h"
#include "../Inc/bldcFOC_core.h"

/* Phase 1 scope: signed_shift and ema_step only. pi_step body lands in Phase 2. */

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

int run_pi_tests(void) {
    test_signed_shift_positive();
    test_signed_shift_zero_shift();
    test_signed_shift_negative_shift();
    test_signed_shift_roundtrip_symmetry();
    test_ema_settle();
    test_ema_holds_dc();
    test_ema_time_constant();
    return 0;
}

#include "test_helpers.h"
#include "../Inc/bldcFOC_core.h"

static void test_dtc_term_zero(void) {
    EXPECT_EQ(dtc_term(0), 0);
}

static void test_dtc_term_monotonic(void) {
    /* ramp across ±FOC_DTC_MAX · 2^FOC_DTC_SHIFT (= ±120) */
    for (int i = -500; i < 500; i++) {
        EXPECT_TRUE(dtc_term((int16_t)i) <= dtc_term((int16_t)(i + 1)));
    }
}

static void test_dtc_term_saturates(void) {
    /* clip kicks in at |i| = FOC_DTC_MAX << FOC_DTC_SHIFT = 120 */
    EXPECT_EQ(dtc_term(120), FOC_DTC_MAX);
    EXPECT_EQ(dtc_term(1000), FOC_DTC_MAX);
    EXPECT_EQ(dtc_term(32000), FOC_DTC_MAX);
    EXPECT_EQ(dtc_term(-120), -FOC_DTC_MAX);
    EXPECT_EQ(dtc_term(-1000), -FOC_DTC_MAX);
    EXPECT_EQ(dtc_term(-32000), -FOC_DTC_MAX);
}

static void test_dtc_term_sign_preserved(void) {
    for (int i = 1; i < 200; i++) {
        EXPECT_TRUE(dtc_term((int16_t)i) >= 0);
        EXPECT_TRUE(dtc_term((int16_t)(-i)) <= 0);
    }
}

static void test_dtc_apply_balanced_linear(void) {
    /* Values chosen so i/4 is exact for every phase → zero-sum preserved.
     * iy=400, ib=-200 → ig=-200. dtc_term(400)=30 (clipped!), so use smaller. */
    iph_t i = { .iy = 80, .ib = -40 };              /* ig = -40, all in linear region */
    int16_t y = 0, b = 0, g = 0;
    dtc_apply(i, &y, &b, &g);
    EXPECT_EQ(y + b + g, 0);
    EXPECT_EQ(y, 80 >> FOC_DTC_SHIFT);
    EXPECT_EQ(b, (-40) >> FOC_DTC_SHIFT);
    EXPECT_EQ(g, (-40) >> FOC_DTC_SHIFT);
}

static void test_dtc_apply_clipped_nonzero_sum(void) {
    /* iy=1000 → clips +30; ib=-500 → clips -30; ig=-500 → clips -30. Sum = -30. */
    iph_t i = { .iy = 1000, .ib = -500 };
    int16_t y = 0, b = 0, g = 0;
    dtc_apply(i, &y, &b, &g);
    EXPECT_EQ(y, FOC_DTC_MAX);
    EXPECT_EQ(b, -FOC_DTC_MAX);
    EXPECT_EQ(g, -FOC_DTC_MAX);
    EXPECT_EQ(y + b + g, -FOC_DTC_MAX);
}

static void test_dtc_apply_accumulates(void) {
    /* dtc_apply should add to the existing PWM deltas, not overwrite. */
    iph_t i = { .iy = 0, .ib = 0 };                 /* all zero → no correction */
    int16_t y = 100, b = -50, g = -50;
    dtc_apply(i, &y, &b, &g);
    EXPECT_EQ(y, 100);
    EXPECT_EQ(b, -50);
    EXPECT_EQ(g, -50);
}

int run_dtc_tests(void) {
    test_dtc_term_zero();
    test_dtc_term_monotonic();
    test_dtc_term_saturates();
    test_dtc_term_sign_preserved();
    test_dtc_apply_balanced_linear();
    test_dtc_apply_clipped_nonzero_sum();
    test_dtc_apply_accumulates();
    return 0;
}

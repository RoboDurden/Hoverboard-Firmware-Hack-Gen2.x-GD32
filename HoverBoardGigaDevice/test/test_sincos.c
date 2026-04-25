#include "test_helpers.h"
#include "../Inc/bldcFOC_core.h"

static void test_boundary(void) {
    EXPECT_EQ(sin_q15(0), 0);
    EXPECT_EQ(sin_q15(90), 32767);
    EXPECT_EQ(sin_q15(180), 0);
    EXPECT_EQ(sin_q15(270), -32767);
    EXPECT_EQ(sin_q15(360), 0);
    EXPECT_EQ(cos_q15(0), 32767);
    EXPECT_EQ(cos_q15(90), 0);
    EXPECT_EQ(cos_q15(180), -32767);
    EXPECT_EQ(cos_q15(270), 0);
}

static void test_periodicity(void) {
    for (int a = -360; a <= 720; a += 7) {
        EXPECT_EQ(sin_q15((int16_t)a), sin_q15((int16_t)(a + 360)));
        EXPECT_EQ(sin_q15((int16_t)a), sin_q15((int16_t)(a - 360)));
    }
}

static void test_oddness(void) {
    for (int a = 0; a < 360; a += 3) {
        EXPECT_EQ(sin_q15((int16_t)(-a)), -sin_q15((int16_t)a));
    }
}

static void test_even_cosine(void) {
    for (int a = 0; a < 360; a += 3) {
        EXPECT_EQ(cos_q15((int16_t)(-a)), cos_q15((int16_t)a));
    }
}

static void test_phase_relation(void) {
    for (int a = 0; a < 360; a += 3) {
        EXPECT_EQ(cos_q15((int16_t)a), sin_q15((int16_t)(a + 90)));
    }
}

static void test_half_symmetry(void) {
    for (int a = 0; a < 180; a++) {
        EXPECT_EQ(sin_q15((int16_t)a), -sin_q15((int16_t)(a + 180)));
    }
}

static void test_range_bound(void) {
    for (int a = 0; a < 360; a++) {
        int16_t s = sin_q15((int16_t)a);
        int16_t c = cos_q15((int16_t)a);
        EXPECT_TRUE(s <= 32767 && s >= -32767);  /* never -32768 */
        EXPECT_TRUE(c <= 32767 && c >= -32767);
    }
}

static void test_monotonicity(void) {
    /* sin increasing on [0, 90] */
    for (int a = 0; a < 90; a++) {
        EXPECT_TRUE(sin_q15((int16_t)a) <= sin_q15((int16_t)(a + 1)));
    }
    /* cos decreasing on [0, 90] */
    for (int a = 0; a < 90; a++) {
        EXPECT_TRUE(cos_q15((int16_t)a) >= cos_q15((int16_t)(a + 1)));
    }
}

static void test_quarter_symmetry(void) {
    for (int a = 0; a <= 90; a++) {
        EXPECT_EQ(sin_q15((int16_t)(90 - a)), cos_q15((int16_t)a));
    }
}

static void test_pythagorean(void) {
    for (int a = 0; a < 360; a += 3) {
        int32_t s = sin_q15((int16_t)a);
        int32_t c = cos_q15((int16_t)a);
        int32_t r = s * s + c * c;
        /* expected 32767^2 = 1073676289; allow rounding slack */
        EXPECT_WITHIN(r, 1073676289L, 200000L);
    }
}

static void test_wrap_angle(void) {
    EXPECT_EQ(wrap_angle(0), 0);
    EXPECT_EQ(wrap_angle(359), 359);
    EXPECT_EQ(wrap_angle(360), 0);
    EXPECT_EQ(wrap_angle(720), 0);
    EXPECT_EQ(wrap_angle(-1), 359);
    EXPECT_EQ(wrap_angle(-360), 0);
    EXPECT_EQ(wrap_angle(1079), 359);
    EXPECT_EQ(wrap_angle(-721), 359);  /* -721 + 720 + 360 = 359 */
}

int run_sincos_tests(void) {
    test_boundary();
    test_periodicity();
    test_oddness();
    test_even_cosine();
    test_phase_relation();
    test_half_symmetry();
    test_range_bound();
    test_monotonicity();
    test_quarter_symmetry();
    test_pythagorean();
    test_wrap_angle();
    return 0;
}

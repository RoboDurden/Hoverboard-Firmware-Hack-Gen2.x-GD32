#include "test_helpers.h"
#include "../Inc/bldcFOC_core.h"

static void check(int pos, int pwm, int e_y, int e_b, int e_g) {
    int16_t y = -99, b = -99, g = -99;
    get_pwm_bc(pwm, pos, &y, &b, &g);
    EXPECT_EQ(y, e_y);
    EXPECT_EQ(b, e_b);
    EXPECT_EQ(g, e_g);
}

static void test_bc_parity(void) {
    int p = 500;
    check(1, p,  0,  p, -p);
    check(2, p, -p,  p,  0);
    check(3, p, -p,  0,  p);
    check(4, p,  0, -p,  p);
    check(5, p,  p, -p,  0);
    check(6, p,  p,  0, -p);
}

static void test_bc_invalid_pos(void) {
    check(0, 500, 0, 0, 0);
    check(-1, 500, 0, 0, 0);
    check(7, 500, 0, 0, 0);
    check(100, 500, 0, 0, 0);
}

static void test_bc_linear_in_pwm(void) {
    for (int pos = 1; pos <= 6; pos++) {
        int16_t y1, b1, g1, y2, b2, g2;
        get_pwm_bc(500, pos, &y1, &b1, &g1);
        get_pwm_bc(-500, pos, &y2, &b2, &g2);
        EXPECT_EQ(y1, -y2);
        EXPECT_EQ(b1, -b2);
        EXPECT_EQ(g1, -g2);
    }
}

static void test_bc_phase_sum_zero(void) {
    for (int pos = 0; pos <= 7; pos++) {
        for (int pwm = -1000; pwm <= 1000; pwm += 123) {
            int16_t y = 0, b = 0, g = 0;
            get_pwm_bc(pwm, pos, &y, &b, &g);
            EXPECT_EQ(y + b + g, 0);
        }
    }
}

int run_bc_tests(void) {
    test_bc_parity();
    test_bc_invalid_pos();
    test_bc_linear_in_pwm();
    test_bc_phase_sum_zero();
    return 0;
}

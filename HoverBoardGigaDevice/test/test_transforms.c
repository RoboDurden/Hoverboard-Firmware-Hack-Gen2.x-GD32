#include "test_helpers.h"
#include "../Inc/bldcFOC_core.h"

static void test_clarke_balanced(void) {
    /* iy + ib + ig = 0 balanced currents → α≈iy, β≈0 when ib=-iy/2 */
    iph_t i = { .iy = 1000, .ib = -500 };   /* ig = -500 */
    ab_i_t ab = clarke(i);
    EXPECT_WITHIN(ab.alpha, 1000, 1);
    EXPECT_WITHIN(ab.beta, 0, 1);

    /* iy = 500, ib = 500, ig = -1000; Iβ = (500 + 1000)/√3 ≈ 866 */
    i.iy = 500; i.ib = 500;
    ab = clarke(i);
    EXPECT_WITHIN(ab.alpha, 500, 1);
    EXPECT_WITHIN(ab.beta, 866, 2);

    /* iy=0, ib=A·√3/2 (balanced @ φ=90°) → α=0, β=A */
    i.iy = 0; i.ib = 866;
    ab = clarke(i);
    EXPECT_WITHIN(ab.alpha, 0, 1);
    EXPECT_WITHIN(ab.beta, 1000, 2);
}

static void test_park_ipark_identity(void) {
    /* The invariant: ipark then park returns the original (d,q) for any θ. */
    for (int theta = 0; theta < 360; theta += 11) {
        for (int d = -800; d <= 800; d += 237) {
            for (int q = -800; q <= 800; q += 251) {
                dq_v_t v_in = { .d = (int16_t)d, .q = (int16_t)q };
                ab_v_t v_ab = ipark(v_in, (int16_t)theta);
                /* Park consumes currents; we round-trip via same-shape struct. */
                ab_i_t ab_i = { .alpha = v_ab.alpha, .beta = v_ab.beta };
                dq_i_t out = park(ab_i, (int16_t)theta);
                EXPECT_WITHIN(out.d, d, 3);
                EXPECT_WITHIN(out.q, q, 3);
            }
        }
    }
}

static void test_iclarke_svpwm_l2l(void) {
    /* Line-to-line differences are preserved through min-max centering,
     * because centering adds the same common-mode to every phase. */
    for (int theta = 0; theta < 360; theta += 7) {
        dq_v_t v_dq = { .d = 0, .q = 10000 };
        ab_v_t v_ab = ipark(v_dq, (int16_t)theta);
        ab_pwm_t v_pwm = ab_scale_for_pwm(v_ab);

        int32_t alpha = v_pwm.alpha, beta = v_pwm.beta;
        int32_t vy_raw = alpha;
        int32_t vb_raw = (-alpha * Q15_HALF + beta * Q15_SQRT3_OVER_2) >> 15;
        int32_t vg_raw = (-alpha * Q15_HALF - beta * Q15_SQRT3_OVER_2) >> 15;

        int16_t y = 0, b = 0, g = 0;
        iclarke_svpwm(v_pwm, &y, &b, &g);

        EXPECT_WITHIN((long)(y - b), (long)(vy_raw - vb_raw), 2);
        EXPECT_WITHIN((long)(b - g), (long)(vb_raw - vg_raw), 2);
        EXPECT_WITHIN((long)(g - y), (long)(vg_raw - vy_raw), 2);
    }
}

static void test_svpwm_headroom(void) {
    /* With |V|=FOC_VQ_MAX=20000 internal, pre-scale amplitude = 20000/16 = 1250.
     * SVPWM min-max injection drops peak phase-to-common to √3/2 · 1250 ≈ 1083. */
    int max_abs = 0;
    for (int theta = 0; theta < 360; theta++) {
        dq_v_t v_dq = { .d = 0, .q = FOC_VQ_MAX };
        ab_v_t v_ab = ipark(v_dq, (int16_t)theta);
        ab_pwm_t v_pwm = ab_scale_for_pwm(v_ab);
        int16_t y = 0, b = 0, g = 0;
        iclarke_svpwm(v_pwm, &y, &b, &g);
        int ay = (y < 0 ? -y : y);
        int ab_abs = (b < 0 ? -b : b);
        int ag = (g < 0 ? -g : g);
        if (ay > max_abs) max_abs = ay;
        if (ab_abs > max_abs) max_abs = ab_abs;
        if (ag > max_abs) max_abs = ag;
    }
    EXPECT_WITHIN(max_abs, 1083, 10);
}

static void test_ab_scale_plain_shift(void) {
    ab_v_t v = { .alpha = 20000, .beta = -16000 };
    ab_pwm_t p = ab_scale_for_pwm(v);
    EXPECT_EQ(p.alpha, (int16_t)(20000 >> FOC_OUT_SHIFT));
    EXPECT_EQ(p.beta,  (int16_t)(-16000 >> FOC_OUT_SHIFT));
}

int run_transforms_tests(void) {
    test_clarke_balanced();
    test_park_ipark_identity();
    test_iclarke_svpwm_l2l();
    test_svpwm_headroom();
    test_ab_scale_plain_shift();
    return 0;
}

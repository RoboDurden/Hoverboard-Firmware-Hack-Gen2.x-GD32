#include "test_helpers.h"
#include "../Inc/bldcFOC_core.h"
#include <stdint.h>

static void nominal_widths(uint16_t w[6]) {
    for (int i = 0; i < 6; i++) w[i] = FOC_PLL_NOMINAL_WIDTH_Q8;
}

static void test_pll_seed(void) {
    pll_state_t pll = { 0 };
    uint16_t w[6]; nominal_widths(w);

    pll_edge_t e = pll_edge(&pll, 1, w);
    EXPECT_EQ(e.kind, PLL_EDGE_SEED);
    EXPECT_EQ(e.consumed_sector, -1);
    EXPECT_EQ(pll.prev_pos, 1);
    EXPECT_EQ(pll.velocity_q8, 0);
    EXPECT_EQ(pll.angle_accum_q8, (int32_t)180 << 8);
    EXPECT_EQ(pll.sector_dwell, 0);

    /* Seed on pos=4 (sector centre = 0°). */
    pll_state_t p2 = { 0 };
    e = pll_edge(&p2, 4, w);
    EXPECT_EQ(e.kind, PLL_EDGE_SEED);
    EXPECT_EQ(p2.angle_accum_q8, 0);
}

static void test_pll_forward(void) {
    pll_state_t pll = { 0 };
    uint16_t w[6]; nominal_widths(w);

    pll_edge(&pll, 1, w);                          /* seed */
    for (int i = 0; i < 100; i++) pll_advance(&pll);

    pll_edge_t e = pll_edge(&pll, 2, w);
    EXPECT_EQ(e.kind, PLL_EDGE_VALID);
    EXPECT_EQ(e.consumed_dwell, 100);
    EXPECT_EQ(e.consumed_sector, 0);
    EXPECT_EQ(pll.velocity_q8, 15360 / 100);
    /* forward snap targets (with wrap at period = 360<<8 = 92160):
     * pos: 1     2     3     4     5    6
     *     38400 53760 69120 84480 7680 23040 */
    EXPECT_EQ(pll.angle_accum_q8, 53760);

    const int32_t expected[7] = { 0, 38400, 53760, 69120, 84480, 7680, 23040 };
    for (int pos = 3; pos <= 6; pos++) {
        for (int i = 0; i < 100; i++) pll_advance(&pll);
        e = pll_edge(&pll, (int8_t)pos, w);
        EXPECT_EQ(e.kind, PLL_EDGE_VALID);
        EXPECT_TRUE(pll.velocity_q8 > 0);
        EXPECT_EQ(pll.angle_accum_q8, expected[pos]);
    }
    /* wrap 6→1 (delta = -5, forward) */
    for (int i = 0; i < 100; i++) pll_advance(&pll);
    e = pll_edge(&pll, 1, w);
    EXPECT_EQ(e.kind, PLL_EDGE_VALID);
    EXPECT_EQ(pll.angle_accum_q8, expected[1]);
}

static void test_pll_reverse(void) {
    pll_state_t pll = { 0 };
    uint16_t w[6]; nominal_widths(w);

    pll_edge(&pll, 1, w);
    for (int i = 0; i < 100; i++) pll_advance(&pll);
    pll_edge_t e = pll_edge(&pll, 6, w);           /* 1→6 = reverse (delta=5) */
    EXPECT_EQ(e.kind, PLL_EDGE_VALID);
    EXPECT_TRUE(pll.velocity_q8 < 0);
    /* reverse snap: entry_fwd(6)+w[5] = 38400 + 5·15360 + 15360 = 130560, wrap→38400 */
    EXPECT_EQ(pll.angle_accum_q8, 38400);

    for (int i = 0; i < 100; i++) pll_advance(&pll);
    e = pll_edge(&pll, 5, w);                      /* 6→5, delta=-1 */
    EXPECT_EQ(e.kind, PLL_EDGE_VALID);
    EXPECT_TRUE(pll.velocity_q8 < 0);
}

static void test_pll_glitch_preserves_state(void) {
    pll_state_t pll = { 0 };
    uint16_t w[6]; nominal_widths(w);

    pll_edge(&pll, 1, w);
    for (int i = 0; i < 100; i++) pll_advance(&pll);
    pll_edge(&pll, 2, w);
    for (int i = 0; i < 50; i++) pll_advance(&pll);

    int8_t   prev  = pll.prev_pos;
    uint16_t dwell = pll.sector_dwell;
    int16_t  vel   = pll.velocity_q8;

    /* pos jump of +2 = glitch */
    pll_edge_t e = pll_edge(&pll, 4, w);
    EXPECT_EQ(e.kind, PLL_EDGE_GLITCH);
    EXPECT_EQ(e.consumed_sector, -1);
    EXPECT_EQ(pll.prev_pos, prev);
    EXPECT_EQ(pll.sector_dwell, dwell);
    EXPECT_EQ(pll.velocity_q8, vel);
}

static void test_pll_velocity_clamp(void) {
    pll_state_t pll = { 0 };
    uint16_t w[6]; nominal_widths(w);

    pll_edge(&pll, 1, w);
    pll_advance(&pll);                             /* dwell = 1 */
    pll_edge(&pll, 2, w);                          /* 15360 / 1 = 15360 → clamped */
    EXPECT_EQ(pll.velocity_q8, FOC_PLL_VEL_CLAMP);

    pll_state_t r = { 0 };
    pll_edge(&r, 1, w);
    pll_advance(&r);
    pll_edge(&r, 6, w);                            /* reverse, same magnitude */
    EXPECT_EQ(r.velocity_q8, -FOC_PLL_VEL_CLAMP);
}

static void test_pll_pos_zero_advances_accumulator(void) {
    pll_state_t pll = { 0 };
    pll.prev_pos = 1;
    pll.velocity_q8 = 256;
    pll.angle_accum_q8 = 0;

    for (int i = 0; i < 10; i++) pll_advance(&pll);
    EXPECT_EQ(pll.angle_accum_q8, 10 * 256);
    EXPECT_EQ(pll.sector_dwell, 10);
    EXPECT_EQ(pll.prev_pos, 1);   /* unchanged — no edge */
}

static void test_pll_dwell_saturation(void) {
    pll_state_t pll = { 0 };
    pll.prev_pos = 1;
    pll.velocity_q8 = 0;
    pll.sector_dwell = 0;
    for (int i = 0; i < 70000; i++) pll_advance(&pll);
    EXPECT_EQ(pll.sector_dwell, UINT16_MAX);

    /* next real edge at saturated dwell → near-zero velocity, no spike */
    uint16_t w[6]; nominal_widths(w);
    pll_edge_t e = pll_edge(&pll, 2, w);
    EXPECT_EQ(e.kind, PLL_EDGE_VALID);
    EXPECT_WITHIN(pll.velocity_q8, 0, 1);   /* 15360 / 65535 ≈ 0 */
}

static void test_pll_accumulator_wraps(void) {
    pll_state_t pll = { 0 };
    pll.prev_pos = 1;
    pll.velocity_q8 = 100;
    pll.angle_accum_q8 = (360 << 8) - 50;     /* just below the wrap boundary */
    pll_advance(&pll);
    EXPECT_EQ(pll.angle_accum_q8, 50);        /* wrapped around */

    pll.velocity_q8 = -100;
    pll.angle_accum_q8 = 50;
    pll_advance(&pll);
    EXPECT_EQ(pll.angle_accum_q8, (360 << 8) - 50);
}

int run_pll_tests(void) {
    test_pll_seed();
    test_pll_forward();
    test_pll_reverse();
    test_pll_glitch_preserves_state();
    test_pll_velocity_clamp();
    test_pll_pos_zero_advances_accumulator();
    test_pll_dwell_saturation();
    test_pll_accumulator_wraps();
    return 0;
}

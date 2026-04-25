#include "test_helpers.h"
#include "../Inc/bldcFOC_core.h"

static void reset_widths(uint16_t w[6]) {
    for (int i = 0; i < 6; i++) w[i] = FOC_PLL_NOMINAL_WIDTH_Q8;
}

static void test_hall_uniform_leaves_nominal(void) {
    uint16_t w[6]; reset_widths(w);
    uint32_t sums[6] = { 100, 100, 100, 100, 100, 100 };
    hall_widths_update(w, sums, 3);
    for (int i = 0; i < 6; i++) EXPECT_EQ(w[i], FOC_PLL_NOMINAL_WIDTH_Q8);
}

static void test_hall_total_zero_guard(void) {
    uint16_t w[6]; reset_widths(w);
    uint32_t sums[6] = { 0, 0, 0, 0, 0, 0 };
    hall_widths_update(w, sums, 3);
    for (int i = 0; i < 6; i++) EXPECT_EQ(w[i], FOC_PLL_NOMINAL_WIDTH_Q8);
}

static void test_hall_ema_shift_zero_snaps(void) {
    uint16_t w[6]; reset_widths(w);
    /* sector 0 ≈ 1.2× nominal, others ≈ 0.96× → target ≈ (18432, 14745×5) */
    uint32_t sums[6] = { 120, 96, 96, 96, 96, 96 };
    hall_widths_update(w, sums, 0);
    EXPECT_EQ(w[0], 18432);
    for (int i = 1; i < 6; i++) EXPECT_EQ(w[i], 14745);
}

static void test_hall_skewed_converges(void) {
    uint16_t w[6]; reset_widths(w);
    uint32_t sums[6] = { 120, 96, 96, 96, 96, 96 };
    for (int iter = 0; iter < 100; iter++) hall_widths_update(w, sums, 3);
    EXPECT_WITHIN(w[0], 18432, 50);
    for (int i = 1; i < 6; i++) EXPECT_WITHIN(w[i], 14745, 50);
}

static void test_hall_ema_one_step_magnitude(void) {
    /* One update with ema_shift=3: delta /= 8 */
    uint16_t w[6]; reset_widths(w);
    uint32_t sums[6] = { 120, 96, 96, 96, 96, 96 };
    hall_widths_update(w, sums, 3);
    /* widths[0] target = 18432; delta = 18432-15360 = 3072; applied = 3072>>3 = 384 */
    EXPECT_EQ(w[0], FOC_PLL_NOMINAL_WIDTH_Q8 + 384);
}

static void test_hall_sum_near_invariant(void) {
    uint16_t w[6]; reset_widths(w);
    uint32_t sums[6] = { 120, 96, 96, 96, 96, 96 };
    for (int iter = 0; iter < 100; iter++) hall_widths_update(w, sums, 3);
    int32_t sum = 0;
    for (int i = 0; i < 6; i++) sum += w[i];
    /* Σ = 360<<8 = 92160; ASR drift on negative deltas accumulates; brief tol = 50 */
    EXPECT_WITHIN(sum, 92160, 50);
}

int run_hall_cal_tests(void) {
    test_hall_uniform_leaves_nominal();
    test_hall_total_zero_guard();
    test_hall_ema_shift_zero_snaps();
    test_hall_skewed_converges();
    test_hall_ema_one_step_magnitude();
    test_hall_sum_near_invariant();
    return 0;
}

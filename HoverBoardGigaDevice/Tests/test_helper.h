#ifndef TEST_HELPER_H
#define TEST_HELPER_H

#include <cmocka.h>

#include "mathDefines.h"

typedef struct
{
    uint8_t hall_a;
    uint8_t hall_b;
    uint8_t hall_c;
    uint32_t expected_pulse_a;
    uint32_t expected_pulse_b;
    uint32_t expected_pulse_c;
} pwm_test_case_t;

static void assert_int_within(int32_t expected, int32_t actual, int32_t margin) {
    int32_t diff = ABS(expected - actual);
    if (diff > margin) {
        printf("Expected: %d, Actual: %d, Margin: %d (|diff|=%d)\n", expected, actual, margin, diff);
    } 
    assert_true(diff <= margin);
}

static void run_pwm_table_tests(int duty, const pwm_test_case_t *cases, size_t num_cases, int error_margin)
{
    for (size_t i = 0; i < num_cases; ++i)
    {
        uint8_t sector = get_sector(cases[i].hall_a, cases[i].hall_b, cases[i].hall_c);
        uint32_t a = 0, b = 0, c = 0;
        get_pwm(duty, sector, &a, &b, &c);
       // printf("{ %d, %d, %d, %d, %d, %d },\n", cases[i].hall_a, cases[i].hall_b, cases[i].hall_c, a, b, c);
       // printf("{ %d, %d, %d, %d, %d, %d },\n", cases[i].hall_a, cases[i].hall_b, cases[i].hall_c, a - cases[i].expected_pulse_a, b - cases[i].expected_pulse_b, c - cases[i].expected_pulse_c);
     
        fflush(stdout);
        assert_int_within((int32_t)cases[i].expected_pulse_a, (int32_t)a, (int32_t)error_margin);
        assert_int_within((int32_t)cases[i].expected_pulse_b, (int32_t)b, (int32_t)error_margin);
        assert_int_within((int32_t)cases[i].expected_pulse_c, (int32_t)c, (int32_t)error_margin);
    }
}


#endif
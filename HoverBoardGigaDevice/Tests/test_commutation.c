#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <stdio.h>
#include "../Inc/commutation.h"

typedef struct
{
    uint8_t hall_a;
    uint8_t hall_b;
    uint8_t hall_c;
    uint32_t expected_pulse_a;
    uint32_t expected_pulse_b;
    uint32_t expected_pulse_c;
} pwm_test_case_t;

static void test_get_sector_hall_patterns(void **state)
{
    (void)state;
    assert_int_equal(1, get_sector(0, 0, 1));
    assert_int_equal(2, get_sector(1, 0, 1));
    assert_int_equal(3, get_sector(1, 0, 0));
    assert_int_equal(4, get_sector(1, 1, 0));
    assert_int_equal(5, get_sector(0, 1, 0));
    assert_int_equal(6, get_sector(0, 1, 1));
}

static void run_pwm_table_tests(int duty, const pwm_test_case_t *cases, size_t num_cases)
{
    for (size_t i = 0; i < num_cases; ++i)
    {
        uint8_t sector = get_sector(cases[i].hall_a, cases[i].hall_b, cases[i].hall_c);
        uint32_t a = 0, b = 0, c = 0;
        get_pwm(duty, sector, &a, &b, &c);
        //printf("{ %u, %u, %u, %u, %u, %u },\n", cases[i].hall_a, cases[i].hall_b, cases[i].hall_c, a, b, c);
        //fflush(stdout);
        assert_int_equal(cases[i].expected_pulse_a, a);
        assert_int_equal(cases[i].expected_pulse_b, b);
        assert_int_equal(cases[i].expected_pulse_c, c);
    }
}

static void test_get_pwm_table_duty_0(void **state)
{
    (void)state;
    const int duty = 0;
    pwm_test_case_t cases[] = {
        {0, 0, 1, 1125, 1125, 1125},
        {1, 0, 1, 1125, 1125, 1125},
        {1, 0, 0, 1125, 1125, 1125},
        {1, 1, 0, 1125, 1125, 1125},
        {0, 1, 0, 1125, 1125, 1125},
        {0, 1, 1, 1125, 1125, 1125}};
    run_pwm_table_tests(duty, cases, sizeof(cases) / sizeof(cases[0]));
}

static void test_get_pwm_table_duty_25_percent(void **state)
{
    (void)state;
    const int duty = 1125 * 25 / 100;
    pwm_test_case_t cases[] = {
        {0, 0, 1, 1406, 1125, 844},
        {1, 0, 1, 1406, 844, 1125},
        {1, 0, 0, 1125, 844, 1406},
        {1, 1, 0, 844, 1125, 1406},
        {0, 1, 0, 844, 1406, 1125},
        {0, 1, 1, 1125, 1406, 844},
    };
    run_pwm_table_tests(duty, cases, sizeof(cases) / sizeof(cases[0]));
}

static void test_get_pwm_table_duty_negative_25_percent(void **state)
{
    (void)state;
    const int duty = -1125 * 25 / 100;
    pwm_test_case_t cases[] = {
        {0, 0, 1, 844, 1125, 1406},
        {1, 0, 1, 844, 1406, 1125},
        {1, 0, 0, 1125, 1406, 844},
        {1, 1, 0, 1406, 1125, 844},
        {0, 1, 0, 1406, 844, 1125},
        {0, 1, 1, 1125, 844, 1406},
    };
    run_pwm_table_tests(duty, cases, sizeof(cases) / sizeof(cases[0]));
}

static void test_get_pwm_table_duty_75_percent(void **state)
{
    (void)state;
    const int duty = 1125 * 75 / 100;
    pwm_test_case_t cases[] = {
        {0, 0, 1, 1968, 1125, 282},
        {1, 0, 1, 1968, 282, 1125},
        {1, 0, 0, 1125, 282, 1968},
        {1, 1, 0, 282, 1125, 1968},
        {0, 1, 0, 282, 1968, 1125},
        {0, 1, 1, 1125, 1968, 282},
    };
    run_pwm_table_tests(duty, cases, sizeof(cases) / sizeof(cases[0]));
}

static void test_get_pwm_table_duty_negative_75_percent(void **state)
{
    (void)state;
    const int duty = -1125 * 75 / 100;
    pwm_test_case_t cases[] = {
        {0, 0, 1, 282, 1125, 1968},
        {1, 0, 1, 282, 1968, 1125},
        {1, 0, 0, 1125, 1968, 282},
        {1, 1, 0, 1968, 1125, 282},
        {0, 1, 0, 1968, 282, 1125},
        {0, 1, 1, 1125, 282, 1968},
    };
    run_pwm_table_tests(duty, cases, sizeof(cases) / sizeof(cases[0]));
}

static void test_get_pwm_table_duty_100_percent(void **state)
{
    (void)state;
    const int duty = 1125;
    pwm_test_case_t cases[] = {
        {0, 0, 1, 2240, 1125, 10},
        {1, 0, 1, 2240, 10, 1125},
        {1, 0, 0, 1125, 10, 2240},
        {1, 1, 0, 10, 1125, 2240},
        {0, 1, 0, 10, 2240, 1125},
        {0, 1, 1, 1125, 2240, 10},
    };
    run_pwm_table_tests(duty, cases, sizeof(cases) / sizeof(cases[0]));
}

static void test_get_pwm_table_duty_negative_100_percent(void **state)
{
    (void)state;
    const int duty = -1125;
    pwm_test_case_t cases[] = {
        {0, 0, 1, 10, 1125, 2240},
        {1, 0, 1, 10, 2240, 1125},
        {1, 0, 0, 1125, 2240, 10},
        {1, 1, 0, 2240, 1125, 10},
        {0, 1, 0, 2240, 10, 1125},
        {0, 1, 1, 1125, 10, 2240},
    };
    run_pwm_table_tests(duty, cases, sizeof(cases) / sizeof(cases[0]));
}

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_get_sector_hall_patterns),
        cmocka_unit_test(test_get_pwm_table_duty_0),
        cmocka_unit_test(test_get_pwm_table_duty_25_percent),
        cmocka_unit_test(test_get_pwm_table_duty_negative_25_percent),
        cmocka_unit_test(test_get_pwm_table_duty_75_percent),
        cmocka_unit_test(test_get_pwm_table_duty_negative_75_percent),
        cmocka_unit_test(test_get_pwm_table_duty_100_percent),
        cmocka_unit_test(test_get_pwm_table_duty_negative_100_percent),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
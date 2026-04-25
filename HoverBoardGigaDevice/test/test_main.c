#include <stdio.h>
#include "test_helpers.h"

int test_failed = 0;

int run_sincos_tests(void);
int run_transforms_tests(void);
int run_bc_tests(void);
int run_pll_tests(void);
int run_pi_tests(void);
int run_dtc_tests(void);
int run_hall_cal_tests(void);

int main(void) {
    run_sincos_tests();
    run_transforms_tests();
    run_bc_tests();
    run_pll_tests();
    run_pi_tests();
    run_dtc_tests();
    run_hall_cal_tests();
    if (test_failed) {
        fprintf(stderr, "FAIL\n");
        return 1;
    }
    printf("OK\n");
    return 0;
}

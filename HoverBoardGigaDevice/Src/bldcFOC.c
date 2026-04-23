/* Stub implementation. The real FOC lands in a later commit; this file
 * exists here only so the project links at this point in the history. */

#include "../Inc/bldcFOC.h"

#ifdef BLDC_FOC

void InitBldc(void) { }

void bldc_get_pwm(int pwm, int pos, int *y, int *b, int *g) {
    (void)pwm; (void)pos;
    *y = 0; *b = 0; *g = 0;
}

#endif

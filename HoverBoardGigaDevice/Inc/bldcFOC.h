#ifndef BLDCFOC_H
#define BLDCFOC_H

#include "bldc.h"

// bldcFOC.c implements the two virtual methods declared in bldc.h:
//   void InitBldc(void);
//   void bldc_get_pwm(int pwm, int pos, int *y, int *b, int *g);
// All FOC internals (PLL, Clarke/Park, PI, SVPWM, sine table) are
// file-local statics in bldcFOC.c — nothing else leaks into the API.

#endif // BLDCFOC_H

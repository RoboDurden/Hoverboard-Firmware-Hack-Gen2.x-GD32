#ifndef BLDCFOC_H
#define BLDCFOC_H

#include "bldc.h"

// Run one per-ISR sensor update cycle: advance angle PLL, read phase
// currents, Clarke/Park, seed/update back-EMF observer, and accumulate
// averages. Called from bldc.c::CalculateBLDC.
void foc_sensor_update(uint8_t pos, volatile adc_buf_t *adc);

#endif // BLDCFOC_H

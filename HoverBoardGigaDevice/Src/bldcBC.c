#include "../Inc/bldcBC.h"
#ifdef BLDC_BC

//----------------------------------------------------------------------------
// Block PWM calculation based on position
//----------------------------------------------------------------------------
void bldc_get_pwm(int pwm, int pos, int *y, int *b, int *g)
{
  switch(pos)
	{
    case 1:
      *y = 0;
      *b = pwm;
      *g = -pwm;
      break;
    case 2:
      *y = -pwm;
      *b = pwm;
      *g = 0;
      break;
    case 3:
      *y = -pwm;
      *b = 0;
      *g = pwm;
      break;
    case 4:
      *y = 0;
      *b = -pwm
		;
      *g = pwm;
      break;
    case 5:
      *y = pwm;
      *b = -pwm;
      *g = 0;
      break;
    case 6:
      *y = pwm;
      *b = 0;
      *g = -pwm;
      break;
    default:
      *y = 0;
      *b = 0;
      *g = 0;
  }	
}

void InitBldc()
{
}
#endif
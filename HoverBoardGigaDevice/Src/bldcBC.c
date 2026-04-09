#include "../Inc/bldcBC.h"
#include "../Inc/foc.h"
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

extern const uint8_t hall_to_pos[8];

void InitBldc()
{
	extern FOC_Angle foc_angle;
	extern FOC_Controller foc_ctrl;
	extern uint16_t foc_offset_y, foc_offset_b;
	extern adc_buf_t adc_buffer;

	foc_angle_init(&foc_angle);
	foc_controller_init(&foc_ctrl);

	#if defined(PHASE_CURRENT_Y) && defined(PHASE_CURRENT_B)
		foc_calibrate_offsets(&foc_offset_y, &foc_offset_b,
		                      &adc_buffer.phase_current_y, &adc_buffer.phase_current_b);
	#endif

	#ifdef FOC_ENABLED
		// Self-calibrate angle offset by aligning rotor to a known electrical angle.
		// Required because hall sensor placement varies between motor designs.
		foc_angle.angle_offset = foc_align_rotor((uint8_t *)hall_to_pos);
	#endif
}
#endif
#ifndef CT5_FX_CT5_H
#define CT5_FX_CT5_H

#include "../hope_hal/hope_dsp_interface.h"

void ct5_fx_ct5( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
void ct5_fx_mute( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
void ct5_fx_pass_through( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );


void ct5_fx_noise_gate( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

#endif



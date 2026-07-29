#ifndef CT5_FX_M1C_H
#define CT5_FX_M1C_H

#include "../hope_hal/hope_dsp_interface.h"

void ct5_fx_m1c( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

typedef struct{
	uint32_t m1c_n_switch;
	float wet_gain;
	float dry_gain;
	float desired_dir;
	float buffer_size;
	float feedback;

}m1c_variables_t;
#ifndef CT5_FX_M2R_H
#define CT5_FX_M2R_H

#include "../hope_hal/hope_dsp_interface.h"

void ct5_fx_m2r( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

//we need to make  a struct or typedef that holds all the variables that m2r uses. then we ask an
//external function to initialize the struct.

typedef struct{
	uint32_t m2r_n_switch;
	float wet_gain;
	float dry_gain;
	float desired_dir;
	float playback_slice_length;
	float playback_start_randomization;	
}m2r_variables_t;

#endif
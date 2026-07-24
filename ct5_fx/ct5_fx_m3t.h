#ifndef CT5_FX_M3T_H
#define CT5_FX_M3T_H

#include "../hope_hal/hope_dsp_interface.h"

void ct5_fx_m3t( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

typedef struct{
	uint32_t m3t_n_switch;
	float wet_gain;
	float dry_gain;
	float desired_dir;
	float playback_volume;
	float overdub_decay;
	uint32_t is_recording;		
}m3t_variables_t;

#endif
#ifndef CT5_FX_M3D_H
#define CT5_FX_M3D_H

#include "../hope_hal/hope_dsp_interface.h"

void ct5_fx_m3d( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output, hope_dsp_control_struct * ctl );


typedef struct{
	uint32_t m3d_n_switch;
	float wet_gain;
	float dry_gain;
	float desired_dir;
    float current_dir;
	float playback_volume;
	uint32_t is_recording;		
    float read_head_address[2];
    float current_double_space_value;
    float desired_double_space_value;

}m3d_variables_t;

#endif
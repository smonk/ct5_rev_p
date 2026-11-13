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
	float double_space_coefficient;
	uint32_t double_space_coefficient_has_updated;
	uint32_t double_space_coefficient_latched;
    float current_double_space_value;
    float desired_double_space_value;
	float double_space_value_increment;
	uint32_t double_sample_ended_flag;
	uint32_t double_sample_restart_request_flag;
	// uint32_t double_space_integer_offset;
	float double_space_offset;
	float read_head_last_block_end_address_ds;
}m3d_variables_t;

#endif
#ifndef CT5_FX_M1C_H
#define CT5_FX_M1C_H

#include "../hope_hal/hope_dsp_interface.h"

void ct5_fx_m1c( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

typedef struct{
	uint32_t m1c_n_switch;
	float wet_gain;
	float dry_gain;
	float desired_dir;
	uint32_t current_buffer_size;  //in samples
	uint32_t desired_buffer_size;
	float current_feedback_volume; //what was the feedback volume when you leave the algo
	float desired_feedback_volume; //what is the desired feedback volume from the pot
	float desired_feedback_volume_override;
	uint32_t use_feedback_volume_override;
	float current_write_head_volume;
	float desired_write_head_volume;
}m1c_variables_t;

#endif
/*
	feedback volume can come from a pot. 
	feedback volume must also be able to be overridden by the state machine. 
	this override volume should be selectable by another variable

*/
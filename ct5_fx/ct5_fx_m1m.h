#ifndef CT5_FX_M1M_H
#define CT5_FX_M1M_H

#include "../hope_hal/hope_dsp_interface.h"

void ct5_fx_m1m( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

#define CT5_M1M_N_READ_HEADS 3

typedef struct{
	uint32_t current_n;
	uint32_t desired_n;
	float wet_gain;	
	float dry_gain;

    uint32_t current_buffer_size;  //in samples
	uint32_t desired_buffer_size;

	float current_write_head_volume;
	float desired_write_head_volume;

    uint32_t hold_mode;
	
    float desired_dir[CT5_M1M_N_READ_HEADS];
	
    float current_feedback_volume[CT5_M1M_N_READ_HEADS]; //what was the feedback volume when you leave the algo
	float desired_feedback_volume[CT5_M1M_N_READ_HEADS]; //what is the desired feedback volume from the pot
	float desired_feedback_volume_override[CT5_M1M_N_READ_HEADS];
	uint32_t use_feedback_volume_override[CT5_M1M_N_READ_HEADS];
	
    float current_output_volume[CT5_M1M_N_READ_HEADS];
	float desired_output_volume[CT5_M1M_N_READ_HEADS];

    float float_read_head_address[CT5_M1M_N_READ_HEADS];

        
}m1m_variables_t;

#endif
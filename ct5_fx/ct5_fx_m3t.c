#include "ct5_buffer.h"
#include "ct5_fx_m3t.h"
#include "../hope_hal/hope_dsp_interface.h"
#include "../hope_hal/hope_pwm_rgb_led.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "../hope_hal/hope_btn_and_sw.h"
#include "../hope_hal/hope_version.h"

#include "ct5_fx_state_machine.h"
#include "ct5_fx_ctl_input.h"

// ███╗   ███╗██████╗ ████████╗
// ████╗ ████║╚════██╗╚══██╔══╝
// ██╔████╔██║ █████╔╝   ██║   
// ██║╚██╔╝██║ ╚═══██╗   ██║   
// ██║ ╚═╝ ██║██████╔╝   ██║   
// ╚═╝     ╚═╝╚═════╝    ╚═╝   

#define M3T_N_BUFFERS 3

//big font from
//https://www.asciiart.eu/text-to-ascii-art
//used 'ansi shadow' font

static ct5_buffer_t * bufs[M3T_N_BUFFERS];
 
static ct5_buffer_t ct5_buffer_a;
static ct5_buffer_t ct5_buffer_b;
static ct5_buffer_t ct5_buffer_c;

static m3t_variables_t m3t_variables_a;
static m3t_variables_t m3t_variables_b;
static m3t_variables_t m3t_variables_c;

static m3t_variables_t m3t_variables_global;
m3t_variables_t * my_m3t_variables;

void ct5_fx_m3t_init( void );

state_function_pointer_t m3t_state_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m3t_state_reset_to_record( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m3t_state_record( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m3t_state_record_to_playback( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m3t_state_playback( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m3t_state_playback_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m3t_state_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m3t_state_overdub_to_playback( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m3t_state_overdub_to_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

uint32_t m3t_event_record_button_pressed( void );
uint32_t m3t_event_record_button_was_tapped( void );
uint32_t m3t_event_record_button_was_held( void );
uint32_t m3t_event_recording_is_wrapped( void );

void m3t_helper_zero_buffer( hope_dsp_buffer_struct * input );
void m3t_helper_wet_dry_mix( hope_dsp_buffer_struct * input_dry, float dry_gain, hope_dsp_buffer_struct * input_wet, float wet_gain, hope_dsp_buffer_struct * output );
void m3t_helper_record_block( ct5_buffer_t * buf, hope_dsp_buffer_struct * input);
void m3t_helper_record_index_to_index( ct5_buffer_t * buf, hope_dsp_buffer_struct * input, uint32_t start_index, uint32_t end_index );
void m3t_helper_overdub_block( ct5_buffer_t * buf, hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
void m3t_helper_overdub_index_to_index( ct5_buffer_t * buf, hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output,  uint32_t start_index, uint32_t end_index );
void m3t_helper_playback_block( ct5_buffer_t * buf,  hope_dsp_buffer_struct * output );
uint32_t m3t_helper_destructive_scan_for_zero_crossing( hope_dsp_buffer_struct * input, uint32_t * left_zc_index, uint32_t * right_zc_index );
uint32_t m3t_helper_destructive_reverse_scan_for_zero_crossing( hope_dsp_buffer_struct * input, uint32_t * left_zc_index, uint32_t * right_zc_index );
void m3t_helper_copy_buffer( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output);
void m3t_helper_block_length_fade_in( hope_dsp_buffer_struct *output );
void m3t_helper_block_length_fade_out( hope_dsp_buffer_struct * output );
void m3t_helper_append_block( ct5_buffer_t * buf , hope_dsp_buffer_struct * input );
void m3t_helper_add_two_dsp_buffers( hope_dsp_buffer_struct * input_1, hope_dsp_buffer_struct * input_2, hope_dsp_buffer_struct * output );

state_function_pointer_t track_0_sfp = {0};
state_function_pointer_t track_1_sfp = {0};
state_function_pointer_t track_2_sfp = {0};

static uint32_t m3t_track_n_processing = 0;
static uint32_t m3t_track_n_highlighted = 0;

static uint32_t m3t_track_n_change_lock = 0;

// ██╗███╗   ██╗██╗████████╗
// ██║████╗  ██║██║╚══██╔══╝
// ██║██╔██╗ ██║██║   ██║   
// ██║██║╚██╗██║██║   ██║   
// ██║██║ ╚████║██║   ██║   
// ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝   

void ct5_fx_m3t_init( )
{
	bufs[0] = &ct5_buffer_a;
	bufs[1] = &ct5_buffer_b;
	bufs[2] = &ct5_buffer_c;

	bufs[0]->m3t_variables = &m3t_variables_a;
	bufs[1]->m3t_variables = &m3t_variables_b;
	bufs[2]->m3t_variables = &m3t_variables_c;

	my_m3t_variables = &m3t_variables_global;


	uint32_t channel_buffer_size = CT5_BUFFER_MEM_SIZE_WORDS / 6;

	for( int i = 0; i < M3T_N_BUFFERS; i++)
	{
		bufs[i]->left_channel_physical_memory_start_address = ( float * )CT5_BUFFER_MEM_BASE_ADDRESS;
		bufs[i]->left_channel_physical_memory_start_address += i * 2 * channel_buffer_size;

		bufs[i]->left_channel_physical_memory_end_address = ( float * )CT5_BUFFER_MEM_BASE_ADDRESS;
		bufs[i]->left_channel_physical_memory_end_address += ( i * 2 * channel_buffer_size + channel_buffer_size - 1 );

		bufs[i]->right_channel_physical_memory_start_address = bufs[i]->left_channel_physical_memory_start_address + channel_buffer_size;
		bufs[i]->right_channel_physical_memory_end_address = bufs[i]->left_channel_physical_memory_end_address + channel_buffer_size;
	
		bufs[i]->buffer_size = channel_buffer_size;

		bufs[i]->block_size = HOPE_DSP_BUFFER_SIZE;

		bufs[i]->float_read_head_address = 0;

		bufs[i]->integer_write_head_address = 0;

		bufs[i]->desired_dir = 1.33;
		bufs[i]->current_dir = 1.0;
		bufs[i]->dir_increment = 0.0005;

		bufs[i]->playback_start = 0;
		bufs[i]->playback_end = 0;
		bufs[i]->recording_wrapped = 0;

		bufs[i]->desired_playback_volume = 1.0;
		bufs[i]->current_playback_volume = 1.0;
		bufs[i]->playback_volume_increment = 0.0005;		

	}

	track_0_sfp.next_state = m3t_state_reset;
	track_1_sfp.next_state = m3t_state_reset;
	track_2_sfp.next_state = m3t_state_reset;

	m3t_track_n_change_lock = 0;

}

// ███╗   ███╗ █████╗ ██╗███╗   ██╗
// ████╗ ████║██╔══██╗██║████╗  ██║
// ██╔████╔██║███████║██║██╔██╗ ██║
// ██║╚██╔╝██║██╔══██║██║██║╚██╗██║
// ██║ ╚═╝ ██║██║  ██║██║██║ ╚████║
// ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝

void ct5_fx_m3t( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	static uint32_t is_init = 0;
	if(!is_init)
	{
		ct5_fx_m3t_init();
		is_init = 1;
		return;
	}

	// // make the audio mono for debugging
	// for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	// {
	// 	input->right_channel_buffer[i] = input->left_channel_buffer[i];
	// }
	

	//this is a lock to block n switching when any track is recording.
	ct5_fx_get_m3t_variables( my_m3t_variables );
	m3t_track_n_change_lock = 0;
	for(uint32_t i = 0; i < M3T_N_BUFFERS; i++)
	{
		if( bufs[i]->m3t_variables->is_recording )
		{
			m3t_track_n_change_lock = 1;
		}
	}
	if( m3t_track_n_change_lock == 0 )
	{
		m3t_track_n_highlighted = my_m3t_variables->m3t_n_switch - 1;	
	}

	//zero the output buffer
	m3t_helper_zero_buffer( output );

	hope_dsp_buffer_struct temp_buf;
	temp_buf.num_samples_per_channel = HOPE_DSP_BUFFER_SIZE;
	float temp_left_channel_buffer[HOPE_DSP_BUFFER_SIZE];
	float temp_right_channel_buffer[HOPE_DSP_BUFFER_SIZE];
	temp_buf.left_channel_buffer = temp_left_channel_buffer;
	temp_buf.right_channel_buffer = temp_right_channel_buffer;


	// now each sfp processed adds to the output buffer
	m3t_helper_zero_buffer( &temp_buf );
	m3t_track_n_processing = 0;
	if( m3t_track_n_highlighted == 0 )
	{
		ct5_fx_get_m3t_variables( bufs[0]->m3t_variables );
	}
	track_0_sfp = track_0_sfp.next_state( input, &temp_buf );
	m3t_helper_add_two_dsp_buffers( output, &temp_buf, output );

	m3t_helper_zero_buffer( &temp_buf );
	m3t_track_n_processing = 1;
	if( m3t_track_n_highlighted == 1 )
	{
		ct5_fx_get_m3t_variables( bufs[1]->m3t_variables );
	}
	track_1_sfp = track_1_sfp.next_state( input, &temp_buf );
	m3t_helper_add_two_dsp_buffers( output, &temp_buf, output );
	
	m3t_helper_zero_buffer( &temp_buf );
	m3t_track_n_processing = 2;
	if( m3t_track_n_highlighted == 2 )
	{
		ct5_fx_get_m3t_variables( bufs[2]->m3t_variables );
	}
	track_2_sfp = track_2_sfp.next_state( input, &temp_buf );
	m3t_helper_add_two_dsp_buffers( output, &temp_buf, output );
	

	//now at the end we need to mix the input and output accordingly.
	float wet_gain, dry_gain;
	wet_gain = my_m3t_variables->wet_gain;
	dry_gain = my_m3t_variables->dry_gain;

	m3t_helper_wet_dry_mix( input, dry_gain, output, wet_gain, output );

}

// ███████╗████████╗ █████╗ ████████╗███████╗███████╗
// ██╔════╝╚══██╔══╝██╔══██╗╚══██╔══╝██╔════╝██╔════╝
// ███████╗   ██║   ███████║   ██║   █████╗  ███████╗
// ╚════██║   ██║   ██╔══██║   ██║   ██╔══╝  ╚════██║
// ███████║   ██║   ██║  ██║   ██║   ███████╗███████║
// ╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚══════╝

extern void (*hope_ct5_led_on_setting)( hope_pwm_rgb_led_struct * );
extern void (*hope_ct5_led_off_setting)( hope_pwm_rgb_led_struct * );

state_function_pointer_t m3t_state_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//only reset the buffer that the fsm controlls
	bufs[m3t_track_n_processing]->float_read_head_address = 0;
	bufs[m3t_track_n_processing]->integer_write_head_address = 0;

	bufs[m3t_track_n_processing]->playback_start = 0;
	bufs[m3t_track_n_processing]->playback_end = 0;
	bufs[m3t_track_n_processing]->recording_wrapped = 0;


	//now check the events
	//else we stay in this state

	state_function_pointer_t sfp;

	//if the buffer is the active buffer
	//then interpret the event, otherwise stay in reset state
	if( m3t_track_n_processing == m3t_track_n_highlighted )
	{
		hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_blinking_green;
		if( m3t_event_record_button_pressed() )
		{
			//go to mtr_state_rec_overdub_1_max
			//first we need a struct to put the pointer in
			sfp.next_state = m3t_state_reset_to_record;
			return sfp;
		}
		else
		{
			//the clean sound is added in the parent function
			//so there is nothing to do here.

			//blink an led?

			sfp.next_state = m3t_state_reset;
			return sfp;
		}		
	}
	else
	{
		sfp.next_state = m3t_state_reset;
		return sfp;
	}

}
uint32_t reset_to_record_miss = 0;

state_function_pointer_t m3t_state_reset_to_record( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//in this state we want to prepare the input vector to be pop free. 

	hope_dsp_buffer_struct temp;
	float temp_left_buffer[ HOPE_DSP_BUFFER_SIZE ];
	float temp_right_buffer[ HOPE_DSP_BUFFER_SIZE ];
	temp.left_channel_buffer = temp_left_buffer;
	temp.right_channel_buffer = temp_right_buffer;

	m3t_helper_copy_buffer( input, &temp );

	//fade in temp
	m3t_helper_block_length_fade_in( &temp );

	bufs[ m3t_track_n_processing ]->m3t_variables->is_recording = 1;	
	m3t_helper_record_block( bufs[ m3t_track_n_processing ], &temp );

	//go to m3t_state_record
	state_function_pointer_t sfp;
	sfp.next_state = m3t_state_record;
	return sfp;


}



state_function_pointer_t m3t_state_record( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//in this state you can start recording to the buffer once an appropriate sample is found.

	//do the thing
	m3t_helper_record_block( bufs[ m3t_track_n_processing ], input );

	//when the button gets released it is either a tap or a hold.
	//so move the next state accordingly
	state_function_pointer_t sfp;
	if( m3t_track_n_processing == m3t_track_n_highlighted )
	{
		hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_red;

		if( m3t_event_record_button_was_tapped() )
		{
			//go to m3t_state_reset

			sfp.next_state = m3t_state_reset;
			return sfp;
		}
		else if( m3t_event_record_button_was_held() )
		{

		
			sfp.next_state = m3t_state_record_to_playback;
			return sfp;
		}
		else if( m3t_event_recording_is_wrapped() )
		{

			sfp.next_state = m3t_state_overdub;
			return sfp;
		}
		else
		{


			sfp.next_state = m3t_state_record;
			return sfp;
		}
	}
	else
	{
		sfp.next_state = m3t_state_record;
		return sfp;
	}
}

state_function_pointer_t m3t_state_record_to_playback( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//we need to trim the end of the recording while starting playback.

	hope_dsp_buffer_struct temp;
	float temp_left_buffer[ HOPE_DSP_BUFFER_SIZE ];
	float temp_right_buffer[ HOPE_DSP_BUFFER_SIZE ];
	temp.left_channel_buffer = temp_left_buffer;
	temp.right_channel_buffer = temp_right_buffer;

	m3t_helper_copy_buffer( input, &temp );

	m3t_helper_block_length_fade_out( &temp );

	m3t_helper_record_block( bufs[ m3t_track_n_processing ] , &temp  );

	bufs[ m3t_track_n_processing ]->m3t_variables->is_recording = 0;	

	//go to m3t_state_playback
	if( bufs[ m3t_track_n_processing ]->recording_wrapped == 0 )
	{
		if( bufs[ m3t_track_n_processing ]->integer_write_head_address == 0 )
		{
			bufs[ m3t_track_n_processing ]->playback_end = (bufs[ m3t_track_n_processing ]->buffer_size - 1);
		}
		else
		{
			bufs[ m3t_track_n_processing ]->playback_end = bufs[ m3t_track_n_processing ]->integer_write_head_address -1;
		}

		// }
	
	}
	else
	{
		//this should never execute because it would be in overdub if the space was wrapped?
		// m3t_helper_record_block( bufs[ m3t_track_n_processing ] , &temp  );
		// bufs[ m3t_track_n_processing ]->playback_end = bufs[ m3t_track_n_processing ]->buffer_size - 1;
	}
	state_function_pointer_t sfp;
	sfp.next_state = m3t_state_playback;
	return sfp;

}



uint32_t record_to_playback_miss = 0;


state_function_pointer_t m3t_state_playback( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//this needs to not be a destructive write (ie just add to output)
	m3t_helper_playback_block( bufs[ m3t_track_n_processing ], output );


	//in this state you can start playback from the buffer once an appropriate sample is found.	

	//we exit this state if the button was pressed.
	state_function_pointer_t sfp;
	if( m3t_track_n_processing == m3t_track_n_highlighted )
	{

		hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_blue;
		if( m3t_event_record_button_pressed() )
		{
			//fade out the playback buffer?
			//we face this out because otherwise there is a guarunteed discontinuity based on 
			//the all of a sudden movement of the write address head. 
			//it is possible just some smarter math can solve this without the need to fade.
			m3t_helper_block_length_fade_out( output );
			bufs[ m3t_track_n_processing ]->integer_write_head_address = (uint32_t)( bufs[ m3t_track_n_processing ]->float_read_head_address );
			sfp.next_state = m3t_state_playback_to_overdub;
			return sfp;
		}
		else
		{
			sfp.next_state = m3t_state_playback;
			return sfp;
		}
	}
	else
	{
		sfp.next_state = m3t_state_playback;
		return sfp;		
	}
}

state_function_pointer_t m3t_state_playback_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//need to keep playing what is playing and add the lead in
	//do the thing

	hope_dsp_buffer_struct temp;
	float temp_left_buffer[ HOPE_DSP_BUFFER_SIZE ];
	float temp_right_buffer[ HOPE_DSP_BUFFER_SIZE ];
	temp.left_channel_buffer = temp_left_buffer;
	temp.right_channel_buffer = temp_right_buffer;


	m3t_helper_copy_buffer( input, &temp );

	m3t_helper_block_length_fade_in( &temp );
	bufs[ m3t_track_n_processing ]->m3t_variables->is_recording = 1;	

	m3t_helper_overdub_block( bufs[ m3t_track_n_processing ] , &temp, output );
	// m3t_helper_record_block( bufs[ m3t_track_n_processing ] , &temp  );
	//go to m3t_state_record
	state_function_pointer_t sfp;
	sfp.next_state = m3t_state_overdub;
	return sfp;

}



state_function_pointer_t m3t_state_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{


	//in overdub we need to read from the buffer and add that to the output as well as record the input.
	//this means there could be collisions. So we need to add collision detection here/in the helper funciton.
	
	m3t_helper_overdub_block( bufs[ m3t_track_n_processing ], input, output );
	// m3t_helper_record_block( bufs[ m3t_track_n_processing ] , input  );

	//in this state you can start overdubbing from the buffer once an appropriate sample is found.
	state_function_pointer_t sfp;

	//exit on button releases, either to reset or to playback
	if( m3t_track_n_processing == m3t_track_n_highlighted )
	{
		hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_green;
		if( m3t_event_record_button_was_tapped() )
		{
			//go to m3t_state_reset

			sfp.next_state = m3t_state_overdub_to_reset;
			return sfp;
		}
		else if( m3t_event_record_button_was_held() )
		{
					
			sfp.next_state = m3t_state_overdub_to_playback;
			return sfp;
		}
		else
		{

			sfp.next_state = m3t_state_overdub;
			return sfp;
		}
	}
	else
	{
		sfp.next_state = m3t_state_overdub;
		return sfp;
	}
}
state_function_pointer_t m3t_state_overdub_to_playback( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{


	hope_dsp_buffer_struct temp;
	float temp_left_buffer[ HOPE_DSP_BUFFER_SIZE ];
	float temp_right_buffer[ HOPE_DSP_BUFFER_SIZE ];
	temp.left_channel_buffer = temp_left_buffer;
	temp.right_channel_buffer = temp_right_buffer;

	m3t_helper_copy_buffer( input, &temp );

	m3t_helper_block_length_fade_out( &temp );

		m3t_helper_overdub_block( bufs[ m3t_track_n_processing ] , &temp , output );
	// m3t_helper_record_block( bufs[ m3t_track_n_processing ] , &temp  );
	bufs[ m3t_track_n_processing ]->m3t_variables->is_recording = 0;	



		state_function_pointer_t sfp;
		sfp.next_state = m3t_state_playback;
		return sfp;

}

state_function_pointer_t m3t_state_overdub_to_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//just do as overdub normally would but then zerocross check that. 
	//or do we want to gauruntee it ends in 1 frame with a fade?
	m3t_helper_overdub_block( bufs[ m3t_track_n_processing ], input, output );

	m3t_helper_block_length_fade_out( output );
	bufs[ m3t_track_n_processing ]->m3t_variables->is_recording = 0;	

	state_function_pointer_t sfp;
	sfp.next_state = m3t_state_reset;
	return sfp;


}

// ███████╗██╗   ██╗███████╗███╗   ██╗████████╗███████╗
// ██╔════╝██║   ██║██╔════╝████╗  ██║╚══██╔══╝██╔════╝
// █████╗  ██║   ██║█████╗  ██╔██╗ ██║   ██║   ███████╗
// ██╔══╝  ╚██╗ ██╔╝██╔══╝  ██║╚██╗██║   ██║   ╚════██║
// ███████╗ ╚████╔╝ ███████╗██║ ╚████║   ██║   ███████║
// ╚══════╝  ╚═══╝  ╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝


extern hope_btn_and_sw_struct my_btn_and_sw[ HOPE_NUM_BTN_AND_SW ];

uint32_t m3t_event_record_button_pressed( )
{
	//check a specific button and return 1 if it is pressed. 0 if not.
	//this is hardcoded for now
	if( my_btn_and_sw[6].pressed_event_flag == 1 )
	{
		my_btn_and_sw[6].pressed_event_flag = 0;
		return 1;
	}
	return 0;
}

uint32_t m3t_event_record_button_was_tapped( )
{
	//check a specific button and return 1 if it was tapped. 0 if not.
	if( my_btn_and_sw[6].tap_event_flag == 1 )
	{
		my_btn_and_sw[6].tap_event_flag = 0;
		return 1;
	}
	return 0;
}

uint32_t m3t_event_record_button_was_held( )
{
	//check a specific button and return 1 if it was held. 0 if not.	
	if( my_btn_and_sw[6].hold_event_flag == 1 )
	{
		my_btn_and_sw[6].hold_event_flag = 0;
		return 1;
	}

	return 0;
}

uint32_t m3t_event_recording_is_wrapped( )
{
	if( bufs[ m3t_track_n_processing ]->recording_wrapped == 1 )	
	{
		// bufs[ m3t_track_n_processing ]->recording_wrapped = 0;
		return 1;
	}
	return 0;
}

// ██╗  ██╗███████╗██╗     ██████╗ ███████╗██████╗ ███████╗
// ██║  ██║██╔════╝██║     ██╔══██╗██╔════╝██╔══██╗██╔════╝
// ███████║█████╗  ██║     ██████╔╝█████╗  ██████╔╝███████╗
// ██╔══██║██╔══╝  ██║     ██╔═══╝ ██╔══╝  ██╔══██╗╚════██║
// ██║  ██║███████╗███████╗██║     ███████╗██║  ██║███████║
// ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝╚══════╝

void m3t_helper_zero_buffer( hope_dsp_buffer_struct * input )
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		input->left_channel_buffer[i] = 0;
		input->right_channel_buffer[i] = 0;
	}

}

void m3t_helper_wet_dry_mix( hope_dsp_buffer_struct * input_dry, float dry_gain, hope_dsp_buffer_struct * input_wet, float wet_gain, hope_dsp_buffer_struct * output )
{
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = dry_gain * input_dry->left_channel_buffer[i] + wet_gain * input_wet->left_channel_buffer[i];		
		output->right_channel_buffer[i] = dry_gain * input_dry->right_channel_buffer[i] + wet_gain * input_wet->right_channel_buffer[i];
	}

}

void m3t_helper_record_block( ct5_buffer_t * buf, hope_dsp_buffer_struct * input)
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		//if the button is not released, just keep recording
		*((buf->left_channel_physical_memory_start_address + buf->integer_write_head_address)) = 
			input->left_channel_buffer[i];

		*((buf->right_channel_physical_memory_start_address + buf->integer_write_head_address)) =
			input->right_channel_buffer[i];

		buf->integer_write_head_address++;
		if( buf->integer_write_head_address >= buf->buffer_size )
		{
			buf->integer_write_head_address = 0;
			buf->recording_wrapped = 1;
			buf->playback_end = buf->buffer_size - 1;
			buf->playback_start = 0;
		}
	}

}

//records from start_index to end_index inclusive.
void m3t_helper_record_index_to_index( ct5_buffer_t * buf, hope_dsp_buffer_struct * input, uint32_t start_index, uint32_t end_index )
{
	
	//the index needs to be less than HOPE_DSP_BUFFER_SIZE
	if( end_index >= HOPE_DSP_BUFFER_SIZE )
	{
		end_index = HOPE_DSP_BUFFER_SIZE - 1;
	}
	if( start_index >= end_index )
	{
		return;
	}
	
	for( uint32_t i = start_index; i <= end_index; i++)
	{
		//if the button is not released, just keep recording
		*((buf->left_channel_physical_memory_start_address + buf->integer_write_head_address)) = 
			input->left_channel_buffer[i];

		*((buf->right_channel_physical_memory_start_address + buf->integer_write_head_address)) =
			input->right_channel_buffer[i];

		buf->integer_write_head_address++;
		if( buf->integer_write_head_address >= buf->buffer_size )
		{
			buf->integer_write_head_address = 0;
			buf->recording_wrapped = 1;
			buf->playback_end = buf->buffer_size - 1;
			buf->playback_start = 0;
		}
	}

}


void m3t_helper_overdub_block( ct5_buffer_t * buf, hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	
	//we need to collapse this into one for loop.

	//write head address vector
	uint32_t write_head_address_vector[ HOPE_DSP_BUFFER_SIZE ];
	
	//write head volume vector
	float write_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];

	//read address vector
	buf->desired_dir = buf->m3t_variables->desired_dir;
	float read_head_address_vector[ HOPE_DSP_BUFFER_SIZE ];

	//playback volume
	buf->desired_playback_volume = buf->m3t_variables->playback_volume;
	float playback_volume_vector[ HOPE_DSP_BUFFER_SIZE ];

	//read volume vector
	float collision_zone_size = 100.0;
	float dead_zone_distance = 5.0;
	float read_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];

	//for linear interpolation of read data
	float alpha, beta;
	uint32_t low_address, high_address;

	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		// write_head_address_vector[i] =  ( ( i + buf->integer_write_head_address) % buf->playback_end );
		write_head_address_vector[i] = buf->integer_write_head_address;
		buf->integer_write_head_address++;
		if( buf->integer_write_head_address > buf->playback_end )
		{
			buf->integer_write_head_address = 0;
		}

		if( fabs(buf->desired_dir - buf->current_dir) > buf->dir_increment )
		{
			if(buf->desired_dir - buf->current_dir > 0)
			{
				buf->current_dir = buf->current_dir + buf->dir_increment;
			}
			else
			{
				buf->current_dir = buf->current_dir - buf->dir_increment;
			}
		}
		else
		{
			buf->current_dir = buf->desired_dir;
		}
		
		buf->float_read_head_address += buf->current_dir;

		if( buf->float_read_head_address > buf->playback_end )
		{
			buf->float_read_head_address = buf->float_read_head_address - buf->playback_end;
		}

		if( buf->float_read_head_address < 0 )
		{
			buf->float_read_head_address = buf->float_read_head_address + buf->playback_end;
		}

		read_head_address_vector[i] = buf->float_read_head_address;


		// for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
		// {
			if( fabs(buf->desired_playback_volume - buf->current_playback_volume) > buf->playback_volume_increment )
			{
				if(buf->desired_playback_volume - buf->current_playback_volume > 0)
				{
					buf->current_playback_volume = buf->current_playback_volume + buf->playback_volume_increment;
				}
				else
				{
					buf->current_playback_volume = buf->current_playback_volume - buf->playback_volume_increment;
				}
			}
			else
			{
				buf->current_playback_volume = buf->desired_playback_volume;
			}

			playback_volume_vector[i] = buf->current_playback_volume;
		// }


		float address_difference = fabs(read_head_address_vector[i] - (float)write_head_address_vector[i]);
		if( address_difference < dead_zone_distance )
		{
			read_head_volume_vector[i] = 0.0;
		}
		else if(  address_difference  < collision_zone_size )
		{
			read_head_volume_vector[i] = (address_difference - dead_zone_distance) / (collision_zone_size - dead_zone_distance);
		}
		else if ( address_difference > buf->playback_end - dead_zone_distance )
		{
			read_head_volume_vector[i] = 0.0;
		}
		else if ( address_difference > buf->playback_end- collision_zone_size )
		{			
			read_head_volume_vector[i] = (buf->playback_end - address_difference - dead_zone_distance )/(collision_zone_size - dead_zone_distance);
		}
		else
		{
			read_head_volume_vector[i] = 1.0;
		}
		// read_head_volume_vector[i] = 1.0;



		write_head_volume_vector[i] = 1.0 ;

		low_address = (uint32_t)read_head_address_vector[i];
		high_address = (uint32_t)read_head_address_vector[i] + 1;
		if( high_address > buf->playback_end )
		{
			high_address = high_address - buf->playback_end;
		}

		alpha = read_head_address_vector[i] - low_address;
		beta = 1.0 - alpha;
		output->left_channel_buffer[i] = 
				beta * (*(low_address + buf->left_channel_physical_memory_start_address)) 
			+ 	alpha * (*((high_address + buf->left_channel_physical_memory_start_address))) ;

		output->left_channel_buffer[i] *= read_head_volume_vector[i] * playback_volume_vector[i];

		output->right_channel_buffer[i] = 
				beta * (*(low_address + buf->right_channel_physical_memory_start_address))
			+ 	alpha * (*((high_address + buf->right_channel_physical_memory_start_address))) ;

		output->right_channel_buffer[i] *= read_head_volume_vector[i] * playback_volume_vector[i];

		//now write
		*((write_head_address_vector[i] + buf->left_channel_physical_memory_start_address))  
			+= input->left_channel_buffer[i] * write_head_volume_vector[i];

		*((write_head_address_vector[i] + buf->right_channel_physical_memory_start_address))
			+= input->right_channel_buffer[i] * write_head_volume_vector[i];




	}


}

void m3t_helper_overdub_index_to_index( ct5_buffer_t * buf, hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output,  uint32_t start_index, uint32_t end_index )
{


	//the index needs to be less than HOPE_DSP_BUFFER_SIZE
	if( end_index >= HOPE_DSP_BUFFER_SIZE )
	{
		end_index = HOPE_DSP_BUFFER_SIZE - 1;
	}
	if( start_index >= end_index )
	{
		return;
	}
	
	//write head address vector
	uint32_t write_head_address_vector[ HOPE_DSP_BUFFER_SIZE ];

	for(uint32_t i = start_index; i <= end_index; i++)
	{
		write_head_address_vector[i] =  ( ( i + buf->integer_write_head_address) % buf->playback_end );
	}
	buf->integer_write_head_address = ( HOPE_DSP_BUFFER_SIZE + buf->integer_write_head_address ) % buf->playback_end;

	//read address vector
	float read_head_address_vector[ HOPE_DSP_BUFFER_SIZE ];
	for(uint32_t i = start_index; i <= end_index ; i++)
	{
		if( fabs(buf->desired_dir - buf->current_dir) > buf->dir_increment )
		{
			if(buf->desired_dir - buf->current_dir > 0)
			{
				buf->current_dir = buf->current_dir + buf->dir_increment;
			}
			else
			{
				buf->current_dir = buf->current_dir - buf->dir_increment;
			}
		}
		else
		{
			buf->current_dir = buf->desired_dir;
		}
		
		buf->float_read_head_address += buf->current_dir;

		if( buf->float_read_head_address > buf->playback_end )
		{
			buf->float_read_head_address = buf->float_read_head_address - buf->playback_end;
		}

		if( buf->float_read_head_address < 0 )
		{
			buf->float_read_head_address = buf->float_read_head_address + buf->playback_end;
		}

		read_head_address_vector[i] = buf->float_read_head_address;
	}

	//read volume vector
	//next calculate the read head volume based on collision detection
	
	float collision_zone_size = 100.0;
	float dead_zone_distance = 5.0;
	float read_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	// volatile uint32_t temp = 0;
	// uint32_t prev = 0;
	for(uint32_t i = start_index; i <= end_index ; i++)
	{
		float address_difference = fabs(read_head_address_vector[i] - (float)write_head_address_vector[i]);
		if( address_difference < dead_zone_distance )
		{
			read_head_volume_vector[i] = 0.0;
		}
		else if(  address_difference  < collision_zone_size )
		{
			read_head_volume_vector[i] = (address_difference - dead_zone_distance) / (collision_zone_size - dead_zone_distance);
		}
		else if ( address_difference > buf->buffer_size - dead_zone_distance )
		{
			read_head_volume_vector[i] = 0.0;
		}
		else if ( address_difference > buf->buffer_size - collision_zone_size )
		{			
			read_head_volume_vector[i] = (buf->buffer_size - address_difference - dead_zone_distance )/(collision_zone_size - dead_zone_distance);
		}
		else
		{
			read_head_volume_vector[i] = 1.0;
		}

	}	

	//write volume vector
	//calculate the write head volume vector
	float write_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	for(uint32_t i = start_index; i <= end_index ; i++)
	{
		write_head_volume_vector[i] = 1.0 ;
	}


	//then read and write per sample.
	// float read_head_data_left[ HOPE_DSP_BUFFER_SIZE ];
	// float read_head_data_right[ HOPE_DSP_BUFFER_SIZE ];

	float alpha, beta;
	uint32_t low_address, high_address;
	for(uint32_t i = start_index; i <= end_index ; i++)
	{
		low_address = (uint32_t)read_head_address_vector[i];
		high_address = (uint32_t)read_head_address_vector[i] + 1;
		if( high_address >= buf->playback_end )
		{
			high_address = high_address - buf->playback_end;
		}

		alpha = read_head_address_vector[i] - low_address;
		beta = 1.0 - alpha;
		output->left_channel_buffer[i] = 
				beta * (*(low_address + buf->left_channel_physical_memory_start_address)) 
			+ 	alpha * (*((high_address + buf->left_channel_physical_memory_start_address))) ;

		output->left_channel_buffer[i] *= read_head_volume_vector[i];

		output->right_channel_buffer[i] = 
				beta * (*(low_address + buf->right_channel_physical_memory_start_address))
			+ 	alpha * (*((high_address + buf->right_channel_physical_memory_start_address))) ;

		output->right_channel_buffer[i] *= read_head_volume_vector[i];

		//now write
		*((write_head_address_vector[i] + buf->left_channel_physical_memory_start_address))  
			+= input->left_channel_buffer[i] * write_head_volume_vector[i];

		*((write_head_address_vector[i] + buf->right_channel_physical_memory_start_address))
			+= input->right_channel_buffer[i] * write_head_volume_vector[i];
	}
	

}

void m3t_helper_playback_block( ct5_buffer_t * buf,  hope_dsp_buffer_struct * output )
{
	// buf->desired_dir = 1.1;
	buf->desired_dir = buf->m3t_variables->desired_dir;
	float read_head_address_vector[ HOPE_DSP_BUFFER_SIZE ];

	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		if( fabs(buf->desired_dir - buf->current_dir) > buf->dir_increment )
		{
			if(buf->desired_dir - buf->current_dir > 0)
			{
				buf->current_dir = buf->current_dir + buf->dir_increment;
			}
			else
			{
				buf->current_dir = buf->current_dir - buf->dir_increment;
			}
		}
		else
		{
			buf->current_dir = buf->desired_dir;
		}
		
		buf->float_read_head_address += buf->current_dir;

		//this math assumes playback_start is 0, which may not be true in general.
		if( buf->float_read_head_address > buf->playback_end )
		{
			buf->float_read_head_address = buf->float_read_head_address - buf->playback_end;
		}

		if( buf->float_read_head_address < 0 )
		{
			buf->float_read_head_address = buf->float_read_head_address + buf->playback_end;
		}

		read_head_address_vector[i] = buf->float_read_head_address;
	}

	buf->desired_playback_volume = buf->m3t_variables->playback_volume;
	float playback_volume_vector[ HOPE_DSP_BUFFER_SIZE ];

	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		if( fabs(buf->desired_playback_volume - buf->current_playback_volume) > buf->playback_volume_increment )
		{
			if(buf->desired_playback_volume - buf->current_playback_volume > 0)
			{
				buf->current_playback_volume = buf->current_playback_volume + buf->playback_volume_increment;
			}
			else
			{
				buf->current_playback_volume = buf->current_playback_volume - buf->playback_volume_increment;
			}
		}
		else
		{
			buf->current_playback_volume = buf->desired_playback_volume;
		}

		playback_volume_vector[i] = buf->current_playback_volume;
	}


	// //now determine the volume vector as well
	// float read_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	// for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	// {
	// 	read_head_volume_vector[i] = 1.0;
	// }

	//now we can make the output
	// float read_head_data_left[ HOPE_DSP_BUFFER_SIZE ];
	// float read_head_data_right[ HOPE_DSP_BUFFER_SIZE ];

	float alpha, beta;
	uint32_t low_address, high_address;
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		low_address = (uint32_t)read_head_address_vector[i];
		high_address = (uint32_t)read_head_address_vector[i] + 1;
		if( high_address > buf->playback_end )
		{
			high_address = high_address - buf->playback_end;
		}

		alpha = read_head_address_vector[i] - low_address;
		beta = 1.0 - alpha;
		output->left_channel_buffer[i] = 
				beta * (*(low_address + buf->left_channel_physical_memory_start_address)) 
			+ 	alpha * (*((high_address + buf->left_channel_physical_memory_start_address))) ;
	
		output->left_channel_buffer[i] *= playback_volume_vector[i];

		output->right_channel_buffer[i] = 
				beta * (*(low_address + buf->right_channel_physical_memory_start_address))
			+ 	alpha * (*((high_address + buf->right_channel_physical_memory_start_address))) ;

		output->right_channel_buffer[i] *= playback_volume_vector[i];

	}

}

uint32_t m3t_helper_destructive_scan_for_zero_crossing( hope_dsp_buffer_struct * input, uint32_t * left_zc_index, uint32_t * right_zc_index )
{
	uint32_t left_crossing_found = 0;
	uint32_t right_crossing_found = 0;

	for( uint32_t i = 1; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
	
		if( !left_crossing_found )
		{
			if( input->left_channel_buffer[i] > 0 && input->left_channel_buffer[i - 1] < 0 )
			{
				left_crossing_found = i;
			}
		}

		if( !right_crossing_found )
		{
			if( input->right_channel_buffer[i] > 0 && input->right_channel_buffer[i - 1] < 0 )
			{
				right_crossing_found = i;
			}
		}
	}

	if( left_crossing_found && right_crossing_found )
	{
		for( uint32_t i = 0; i < left_crossing_found; i++)
		{
			input->left_channel_buffer[i] = 0;
		}

		for( uint32_t i = 0; i < right_crossing_found; i++)
		{
			input->right_channel_buffer[i] = 0;
		}
		*left_zc_index = left_crossing_found;
		*right_zc_index = right_crossing_found;
		return 1;
	}

	return 0;
}

uint32_t m3t_helper_destructive_reverse_scan_for_zero_crossing( hope_dsp_buffer_struct * input, uint32_t * left_zc_index, uint32_t * right_zc_index )
{
	uint32_t left_crossing_found = 0;
	uint32_t right_crossing_found = 0;

	for( uint32_t i = HOPE_DSP_BUFFER_SIZE -1; i > 0; i--)
	{
	
		if( !left_crossing_found )
		{
			if( input->left_channel_buffer[i] > 0 && input->left_channel_buffer[i - 1] < 0 )
			{
				left_crossing_found = i;
			}
		}

		if( !right_crossing_found )
		{
			if( input->right_channel_buffer[i] > 0 && input->right_channel_buffer[i - 1] < 0 )
			{
				right_crossing_found = i;
			}
		}
	}

	if( left_crossing_found && right_crossing_found )
	{
		for( uint32_t i = left_crossing_found; i < HOPE_DSP_BUFFER_SIZE; i++)
		{
			input->left_channel_buffer[i] = 0;
		}

		for( uint32_t i = right_crossing_found; i < HOPE_DSP_BUFFER_SIZE; i++)
		{
			input->right_channel_buffer[i] = 0;
		}
		*left_zc_index = left_crossing_found;
		*right_zc_index = right_crossing_found;
		return 1;
	}

	return 0;
}

void m3t_helper_copy_buffer( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output)
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = input->left_channel_buffer[i];
		output->right_channel_buffer[i] = input->right_channel_buffer[i];
	}
}

void m3t_helper_block_length_fade_in( hope_dsp_buffer_struct *output )
{
	float increment = 1.0 / HOPE_DSP_BUFFER_SIZE;
	float gain = 0.0;
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] *= gain;
		output->right_channel_buffer[i] *= gain;
		gain += increment;
	}
}

void m3t_helper_block_length_fade_out( hope_dsp_buffer_struct *output )
{
	float decrement = 1.0 / HOPE_DSP_BUFFER_SIZE;
	float gain = 1.0;
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] *= gain;
		output->right_channel_buffer[i] *= gain;
		gain -= decrement;
	}
}

void m3t_helper_append_block( ct5_buffer_t * buf , hope_dsp_buffer_struct *input  )
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		*((buf->left_channel_physical_memory_start_address + buf->integer_write_head_address)) += 
			input->left_channel_buffer[i];

		*((buf->right_channel_physical_memory_start_address + buf->integer_write_head_address)) +=
			input->right_channel_buffer[i];

		buf->integer_write_head_address++;
		if( buf->integer_write_head_address >= buf->buffer_size )
		{
			buf->integer_write_head_address = 0;
			// buf->recording_wrapped = 1;
			// buf->playback_end = buf->buffer_size - 1;
			// buf->playback_start = 0;
		}
	}
}

void m3t_helper_add_two_dsp_buffers( hope_dsp_buffer_struct * input_1, hope_dsp_buffer_struct * input_2, hope_dsp_buffer_struct * output )
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = input_1->left_channel_buffer[i] + input_2->left_channel_buffer[i];
		output->right_channel_buffer[i] = input_1->right_channel_buffer[i] + input_2->right_channel_buffer[i];
	}
}
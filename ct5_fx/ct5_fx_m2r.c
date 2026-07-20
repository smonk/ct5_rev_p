#include "ct5_buffer.h"
#include "ct5_fx_m2r.h"
#include "../hope_hal/hope_dsp_interface.h"
#include "../hope_hal/hope_pwm_rgb_led.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "../hope_hal/hope_btn_and_sw.h"
#include "../hope_hal/hope_version.h"

#include "ct5_fx_state_machine.h"
#include "ct5_fx_ctl_input.h"

// ███╗   ███╗██████╗ ██████╗ 
// ████╗ ████║╚════██╗██╔══██╗
// ██╔████╔██║ █████╔╝██████╔╝
// ██║╚██╔╝██║██╔═══╝ ██╔══██╗
// ██║ ╚═╝ ██║███████╗██║  ██║
// ╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝

//todo:
// eliminate start/stop artifacts
// mask m2r_n_playing with N switch value (or midi etc)
// connect pots and switches.
// should the contents of the current buffer be played during overdub (recording_wrapped)
// should the state functions take a pointer to the state that called it? this way you could access data

#define M2_RAND_N_BUFFERS 4

//big font from
//https://www.asciiart.eu/text-to-ascii-art
//used 'ansi shadow' font

static ct5_buffer_t * bufs[M2_RAND_N_BUFFERS];
 
static ct5_buffer_t ct5_buffer_a;
static ct5_buffer_t ct5_buffer_b;
static ct5_buffer_t ct5_buffer_c;
static ct5_buffer_t ct5_buffer_d;

static m2r_variables_t m2r_variables_a;
static m2r_variables_t m2r_variables_b;
static m2r_variables_t m2r_variables_c;
static m2r_variables_t m2r_variables_d;

static m2r_variables_t * m2r_global_variables;

// struct state_function_ponter;

// typedef struct state_function_pointer (*state_function)( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

// typedef struct state_function_pointer
// {
// 	state_function next_state;
// }state_function_pointer_t;

//function declarations
void ct5_fx_m2r_init( void );

state_function_pointer_t m2r_state_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m2r_state_reset_to_record( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m2r_state_rec_overdub_n_max( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m2r_state_record_to_playback( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m2r_state_play_n_max( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m2r_state_playback_to_record( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m2r_state_record_to_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

uint32_t m2r_event_record_button_pressed( void );
uint32_t m2r_event_record_button_was_tapped( void );
uint32_t m2r_event_record_button_was_held( void );

void m2r_helper_playback_block( ct5_buffer_t * buf,  hope_dsp_buffer_struct * output );
void m2r_helper_record_block( ct5_buffer_t * buf, hope_dsp_buffer_struct * input);
void m2r_helper_wet_dry_mix( hope_dsp_buffer_struct * input_dry, float dry_gain, hope_dsp_buffer_struct * input_wet, float wet_gain, hope_dsp_buffer_struct * output );
void m2r_helper_sum_n_buffers( hope_dsp_buffer_struct * input, uint32_t n, hope_dsp_buffer_struct * output, uint32_t pre_zero_output );
void m2r_helper_sum_2_buffers( hope_dsp_buffer_struct * input_1, hope_dsp_buffer_struct * input_2,  hope_dsp_buffer_struct * output );
void m2r_helper_zero_buffer( hope_dsp_buffer_struct * input );
void m2r_helper_increment_n_playing( ct5_buffer_t ** bufs, uint32_t * n );
uint32_t m2r_helper_destructive_scan_for_zero_crossing( hope_dsp_buffer_struct * input );
void m2r_helper_copy_buffer( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output);
void m2r_helper_block_length_fade_in( hope_dsp_buffer_struct *output );
void m2r_helper_block_length_fade_out( hope_dsp_buffer_struct * output );
float m2r_helper_distance_to_start( ct5_buffer_t * buf );
float m2r_helper_distance_to_end( ct5_buffer_t * buf );
float m2r_helper_distance_to_gain_coefficient( float distance ); 
void m2r_helper_new_playback_markers( ct5_buffer_t * buf );

uint32_t m2r_n_playing = 0;

state_function_pointer_t main_sfp;

float * debug_address_pointer;
uint32_t debug_index = 0;

// ██╗███╗   ██╗██╗████████╗
// ██║████╗  ██║██║╚══██╔══╝
// ██║██╔██╗ ██║██║   ██║   
// ██║██║╚██╗██║██║   ██║   
// ██║██║ ╚████║██║   ██║   
// ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝   


void ct5_fx_m2r_init()
{

	bufs[0] = &ct5_buffer_a;
	bufs[1] = &ct5_buffer_b;
	bufs[2] = &ct5_buffer_c;
	bufs[3] = &ct5_buffer_d;

	bufs[0]->m2r_variables = &m2r_variables_a;
	bufs[1]->m2r_variables = &m2r_variables_b;
	bufs[2]->m2r_variables = &m2r_variables_c;
	bufs[3]->m2r_variables = &m2r_variables_d;

	uint32_t channel_buffer_size = CT5_BUFFER_MEM_SIZE_WORDS / 8;

	for( int i = 0; i < M2_RAND_N_BUFFERS; i++)
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

		bufs[i]->desired_dir = 2.0 - i * .25;
		bufs[i]->current_dir = 1.0;
		bufs[i]->dir_increment = 0.0005;

		bufs[i]->playback_start = 0;
		bufs[i]->playback_end = 0;
		bufs[i]->playback_length = 0;

		bufs[i]->recording_start = 0;
		bufs[i]->recording_end = 0;
		bufs[i]->recording_length = 0;
		bufs[i]->recording_wrapped = 0;

	}

	main_sfp.next_state = m2r_state_reset;

	// my_m2r_variables = &m2r_variables_a;
}


// ███╗   ███╗ █████╗ ██╗███╗   ██╗
// ████╗ ████║██╔══██╗██║████╗  ██║
// ██╔████╔██║███████║██║██╔██╗ ██║
// ██║╚██╔╝██║██╔══██║██║██║╚██╗██║
// ██║ ╚═╝ ██║██║  ██║██║██║ ╚████║
// ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝



void ct5_fx_m2r( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	static uint32_t is_init = 0;
	if(!is_init)
	{
		ct5_fx_m2r_init();
		is_init = 1;
		return;
	}
	//zero the output buffer,start
	m2r_helper_zero_buffer( output );

	//
	ct5_fx_get_m2r_variables( m2r_global_variables );

	//update the ctl_inputs.
	ct5_fx_get_m2r_variables( bufs[1]->m2r_variables );

	//run algorithm
	main_sfp = main_sfp.next_state( input , output );

	float wet_gain, dry_gain;
	wet_gain = m2r_global_variables->wet_gain;
	dry_gain = m2r_global_variables->dry_gain;

	//do the clean mix here?
	m2r_helper_wet_dry_mix( input, dry_gain, output, wet_gain, output );

}


// ███████╗████████╗ █████╗ ████████╗███████╗███████╗
// ██╔════╝╚══██╔══╝██╔══██╗╚══██╔══╝██╔════╝██╔════╝
// ███████╗   ██║   ███████║   ██║   █████╗  ███████╗
// ╚════██║   ██║   ██╔══██║   ██║   ██╔══╝  ╚════██║
// ███████║   ██║   ██║  ██║   ██║   ███████╗███████║
// ╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚══════╝

extern void (*hope_ct5_led_on_setting)( hope_pwm_rgb_led_struct * );
extern void (*hope_ct5_led_off_setting)( hope_pwm_rgb_led_struct * );

state_function_pointer_t m2r_state_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//in this state we want to reset all the record and play heads
	//we leave the state when a button pressed event occurs on the soft foot switch.


	//do the thing
	//blink an led?
	hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_blinking_blue;

	for( uint32_t i = 0; i < M2_RAND_N_BUFFERS; i++)
	{
		bufs[i]->float_read_head_address = 0;
		bufs[i]->integer_write_head_address = 0;


		bufs[i]->playback_start = 0;
		bufs[i]->playback_end = 0;
		bufs[i]->playback_length = 0;
		bufs[i]->recording_wrapped = 0;

	}

	m2r_n_playing = 0;


	//now check the events
	//if the button was pressed, then we need to return the epointer to m2r_state_rec_overdub_1_max
	//else we stay in this state

	state_function_pointer_t sfp;
	if( m2r_event_record_button_pressed() )
	{
		//go to mtr_state_rec_overdub_1_max
		//first we need a struct to put the pointer in
		sfp.next_state = m2r_state_reset_to_record;
		return sfp;
	}
	else
	{

		sfp.next_state = m2r_state_reset;
		return sfp;
	}

}

state_function_pointer_t m2r_state_reset_to_record( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//do the thing.

	//our first try we will just hold this state until zero crossing is detected.

	//do the thing

	hope_dsp_buffer_struct temp;
	float temp_left_buffer[ HOPE_DSP_BUFFER_SIZE ];
	float temp_right_buffer[ HOPE_DSP_BUFFER_SIZE ];
	temp.left_channel_buffer = temp_left_buffer;
	temp.right_channel_buffer = temp_right_buffer;

	//make a copy of the input
	m2r_helper_copy_buffer( input, &temp );

	//fade in temp
	m2r_helper_block_length_fade_in( &temp );

	//record the modified temp into bufs[0]
	m2r_helper_record_block( bufs[0], &temp );

	//go to m2t_state_record
	state_function_pointer_t sfp;
	sfp.next_state = m2r_state_rec_overdub_n_max;
	return sfp;	
}



state_function_pointer_t m2r_state_rec_overdub_n_max( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//first do the thing
	//record into bufs[0]
	m2r_helper_record_block( bufs[0], input );

	//we need a temp buffer to hold the output of playback
	hope_dsp_buffer_struct temp_read_head_data;
	float trhd_left[HOPE_DSP_BUFFER_SIZE];
	float trhd_right[HOPE_DSP_BUFFER_SIZE];
	temp_read_head_data.left_channel_buffer = trhd_left;
	temp_read_head_data.right_channel_buffer = trhd_right;


	//now stack the read data
	for( uint32_t i = 1; i <= m2r_n_playing; i++)
	{
		m2r_helper_playback_block( bufs[i], &temp_read_head_data );
		m2r_helper_sum_2_buffers( &temp_read_head_data, output, output );
	}

	//led is red
	hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_red;

	//in this state you can start recording to the buffer once an appropriate sample is found.

	//when the button gets released it is either a tap or a hold.
	//if its a tap set the next state to reset
	state_function_pointer_t sfp;
	if( m2r_event_record_button_was_tapped() )
	{
		//go to mtr_state_reset
		//first we need a struct to put the pointer in

		sfp.next_state = m2r_state_record_to_reset;
		return sfp;
	}
	else if( m2r_event_record_button_was_held() )
	{
		//if its a hold set the next state to play_1_max
		//go to mtr_state_play_1_max
		//first we need a struct to put the pointer in
		sfp.next_state = m2r_state_record_to_playback;
		return sfp;
	}
	else
	{
		sfp.next_state = m2r_state_rec_overdub_n_max;
		return sfp;
	}
}

uint32_t m2r_n_to_fade_on_increment = 3;

state_function_pointer_t m2r_state_record_to_playback( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//first do the thing
	//we need a temp buffer to hold the output of playback
	hope_dsp_buffer_struct temp;
	float trhd_left[HOPE_DSP_BUFFER_SIZE];
	float trhd_right[HOPE_DSP_BUFFER_SIZE];
	temp.left_channel_buffer = trhd_left;
	temp.right_channel_buffer = trhd_right;

	//copt ths input into temp
	m2r_helper_copy_buffer( input, &temp );

	m2r_helper_block_length_fade_out( &temp );

	//you need the vars
	ct5_fx_get_m2r_variables( bufs[0]->m2r_variables );

	//record into bufs[0]
	m2r_helper_record_block( bufs[0], &temp );


	//now calculate the output
	m2r_helper_zero_buffer( &temp );

	//its a bit more complicated because we need to fade out
	//the read buffer that is about to be removed
	//and fade in the read buffer that is about to be added? 
	//the fade in shoudl be handled already actually 

	bufs[0]->recording_length = bufs[0]->recording_end - bufs[0]->recording_start;
	
	m2r_helper_new_playback_markers( bufs[0] );
	if( m2r_global_variables->desired_dir > 0)
	{
		bufs[0]->float_read_head_address = bufs[0]->recording_start;
	}
	else
	{
		bufs[0]->float_read_head_address = bufs[0]->recording_end;
	}
	bufs[0]->desired_dir = m2r_global_variables->desired_dir;

	//the fade out should be done to bufs[m2r_n_playing] buffer
	
	//we need a randomized start based on the info in the buffer, so make a helper that takes a buffer and returns an integer start address

	//now stack the read data
	for( uint32_t i = 1; i < m2r_n_playing; i++)
	{
		m2r_helper_playback_block( bufs[i], &temp );
		m2r_helper_sum_2_buffers( &temp, output, output );
	}
	if( m2r_n_playing == m2r_n_to_fade_on_increment )	 
	{
		m2r_helper_playback_block( bufs[m2r_n_to_fade_on_increment], &temp );
		m2r_helper_block_length_fade_out( &temp );
		m2r_helper_sum_2_buffers( &temp, output, output );
	}
	else
	{
		m2r_helper_playback_block( bufs[m2r_n_playing], &temp );
		m2r_helper_sum_2_buffers( &temp, output, output );
	}

	//led is red
	hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_red;

	//in this state you can start recording to the buffer once an appropriate sample is found.

	//when the button gets released it is either a tap or a hold.
	//if its a tap set the next state to reset

	m2r_helper_increment_n_playing( bufs, &m2r_n_playing );

	state_function_pointer_t sfp;

	sfp.next_state = m2r_state_play_n_max;
	return sfp;


}
state_function_pointer_t m2r_state_play_n_max( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//do the thing

	//led is blue?
	hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_blue;

	//we need to playback m2r_n_playing buffers
	//we need a temp buffer to hold teh output of playback
	hope_dsp_buffer_struct temp_read_head_data;
	float trhd_left[HOPE_DSP_BUFFER_SIZE];
	float trhd_right[HOPE_DSP_BUFFER_SIZE];
	temp_read_head_data.left_channel_buffer = trhd_left;
	temp_read_head_data.right_channel_buffer = trhd_right;
	
	//zero the output buffer
	m2r_helper_zero_buffer( output );
	
	//now stack the read data
	for( uint32_t i = 1; i <= m2r_n_playing; i++)
	{
		m2r_helper_playback_block( bufs[i], &temp_read_head_data );
		m2r_helper_sum_2_buffers( &temp_read_head_data, output, output );
	}

	//in this state we are just playing back, no recording.
	state_function_pointer_t sfp;
	if( m2r_event_record_button_pressed() )
	{

		sfp.next_state = m2r_state_playback_to_record;
		return sfp;
	}
	else
	{
		sfp.next_state = m2r_state_play_n_max;
		return sfp;
	}

}

state_function_pointer_t m2r_state_playback_to_record( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//do the thing
	//we need a temp buffer to hold the output of playback
	hope_dsp_buffer_struct temp;
	float trhd_left[HOPE_DSP_BUFFER_SIZE];
	float trhd_right[HOPE_DSP_BUFFER_SIZE];
	temp.left_channel_buffer = trhd_left;
	temp.right_channel_buffer = trhd_right;

	//copt ths input into temp
	m2r_helper_copy_buffer( input, &temp );

	m2r_helper_block_length_fade_in( &temp );

	//record into bufs[0]
	m2r_helper_record_block( bufs[0], &temp );

	m2r_helper_zero_buffer( &temp );

	//now stack the read data
	for( uint32_t i = 1; i <= m2r_n_playing; i++)
	{
		m2r_helper_playback_block( bufs[i], &temp );
		m2r_helper_sum_2_buffers( &temp, output, output );
	}

	//led is red
	hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_red;

	//in this state you can start recording to the buffer once an appropriate sample is found.

	//when the button gets released it is either a tap or a hold.
	//if its a tap set the next state to reset
	state_function_pointer_t sfp;

	sfp.next_state = m2r_state_rec_overdub_n_max;
	return sfp;


}

state_function_pointer_t m2r_state_record_to_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//first do the thing
	//record into bufs[0]
	m2r_helper_record_block( bufs[0], input );

	//we need a temp buffer to hold the output of playback
	hope_dsp_buffer_struct temp_read_head_data;
	float trhd_left[HOPE_DSP_BUFFER_SIZE];
	float trhd_right[HOPE_DSP_BUFFER_SIZE];
	temp_read_head_data.left_channel_buffer = trhd_left;
	temp_read_head_data.right_channel_buffer = trhd_right;

	//now stack the read data
	for( uint32_t i = 1; i <= m2r_n_playing; i++)
	{
		m2r_helper_playback_block( bufs[i], &temp_read_head_data );
		m2r_helper_sum_2_buffers( &temp_read_head_data, output, output );
	}

	//led is red
	hope_ct5_led_on_setting = hope_ct5_pwm_rgb_set_solid_red;

	//now fade out the output.
	m2r_helper_block_length_fade_out( output );


	//in this state you can start recording to the buffer once an appropriate sample is found.

	//when the button gets released it is either a tap or a hold.
	//if its a tap set the next state to reset
	state_function_pointer_t sfp;
	sfp.next_state = m2r_state_reset;
	return sfp;

}

// ███████╗██╗   ██╗███████╗███╗   ██╗████████╗███████╗
// ██╔════╝██║   ██║██╔════╝████╗  ██║╚══██╔══╝██╔════╝
// █████╗  ██║   ██║█████╗  ██╔██╗ ██║   ██║   ███████╗
// ██╔══╝  ╚██╗ ██╔╝██╔══╝  ██║╚██╗██║   ██║   ╚════██║
// ███████╗ ╚████╔╝ ███████╗██║ ╚████║   ██║   ███████║
// ╚══════╝  ╚═══╝  ╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝


extern hope_btn_and_sw_struct my_btn_and_sw[ HOPE_NUM_BTN_AND_SW ];

uint32_t m2r_event_record_button_pressed( )
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

uint32_t m2r_event_record_button_was_tapped( )
{
	//check a specific button and return 1 if it was tapped. 0 if not.
	if( my_btn_and_sw[6].tap_event_flag == 1 )
	{
		my_btn_and_sw[6].tap_event_flag = 0;
		return 1;
	}
	return 0;
}

uint32_t m2r_event_record_button_was_held( )
{
	//check a specific button and return 1 if it was held. 0 if not.	
	if( my_btn_and_sw[6].hold_event_flag == 1 )
	{
		my_btn_and_sw[6].hold_event_flag = 0;
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


//this function will put 1 block of data into the output buffer given based on what is in the ct5_buffer_t
void m2r_helper_playback_block( ct5_buffer_t * buf,  hope_dsp_buffer_struct * output )
{
	// buf->desired_dir = 1.1;
	buf->desired_dir = buf->m2r_variables->desired_dir;
	float read_head_address_vector[ HOPE_DSP_BUFFER_SIZE ];
	float read_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	
	debug_address_pointer = read_head_address_vector;
	float alpha, beta;
	uint32_t low_address, high_address;
	
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		debug_index = i;
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

		//if either of this condtitions are true, then we need to reroll the start and end address.
		//this section assumes playback_end > playback_start. Unfortunately this
		//is not always true. So we need to make a better  function that handles things properly.
		
		// in this situation where end is greater than sart, we dont have to worry about wrappying on the record space, because we cant get there.
		if( buf->playback_end > buf->playback_start)
		{
			if( buf->float_read_head_address > buf->playback_end )
			{
				m2r_helper_new_playback_markers ( buf );
				buf->float_read_head_address = buf->playback_start;
			}
			else if( buf->float_read_head_address < buf->playback_start )
			{
				m2r_helper_new_playback_markers( buf );
				buf->float_read_head_address = buf->playback_end;
			}
		}
		// however here we have to wrap around the record space becuase we can be there.
		else
		{
			if( buf->float_read_head_address > buf->recording_end )
			{
				buf->float_read_head_address -= buf->recording_length; //should be length? but length equale end 
			}
			else if( buf->float_read_head_address < buf->recording_start )
			{
				buf->float_read_head_address += buf->recording_length;
			}

			if( ( buf->float_read_head_address < buf->playback_start ) && ( buf->float_read_head_address > buf->playback_end ) )
			{
				//this means we are out of bounds.
				m2r_helper_new_playback_markers( buf );
				if(buf->current_dir > 0)
				{
					buf->float_read_head_address = buf->playback_start;
				}
				else
				{
					buf->float_read_head_address = buf->playback_end;
				}
			}
		}
		read_head_address_vector[i] = buf->float_read_head_address;
		float ds = m2r_helper_distance_to_start( buf );
		float ds_gain = m2r_helper_distance_to_gain_coefficient( ds );

		float de = m2r_helper_distance_to_end( buf );
		float de_gain = m2r_helper_distance_to_gain_coefficient( de );
		read_head_volume_vector[i] = ds_gain * de_gain; 

		low_address = (uint32_t)read_head_address_vector[i];
		high_address = (uint32_t)read_head_address_vector[i] + 1;
		if( high_address > buf->recording_end )
		{
			high_address = 0;
		}


		// if( high_address > buf->playback_end )
		// {
		// 	high_address = high_address - buf->playback_length;
		// }

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


	}

	// //now determine the volume vector as well
	// for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	// {
	// 	read_head_volume_vector[i] = 1.0;
	// }

	//now we can make the output
	// float read_head_data_left[ HOPE_DSP_BUFFER_SIZE ];
	// float read_head_data_right[ HOPE_DSP_BUFFER_SIZE ];


	// for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	// {
	// 	low_address = (uint32_t)read_head_address_vector[i];
	// 	high_address = (uint32_t)read_head_address_vector[i] + 1;
	// 	if( high_address > buf->playback_end )
	// 	{
	// 		high_address = high_address - buf->playback_length;
	// 	}

	// 	alpha = read_head_address_vector[i] - low_address;
	// 	beta = 1.0 - alpha;
	// 	output->left_channel_buffer[i] = 
	// 			beta * (*(low_address + buf->left_channel_physical_memory_start_address)) 
	// 		+ 	alpha * (*((high_address + buf->left_channel_physical_memory_start_address))) ;
	
	// 	output->left_channel_buffer[i] *= read_head_volume_vector[i];

	// 	output->right_channel_buffer[i] = 
	// 			beta * (*(low_address + buf->right_channel_physical_memory_start_address))
	// 		+ 	alpha * (*((high_address + buf->right_channel_physical_memory_start_address))) ;

	// 	output->right_channel_buffer[i] *= read_head_volume_vector[i];

	// }

}

void m2r_helper_new_playback_markers( ct5_buffer_t * buf )
{
	//we need a uniform random variable between 0 and 1
	float uniform_random_variable = (float)rand() / (float)RAND_MAX;
	//assumes recording_start = 0, which i t should be in this algorithm
	buf->playback_start = uniform_random_variable * buf->recording_end * buf->m2r_variables->playback_start_randomization;
	buf->playback_length = buf->recording_end * buf->m2r_variables->playback_slice_length;
	if( buf->playback_length < (10* HOPE_DSP_BUFFER_SIZE))
	{
		buf->playback_length = (10* HOPE_DSP_BUFFER_SIZE);
	}		
	buf->playback_end = buf->playback_start + buf->playback_length -1; 
	if( buf->playback_end > buf->recording_end )
	{
		buf->playback_end -= buf->recording_end;
		if(buf->playback_end < 0)
		{
			while(1);
		}
	}

}
float m2r_helper_distance_to_start( ct5_buffer_t * buf )
{
	//first case is simple, we know start is less than end.
	if( buf->playback_start < buf->playback_end )
	{
		return buf->float_read_head_address - buf->playback_start;
	}
	else
	{
		if( buf->float_read_head_address < buf->playback_end )
		{
			return buf->float_read_head_address + buf->recording_end - buf->playback_start;
		}
		else
		{
			return buf->float_read_head_address - buf->playback_start;
		}
	}
}

float m2r_helper_distance_to_end( ct5_buffer_t * buf )
{
	//first case is simple, we know start is less than end.
	if( buf->playback_start < buf->playback_end )
	{
		return buf->playback_end - buf->float_read_head_address;
	}
	else
	{
		if( buf->float_read_head_address <= buf->playback_end )
		{
			return buf->playback_end - buf->float_read_head_address;
		}
		else
		{
			return buf->playback_end + buf->recording_end - buf->float_read_head_address;
		}
	}
}

float m2r_helper_distance_to_gain_coefficient( float distance )
{
	if( distance >= HOPE_DSP_BUFFER_SIZE )
	{
		return 1.0;
	}
	else
	{
		return distance / HOPE_DSP_BUFFER_SIZE;
	}
}

//this will recordt into the ct5_buffer_t whatever is in the hope_dsp_buffer_struct input
void m2r_helper_record_block( ct5_buffer_t * buf, hope_dsp_buffer_struct * input)
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		//destruictive write or not? check recording_wrapped
		if( buf->recording_wrapped == 1 )
		{
		// if the button is not released, just keep recording
			*((buf->left_channel_physical_memory_start_address + buf->integer_write_head_address)) += 
				input->left_channel_buffer[i];

			*((buf->right_channel_physical_memory_start_address + buf->integer_write_head_address)) +=
				input->right_channel_buffer[i];
		}
		else
		//destruictive write or not? check recording_wrapped
		if( buf->recording_wrapped == 1 )
		{
		// if the button is not released, just keep recording
			*((buf->left_channel_physical_memory_start_address + buf->integer_write_head_address)) += 
				input->left_channel_buffer[i];

			*((buf->right_channel_physical_memory_start_address + buf->integer_write_head_address)) +=
				input->right_channel_buffer[i];
		}
		else
		{
			*((buf->left_channel_physical_memory_start_address + buf->integer_write_head_address)) = 
				input->left_channel_buffer[i];

			*((buf->right_channel_physical_memory_start_address + buf->integer_write_head_address)) =
				input->right_channel_buffer[i];

		}

		buf->integer_write_head_address++;
		if( buf->integer_write_head_address >= buf->buffer_size )
		{
			buf->integer_write_head_address = 0;
			buf->recording_wrapped = 1;
			buf->recording_start = 0;
			buf->recording_end = buf->buffer_size - 1;

			// buf->playback_end = buf->buffer_size - 1;
			// buf->playback_start = 0;
		}
	}

	if( buf->recording_wrapped == 0 )
	{
		// buf->recording_wrapped = 0;
		// buf->playback_end = buf->integer_write_head_address - 1;
		buf->recording_end = buf->integer_write_head_address - 1;

	}

}

//specific wet dry mix function
void m2r_helper_wet_dry_mix( hope_dsp_buffer_struct * input_dry, float dry_gain, hope_dsp_buffer_struct * input_wet, float wet_gain, hope_dsp_buffer_struct * output )
{
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = dry_gain * input_dry->left_channel_buffer[i] + wet_gain * input_wet->left_channel_buffer[i];		
		output->right_channel_buffer[i] = dry_gain * input_dry->right_channel_buffer[i] + wet_gain * input_wet->right_channel_buffer[i];
	}

}

//sum_n input buffers
void m2r_helper_sum_n_buffers( hope_dsp_buffer_struct * input, uint32_t n, hope_dsp_buffer_struct * output, uint32_t pre_zero_output )
{
	if( pre_zero_output )
	{
		for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
		{
			output->left_channel_buffer[i] = 0;
			output->right_channel_buffer[i] = 0;
		}
	}

	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		for( uint32_t j = 0; j < n; j++)
		{
			output->left_channel_buffer[i] += input[j].left_channel_buffer[i];
			output->right_channel_buffer[i] += input[j].right_channel_buffer[i];
		}
	}
}

//sum 2 buffers
void m2r_helper_sum_2_buffers( hope_dsp_buffer_struct * input_1, hope_dsp_buffer_struct * input_2,  hope_dsp_buffer_struct * output )
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = input_1->left_channel_buffer[i] + input_2->left_channel_buffer[i];
		output->right_channel_buffer[i] = input_1->right_channel_buffer[i] + input_2->right_channel_buffer[i];
	}
}
void m2r_helper_zero_buffer( hope_dsp_buffer_struct * input )
{
		for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
		{
			input->left_channel_buffer[i] = 0;
			input->right_channel_buffer[i] = 0;
		}

}

void m2r_helper_increment_n_playing( ct5_buffer_t ** bufs, uint32_t * n )
{
	//first clamp n at 3
	(*n)++;
	if( *n > 3 )
	{
		*n = 3;
	}

	//now we need to rotate the buffers
	ct5_buffer_t * temp_pointer = bufs[M2_RAND_N_BUFFERS - 1];

	for( uint32_t i = (M2_RAND_N_BUFFERS - 1); i > 0; i--)
	{
		bufs[i] = bufs[i - 1];
	}

	bufs[0] = temp_pointer;

	bufs[0]->float_read_head_address = 0;
	bufs[0]->integer_write_head_address = 0;


	bufs[0]->playback_start = 0;
	bufs[0]->playback_end = 0;

	bufs[0]->recording_start = 0;
	bufs[0]->recording_end = 0;
	bufs[0]->recording_wrapped = 0;
	// return n;
}

uint32_t m2r_helper_destructive_scan_for_zero_crossing( hope_dsp_buffer_struct * input )
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
		return 1;
	}

	return 0;
}

void m2r_helper_copy_buffer( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output)
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = input->left_channel_buffer[i];
		output->right_channel_buffer[i] = input->right_channel_buffer[i];
	}
}

void m2r_helper_block_length_fade_in( hope_dsp_buffer_struct *output )
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

void m2r_helper_block_length_fade_out( hope_dsp_buffer_struct *output )
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


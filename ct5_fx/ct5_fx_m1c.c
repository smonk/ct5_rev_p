#include "ct5_buffer.h"
#include "ct5_fx_m1c.h"
#include "../hope_hal/hope_dsp_interface.h"
#include "../hope_hal/hope_pwm_rgb_led.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "../hope_hal/hope_btn_and_sw.h"
#include "../hope_hal/hope_version.h"

#include "ct5_fx_state_machine.h"
#include "ct5_fx_ctl_input.h"

// ███╗   ███╗ ██╗ ██████╗
// ████╗ ████║███║██╔════╝
// ██╔████╔██║╚██║██║     
// ██║╚██╔╝██║ ██║██║     
// ██║ ╚═╝ ██║ ██║╚██████╗
// ╚═╝     ╚═╝ ╚═╝ ╚═════╝

//big font from
//https://www.asciiart.eu/text-to-ascii-art
//used 'ansi shadow' font

#define M1_CT5_LINKS 3

static ct5_buffer_t * bufs[M1_CT5_LINKS];
 
static ct5_buffer_t ct5_buffer_a;
static ct5_buffer_t ct5_buffer_b;
static ct5_buffer_t ct5_buffer_c;


static m1c_variables_t m1c_variables_a;
static m1c_variables_t m1c_variables_b;
static m1c_variables_t m1c_variables_c;

static m1c_variables_t m1c_variables_global;

static m1c_variables_t * m1c_global_variables;

void ct5_fx_m1c_init( void );

state_function_pointer_t m1c_state_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_reset_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

state_function_pointer_t m1c_state_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

state_function_pointer_t m1c_state_overdub_to_change_n( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_change_n( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_change_n_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

// state_function_pointer_t m1c_state_change_n_to_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output ); 

state_function_pointer_t m1c_state_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_overdub_to_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_holding_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );


state_function_pointer_t m1c_state_overdub_to_change_bs( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_change_bs( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

// state_function_pointer_t m1c_state_change_bs_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
// state_function_pointer_t m1c_state_ch_bs_to_od( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );


// state_function_pointer_t m1c_state_h_to_ch_n( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
// state_function_pointer_t m1c_state_change_n_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
// state_function_pointer_t m1c_state_ch_n_to_h( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output ); 

// state_function_pointer_t m1c_state_h_to_ch_bs( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
// state_function_pointer_t m1c_state_change_bs_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
// state_function_pointer_t m1c_state_ch_bs_to_h( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

// state_function_pointer_t m1c_state_holding_to_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

uint32_t m1c_event_freeze_button_is_held( void );
uint32_t m1c_event_n_switch_has_changed( void );
uint32_t m1c_event_buffer_size_has_changed( void );

void m1c_helper_zero_buffer( hope_dsp_buffer_struct * input );
void m1c_helper_copy_buffer( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output);
void m1c_helper_block_length_fade_in( hope_dsp_buffer_struct *output );
void m1c_helper_block_length_fade_out( hope_dsp_buffer_struct *output );
void m1c_helper_wet_dry_mix( hope_dsp_buffer_struct * input_dry, float dry_gain, hope_dsp_buffer_struct * input_wet, float wet_gain, hope_dsp_buffer_struct * output );
void m1c_helper_add_two_dsp_buffers( hope_dsp_buffer_struct * input_1, hope_dsp_buffer_struct * input_2, hope_dsp_buffer_struct * output );
void m1c_helper_ct5_algo( ct5_buffer_t * buf, hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );


state_function_pointer_t link_0_sfp = {0};
state_function_pointer_t link_1_sfp = {0};
state_function_pointer_t link_2_sfp = {0};

static uint32_t m1c_link_n_processing = 0;
static uint32_t m1c_link_n_highlighted = 0;

static uint32_t m1c_link_n_change_lock = 0;


// ██╗███╗   ██╗██╗████████╗
// ██║████╗  ██║██║╚══██╔══╝
// ██║██╔██╗ ██║██║   ██║   
// ██║██║╚██╗██║██║   ██║   
// ██║██║ ╚████║██║   ██║   
// ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝   

void ct5_fx_m1c_init( void )
{

	bufs[0] = &ct5_buffer_a;
	bufs[1] = &ct5_buffer_b;
	bufs[2] = &ct5_buffer_c;

	bufs[0]->m1c_variables = &m1c_variables_a;
	bufs[1]->m1c_variables = &m1c_variables_b;
	bufs[2]->m1c_variables = &m1c_variables_c;

	m1c_global_variables = &m1c_variables_global;

	uint32_t channel_buffer_size = CT5_BUFFER_MEM_SIZE_WORDS / 6;

	for( int32_t i = 0; i < M1_CT5_LINKS; i++ )
	{
		bufs[i]->this_buffers_index = i;

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

		bufs[i]->desired_dir = 0;
		bufs[i]->current_dir = 0;
		bufs[i]->dir_increment = 0.0005;

		//these aren't really needed for m1c
		// bufs[i]->playback_start = 0;
		// bufs[i]->playback_end = 0;
		// bufs[i]->recording_wrapped = 0;

		// bufs[i]->desired_playback_volume = 1.0;
		// bufs[i]->current_playback_volume = 1.0;
		// bufs[i]->playback_volume_increment = 0.0005;		

	}
	
}

// ███╗   ███╗ █████╗ ██╗███╗   ██╗
// ████╗ ████║██╔══██╗██║████╗  ██║
// ██╔████╔██║███████║██║██╔██╗ ██║
// ██║╚██╔╝██║██╔══██║██║██║╚██╗██║
// ██║ ╚═╝ ██║██║  ██║██║██║ ╚████║
// ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝

void ct5_fx_m1c( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//the first step of development is to make the states work for N = 1. let's try that.

	static uint32_t is_init = 0;
	if(!is_init)
	{
		ct5_fx_m1c_init();
		is_init = 1;
		return;
	}

	//the non link associated variable struct, mostly used for mix and n switch
	//but the n switch should not be handled here.

	ct5_fx_get_m1c_variables( m1c_global_variables );
	// m1c_link_n_change_lock = 0;

	// if( m1c_link_n_change_lock == 0 )
	// {
	// 	m1c_link_n_highlighted = my_m1c_variables->m1c_n_switch - 1;	
	// }
	
	//zero the output buffer
	m1c_helper_zero_buffer( output );

	
	hope_dsp_buffer_struct temp_buf;
	temp_buf.num_samples_per_channel = HOPE_DSP_BUFFER_SIZE;
	float temp_left_channel_buffer[HOPE_DSP_BUFFER_SIZE];
	float temp_right_channel_buffer[HOPE_DSP_BUFFER_SIZE];
	temp_buf.left_channel_buffer = temp_left_channel_buffer;
	temp_buf.right_channel_buffer = temp_right_channel_buffer;


	m1c_helper_zero_buffer( &temp_buf );
	m1c_link_n_processing = 0;
	// if( m1c_link_n_highlighted == 0 )
	{
		ct5_fx_get_m1c_variables( bufs[0]->m1c_variables );
	}
	
	//sfp means state function pointer
	link_0_sfp = link_0_sfp.next_state( input, &temp_buf );
	m1c_helper_add_two_dsp_buffers( output, &temp_buf, output );

	// m1c_helper_zero_buffer( &temp_buf );
	// m1c_link_n_processing = 1;
	// if( m1c_link_n_highlighted == 1 )
	// {
	// 	ct5_fx_get_m1c_variables( bufs[1]->m1c_variables );
	// }
	// track_1_sfp = track_1_sfp.next_state( input, &temp_buf );
	// m1c_helper_add_two_dsp_buffers( output, &temp_buf, output );
	
	// m1c_helper_zero_buffer( &temp_buf );
	// m1c_link_n_processing = 2;
	// if( m1c_link_n_highlighted == 2 )
	// {
	// 	ct5_fx_get_m1c_variables( bufs[2]->m1c_variables );
	// }
	// track_2_sfp = track_2_sfp.next_state( input, &temp_buf );
	// m1c_helper_add_two_dsp_buffers( output, &temp_buf, output );
	

	//now at the end we need to mix the input and output accordingly.
	float wet_gain, dry_gain;
	wet_gain = m1c_global_variables->wet_gain;
	dry_gain = m1c_global_variables->dry_gain;

	m1c_helper_wet_dry_mix( input, dry_gain, output, wet_gain, output );


	
}



// ███████╗████████╗ █████╗ ████████╗███████╗███████╗
// ██╔════╝╚══██╔══╝██╔══██╗╚══██╔══╝██╔════╝██╔════╝
// ███████╗   ██║   ███████║   ██║   █████╗  ███████╗
// ╚════██║   ██║   ██╔══██║   ██║   ██╔══╝  ╚════██║
// ███████║   ██║   ██║  ██║   ██║   ███████╗███████║
// ╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚══════╝

extern void (*hope_ct5_led_on_setting)( hope_pwm_rgb_led_struct * );
extern void (*hope_ct5_led_off_setting)( hope_pwm_rgb_led_struct * );

state_function_pointer_t m1c_state_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	
	//this state wipes the ram annd only exits after it is totally wiped, which wil take several blocks of time.

	uint32_t wipe_ratio = 32;
	uint32_t N = HOPE_DSP_BUFFER_SIZE * wipe_ratio;
	static uint32_t n = 0;

	//we need to clear all the ram
	uint32_t start_address = n;
	uint32_t end_address;

	if( n + N > CT5_BUFFER_MEM_SIZE_WORDS )
	{
		end_address = CT5_BUFFER_MEM_SIZE_WORDS;
	}
	else
	{
		end_address = n + N;
	}

	float * buffer = (float *)CT5_BUFFER_MEM_BASE_ADDRESS;

	for( uint32_t i = start_address; i < end_address; i++ )
	{
		buffer[i] = 0.0f;
	}

	n = end_address;

	//this state zeros its output
	m1c_helper_zero_buffer( output );

	state_function_pointer_t sfp;
	if( n == CT5_BUFFER_MEM_SIZE_WORDS )
	{
		sfp.next_state = m1c_state_reset_to_overdub;
		n = 0;
		return sfp;
	}
	else
	{
		sfp.next_state = m1c_state_reset;
		return sfp;
	}
}

state_function_pointer_t m1c_state_reset_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//here we need to start the algorithm but fade in the write and read head volumes.

	// m1c_helper_block_length_fade_in( input );

	bufs[m1c_link_n_processing]->m1c_variables->current_write_head_volume = 0.0f;
	bufs[m1c_link_n_processing]->m1c_variables->desired_write_head_volume = 1.0f;

	bufs[m1c_link_n_processing]->m1c_variables->current_feedback_volume = 0.0f;
	bufs[m1c_link_n_processing]->m1c_variables->use_feedback_volume_override = 0;	

	m1c_helper_ct5_algo( bufs[m1c_link_n_processing], input, output );

	state_function_pointer_t sfp;
	sfp.next_state = m1c_state_overdub;
	return sfp;

}

state_function_pointer_t m1c_state_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//do the thing
	m1c_helper_ct5_algo( bufs[m1c_link_n_processing], input, output );

	state_function_pointer_t sfp;
	
	if( m1c_event_freeze_button_is_held( ) == 1 )
	{
		sfp.next_state = m1c_state_overdub_to_holding;
		return sfp;
	}
	else if( m1c_event_n_switch_has_changed( ) == 1 )
	{
		//todo: this should go to change n but for now we only have 1 n
		sfp.next_state = m1c_state_overdub;
	}
	else if( m1c_event_buffer_size_has_changed( ) == 1 )
	{
		sfp.next_state = m1c_state_overdub_to_change_bs;
	}

	sfp.next_state = m1c_state_overdub;
	return sfp;
}

state_function_pointer_t m1c_state_overdub_to_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	// in this state you keep running the base algo but
	// feedback should be set to 1.
	// the write head is faded out

	// the write head face can be achieved by faing out the input buffer
	bufs[m1c_link_n_processing]->m1c_variables->desired_write_head_volume = 0.0f;

	// somehow we need to set the goal feedback to 1.
	bufs[m1c_link_n_processing]->m1c_variables->desired_feedback_volume_override = 1.0f;
	bufs[m1c_link_n_processing]->m1c_variables->use_feedback_volume_override = 1;	

	// then run the algo
	m1c_helper_ct5_algo( bufs[m1c_link_n_processing], input, output );

	state_function_pointer_t sfp;
	sfp.next_state = m1c_state_holding;
	return sfp;
}

state_function_pointer_t m1c_state_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	// in this state run the base algo with feedback clamped at 1.

	// somehow set the goal feedback to 1

	// then run the algo
	m1c_helper_ct5_algo( bufs[m1c_link_n_processing], input, output );

	//the even to to leave is the hold switch was released

	state_function_pointer_t sfp;
	if( m1c_event_freeze_button_is_held( ) == 0 )
	{
		sfp.next_state = m1c_state_holding_to_overdub;
		return sfp;
	}
	else
	{
		sfp.next_state = m1c_state_holding;
		return sfp;
	}
}

state_function_pointer_t m1c_state_holding_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	// fade back in the write head and also let the feedback goal go back to the pot setting

	// fade back in the write head volume
	bufs[m1c_link_n_processing]->m1c_variables->desired_write_head_volume = 1.0f;

	// somehow deal with feedback value
	bufs[m1c_link_n_processing]->m1c_variables->use_feedback_volume_override = 0;

	// then run the algo
	m1c_helper_ct5_algo( bufs[m1c_link_n_processing], input, output );

	// then return to overdub
	state_function_pointer_t sfp;
	sfp.next_state = m1c_state_overdub;
	return sfp;
}

state_function_pointer_t m1c_state_overdub_to_change_n( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	// n should be able to change without a buffer reset. 
	// you need to mute the feedback head

	// somehow mute the feedback head
	bufs[m1c_link_n_processing]->m1c_variables->desired_feedback_volume_override = 0.0f;
	bufs[m1c_link_n_processing]->m1c_variables->use_feedback_volume_override = 1;

	// then run the algo
	m1c_helper_ct5_algo( bufs[m1c_link_n_processing], input, output );

	// then return to overdub
	state_function_pointer_t sfp;
	sfp.next_state = m1c_state_change_n;
	return sfp;

}

state_function_pointer_t m1c_state_change_n( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	//feedback has to be zero.

	//change feedback pointer

	// run the algo
	m1c_helper_ct5_algo( bufs[m1c_link_n_processing], input, output );

	// then return to overdub
	state_function_pointer_t sfp;
	sfp.next_state = m1c_state_change_n_to_overdub;
	return sfp;
}

state_function_pointer_t m1c_state_change_n_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	// bring back feedback to pot level
	bufs[m1c_link_n_processing]->m1c_variables->use_feedback_volume_override = 0;

	// run the algo
	m1c_helper_ct5_algo( bufs[m1c_link_n_processing], input, output );

	// then return to overdub
	state_function_pointer_t sfp;
	sfp.next_state = m1c_state_overdub;
	return sfp;
}

state_function_pointer_t m1c_state_overdub_to_change_bs( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//we need to mute the wet out
	bufs[m1c_link_n_processing]->m1c_variables->desired_write_head_volume = 0.0f;

	//run the algo
	m1c_helper_ct5_algo(bufs[m1c_link_n_processing], input, output );

	//now fade out the output
	m1c_helper_block_length_fade_out( output );



	state_function_pointer_t sfp;
	sfp.next_state = m1c_state_change_bs;
	return sfp;
}

state_function_pointer_t m1c_state_change_bs( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//change the buffer length somehow
	bufs[m1c_link_n_processing]->m1c_variables->current_buffer_size = bufs[m1c_link_n_processing]->m1c_variables->desired_buffer_size;

	//output should be zero
	m1c_helper_zero_buffer( output );

	state_function_pointer_t sfp;
	sfp.next_state = m1c_state_reset;
	return sfp;
}

// ███████╗██╗   ██╗███████╗███╗   ██╗████████╗███████╗
// ██╔════╝██║   ██║██╔════╝████╗  ██║╚══██╔══╝██╔════╝
// █████╗  ██║   ██║█████╗  ██╔██╗ ██║   ██║   ███████╗
// ██╔══╝  ╚██╗ ██╔╝██╔══╝  ██║╚██╗██║   ██║   ╚════██║
// ███████╗ ╚████╔╝ ███████╗██║ ╚████║   ██║   ███████║
// ╚══════╝  ╚═══╝  ╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝

extern hope_btn_and_sw_struct my_btn_and_sw[ HOPE_NUM_BTN_AND_SW ];

uint32_t m1c_event_freeze_button_is_held( )
{
	if( my_btn_and_sw[6].hold_event_flag == 1 )
	{
		my_btn_and_sw[6].hold_event_flag = 0;
		return 1;
	}
	return 0;
}

uint32_t m1c_event_n_switch_has_changed( )
{

	return 0;
}

uint32_t m1c_event_buffer_size_has_changed( )
{
	if( bufs[m1c_link_n_processing]->m1c_variables->current_buffer_size != bufs[m1c_link_n_processing]->m1c_variables->desired_buffer_size )
	{
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

void m1c_helper_copy_buffer( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output)
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = input->left_channel_buffer[i];
		output->right_channel_buffer[i] = input->right_channel_buffer[i];
	}
}

void m1c_helper_block_length_fade_in( hope_dsp_buffer_struct *output )
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

void m1c_helper_block_length_fade_out( hope_dsp_buffer_struct *output )
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

void m1c_helper_zero_buffer( hope_dsp_buffer_struct * input )
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		input->left_channel_buffer[i] = 0;
		input->right_channel_buffer[i] = 0;
	}

}

void m1c_helper_wet_dry_mix( hope_dsp_buffer_struct * input_dry, float dry_gain, hope_dsp_buffer_struct * input_wet, float wet_gain, hope_dsp_buffer_struct * output )
{
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = dry_gain * input_dry->left_channel_buffer[i] + wet_gain * input_wet->left_channel_buffer[i];		
		output->right_channel_buffer[i] = dry_gain * input_dry->right_channel_buffer[i] + wet_gain * input_wet->right_channel_buffer[i];
	}

}

void m1c_helper_add_two_dsp_buffers( hope_dsp_buffer_struct * input_1, hope_dsp_buffer_struct * input_2, hope_dsp_buffer_struct * output )
{
	for( uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = input_1->left_channel_buffer[i] + input_2->left_channel_buffer[i];
		output->right_channel_buffer[i] = input_1->right_channel_buffer[i] + input_2->right_channel_buffer[i];
	}
}

void m1c_helper_ct5_algo( ct5_buffer_t * buf, hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	//we need to pre caclulate the write head address vector so that we can do proper collision detecion
	uint32_t write_head_address_vector[ HOPE_DSP_BUFFER_SIZE ];

	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		write_head_address_vector[i] =  ( ( i + buf->integer_write_head_address) % buf->buffer_size );
	}

	buf->integer_write_head_address = 
		( buf->integer_write_head_address + HOPE_DSP_BUFFER_SIZE ) % buf->buffer_size;

	// static float temp_dir = 1.0;
	//next calculate the read address vector using dir
	buf->desired_dir = buf->m1c_variables->desired_dir; 
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

		if( buf->float_read_head_address > buf->m1c_variables->current_buffer_size )
		{
			buf->float_read_head_address = buf->float_read_head_address - buf->m1c_variables->current_buffer_size;
		}

		if( buf->float_read_head_address < 0 )
		{
			buf->float_read_head_address = buf->float_read_head_address + buf->m1c_variables->current_buffer_size;
		}

		read_head_address_vector[i] = buf->float_read_head_address;
	}


	//next calculate the read head volume based on collision detection	
	float collision_zone_size = 100.0;
	float dead_zone_distance = 5.0;
	float read_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];

	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		float address_difference = fabs(read_head_address_vector[i] - (float)write_head_address_vector[i]);
		if( address_difference < dead_zone_distance )
		{
			read_head_volume_vector[i] = 0.0;
		}
		else if(  address_difference  < collision_zone_size )
		{
			// read_head_volume_vector[i] = address_difference/collision_zone_size;
			read_head_volume_vector[i] = (address_difference - dead_zone_distance) / (collision_zone_size - dead_zone_distance);

		}
		else if ( address_difference > buf->m1c_variables->current_buffer_size - dead_zone_distance )
		{
			read_head_volume_vector[i] = 0.0;
		}
		else if ( address_difference > buf->m1c_variables->current_buffer_size - collision_zone_size )
		{
			// read_head_volume_vector[i] = (buf->buffer_size - address_difference)/collision_zone_size;
			read_head_volume_vector[i] = ( buf->m1c_variables->current_buffer_size - address_difference - dead_zone_distance )/(collision_zone_size - dead_zone_distance);
		}
		else
		{
			read_head_volume_vector[i] = 1.0;
		}

	}

	//calculate the write head volume vector
	float write_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	float temp_wh_increment = ( buf->m1c_variables->desired_write_head_volume - buf->m1c_variables->current_write_head_volume )/HOPE_DSP_BUFFER_SIZE;
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		write_head_volume_vector[i] = buf->m1c_variables->current_write_head_volume + ((i + 1) * temp_wh_increment);
		//clamp the value between 0 and 1
		if( write_head_volume_vector[i] > 1.0 )
		{
			write_head_volume_vector[i] = 1.0;
		}
		else if( write_head_volume_vector[i] < 0.0 )
		{
			write_head_volume_vector[i] = 0.0;
		}
	}
	buf->m1c_variables->current_write_head_volume = write_head_volume_vector[HOPE_DSP_BUFFER_SIZE - 1];

	//calculate the feedback volume vector
	//TODO MAKE THIS READ FROM THE POT
	float feedback_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	float temp_desired_feedback_volume = buf->m1c_variables->desired_feedback_volume;
	if( buf->m1c_variables->use_feedback_volume_override )
	{
		temp_desired_feedback_volume = buf->m1c_variables->desired_feedback_volume_override;
	}
	float temp_fb_increment = ( temp_desired_feedback_volume - buf->m1c_variables->current_feedback_volume )/HOPE_DSP_BUFFER_SIZE;
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		feedback_volume_vector[i] = buf->m1c_variables->current_feedback_volume + ((i + 1) * temp_fb_increment);
		//clamp the value between 0 and 1
		if( feedback_volume_vector[i] > 1.0 )
		{
			feedback_volume_vector[i] = 1.0;
		}
		else if( feedback_volume_vector[i] < 0.0 )
		{
			feedback_volume_vector[i] = 0.0;
		}
	}
	buf->m1c_variables->current_feedback_volume = feedback_volume_vector[HOPE_DSP_BUFFER_SIZE - 1];

	//now we need the actual audio signals
	//use linear interpolation to assemble read data
	float read_head_data_left[ HOPE_DSP_BUFFER_SIZE ];
	float read_head_data_right[ HOPE_DSP_BUFFER_SIZE ];

	float alpha, beta;
	uint32_t low_address, high_address;
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		low_address = (uint32_t)read_head_address_vector[i];
		high_address = (uint32_t)read_head_address_vector[i] + 1;
		if( high_address >= buf->m1c_variables->current_buffer_size )
		{
			high_address = high_address - buf->m1c_variables->current_buffer_size;
		}

		alpha = read_head_address_vector[i] - low_address;
		beta = 1.0 - alpha;

		read_head_data_left[i] = 
				beta * (*(low_address + buf->left_channel_physical_memory_start_address)) 
			+ 	alpha * (*((high_address + buf->left_channel_physical_memory_start_address))) ;
	
		read_head_data_left[i] *= read_head_volume_vector[i];


		read_head_data_right[i] = 
				beta * (*(low_address + buf->right_channel_physical_memory_start_address))
			+ 	alpha * (*((high_address + buf->right_channel_physical_memory_start_address))) ;

		read_head_data_right[i] *= read_head_volume_vector[i];

		//now write
		*((write_head_address_vector[i] + buf->left_channel_physical_memory_start_address)) = 
			read_head_data_left[i] * feedback_volume_vector[i] 
			+ input->left_channel_buffer[i] * write_head_volume_vector[i];

		*((write_head_address_vector[i] + buf->right_channel_physical_memory_start_address)) =
			read_head_data_right[i] * feedback_volume_vector[i]
			+ input->right_channel_buffer[i] * write_head_volume_vector[i];
	}

	//the end?

}
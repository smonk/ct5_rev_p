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

static ct5_buffer_t * bufs[M2_RAND_N_BUFFERS];
 
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

state_function_pointer_t m1c_state_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_od_to_ch_n( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_change_n_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_ch_n_to_od( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output ); 

state_function_pointer_t m1c_state_od_to_ch_bs( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_change_bs_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_ch_bs_to_od( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

state_function_pointer_t m1c_state_overdub_to_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

state_function_pointer_t m1c_overdub_to_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_holding_to_overdub( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

state_function_pointer_t m1c_state_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_h_to_ch_n( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_change_n_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_ch_n_to_h( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output ); 

state_function_pointer_t m1c_state_h_to_ch_bs( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_change_bs_holding( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );
state_function_pointer_t m1c_state_ch_bs_to_h( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

state_function_pointer_t m1c_state_holding_to_reset( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );







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
	
}

// ███╗   ███╗ █████╗ ██╗███╗   ██╗
// ████╗ ████║██╔══██╗██║████╗  ██║
// ██╔████╔██║███████║██║██╔██╗ ██║
// ██║╚██╔╝██║██╔══██║██║██║╚██╗██║
// ██║ ╚═╝ ██║██║  ██║██║██║ ╚████║
// ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝

void ct5_fx_m1c( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{

	static uint32_t is_init = 0;
	if(!is_init)
	{
		ct5_fx_m1c_init();
		is_init = 1;
		return;
	}


	
}



// ███████╗████████╗ █████╗ ████████╗███████╗███████╗
// ██╔════╝╚══██╔══╝██╔══██╗╚══██╔══╝██╔════╝██╔════╝
// ███████╗   ██║   ███████║   ██║   █████╗  ███████╗
// ╚════██║   ██║   ██╔══██║   ██║   ██╔══╝  ╚════██║
// ███████║   ██║   ██║  ██║   ██║   ███████╗███████║
// ╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚══════╝


// ███████╗██╗   ██╗███████╗███╗   ██╗████████╗███████╗
// ██╔════╝██║   ██║██╔════╝████╗  ██║╚══██╔══╝██╔════╝
// █████╗  ██║   ██║█████╗  ██╔██╗ ██║   ██║   ███████╗
// ██╔══╝  ╚██╗ ██╔╝██╔══╝  ██║╚██╗██║   ██║   ╚════██║
// ███████╗ ╚████╔╝ ███████╗██║ ╚████║   ██║   ███████║
// ╚══════╝  ╚═══╝  ╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝


// ██╗  ██╗███████╗██╗     ██████╗ ███████╗██████╗ ███████╗
// ██║  ██║██╔════╝██║     ██╔══██╗██╔════╝██╔══██╗██╔════╝
// ███████║█████╗  ██║     ██████╔╝█████╗  ██████╔╝███████╗
// ██╔══██║██╔══╝  ██║     ██╔═══╝ ██╔══╝  ██╔══██╗╚════██║
// ██║  ██║███████╗███████╗██║     ███████╗██║  ██║███████║
// ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝╚══════╝

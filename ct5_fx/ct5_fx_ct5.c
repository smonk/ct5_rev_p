#include "ct5_buffer.h"
#include "ct5_fx_ct5.h"
#include "../hope_hal/hope_dsp_interface.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

ct5_buffer_t * buf;

ct5_buffer_t my_ct5_buffer;

//this value is in samples
#define MAX_CT5_BUFFER_SIZE 349524

void ct5_fx_ct5_init()
{

	buf = &my_ct5_buffer;

	//the memory space is 8388608 bytes
	//divided by 3 and rounded down to something nice is 2796200
	//that divided by 2 is 1398100
	//remember addressess here count bytes
	buf->left_channel_physical_memory_start_address = (float *)CT5_BUFFER_MEM_BASE_ADDRESS;
	buf->right_channel_physical_memory_start_address = (float *)(CT5_BUFFER_MEM_BASE_ADDRESS + 1398100);

	buf->left_channel_physical_memory_end_address = (float *)(buf->right_channel_physical_memory_start_address -4);
	buf->right_channel_physical_memory_end_address = (float *)(CT5_BUFFER_MEM_BASE_ADDRESS + 2796200 -4);

	//this is in words though
	buf->buffer_size = MAX_CT5_BUFFER_SIZE/16; //the max length in 32 bit floats

	buf->block_size = HOPE_DSP_BUFFER_SIZE;

	buf->float_read_head_address = buf->buffer_size / 2.0;

	buf->integer_write_head_address = 0;

	buf->desired_dir = 1.0;
	buf->current_dir = 1.0;
	buf->dir_increment = 0.0005;

}

void ct5_fx_ct5( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output )
{
	static uint32_t algo_is_init = 0;
	static uint32_t algo_n = 0;
	if( algo_is_init == 0 )
	{
		ct5_fx_ct5_init();
		algo_is_init = 1;
		return;
	}

	//we need to pre caclulate the write head address vector so that we can do proper collision detecion
	uint32_t write_head_address_vector[ HOPE_DSP_BUFFER_SIZE ];

	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		write_head_address_vector[i] =  ( ( i + buf->integer_write_head_address) % buf->buffer_size );
	}
	buf->integer_write_head_address = 
		( buf->integer_write_head_address + HOPE_DSP_BUFFER_SIZE ) % buf->buffer_size;

	static float temp_dir = 1.0;
	//next calculate the read address vector using dir
	if(algo_n % 500 == 0)
	{
		temp_dir = rand() / (float)RAND_MAX;
		temp_dir -= 0.5;
		temp_dir *= 2.0;

		if( temp_dir > 0.0)
		{
			temp_dir = (temp_dir * 1.75) + .25;
		}
		else
		{
			temp_dir = (temp_dir * 1.75) - .25;
		}

	}
	buf->desired_dir = temp_dir;
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

		if( buf->float_read_head_address > buf->buffer_size )
		{
			buf->float_read_head_address = buf->float_read_head_address - buf->buffer_size;
		}

		if( buf->float_read_head_address < 0 )
		{
			buf->float_read_head_address = buf->float_read_head_address + buf->buffer_size;
		}

		read_head_address_vector[i] = buf->float_read_head_address;
	}


	//next calculate the read head volume based on collision detection
	
	float collision_zone_size = 100.0;
	float dead_zone_distance = 5.0;
	float read_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	volatile uint32_t temp = 0;
	uint32_t prev = 0;
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
		else if ( address_difference > buf->buffer_size - dead_zone_distance )
		{
			read_head_volume_vector[i] = 0.0;
		}
		else if ( address_difference > buf->buffer_size - collision_zone_size )
		{
			// read_head_volume_vector[i] = (buf->buffer_size - address_difference)/collision_zone_size;
			read_head_volume_vector[i] = (buf->buffer_size - address_difference - dead_zone_distance )/(collision_zone_size - dead_zone_distance);
		}
		else
		{
			read_head_volume_vector[i] = 1.0;
		}

	}

	if( prev == 1 )
	{
		temp++;
	}

	//calculate the write head volume vector
	float write_head_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		write_head_volume_vector[i] = 1.0 ;
	}

	//calculate the feedback volume vector
	float feedback_volume_vector[ HOPE_DSP_BUFFER_SIZE ];
	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		feedback_volume_vector[i] = 0.8 ;
	}


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
		if( high_address >= buf->buffer_size )
		{
			high_address = high_address - buf->buffer_size;
		}

		alpha = read_head_address_vector[i] - low_address;
		beta = 1.0 - alpha;
		read_head_data_left[i] = 
				beta * (*(low_address + buf->left_channel_physical_memory_start_address)) 
			+ 	alpha * (*((high_address + buf->left_channel_physical_memory_start_address))) ;
		// read_head_data_left[i] = 
		// 		1 * (*(low_address + buf->left_channel_physical_memory_start_address)) ;
		// 	// + 	alpha * (*((high_address + buf->left_channel_physical_memory_start_address))) ;
	
		read_head_data_left[i] *= read_head_volume_vector[i];

		read_head_data_right[i] = 
				beta * (*(low_address + buf->right_channel_physical_memory_start_address))
			+ 	alpha * (*((high_address + buf->right_channel_physical_memory_start_address))) ;

		read_head_data_right[i] *= read_head_volume_vector[i];

		//now write
		*((write_head_address_vector[i] + buf->left_channel_physical_memory_start_address)) = 
			read_head_data_left[i] * feedback_volume_vector[i] 
			+ input->left_channel_buffer[i] * write_head_volume_vector[i];
		// *((write_head_address_vector[i] + buf->left_channel_physical_memory_start_address)) = 
		// 	write_head_volume_vector[i];
			//read_head_data_left[i] * feedback_volume_vector[i] 

		*((write_head_address_vector[i] + buf->right_channel_physical_memory_start_address)) =
			read_head_data_right[i] * feedback_volume_vector[i]
			+ input->right_channel_buffer[i] * write_head_volume_vector[i];
	}


	//now to cacluate the output of the algo to push to the codec
	//this should be the input weighted by 1 -the mix pot, plus the read head data weighted by the mix pot
	float dry_gain, wet_gain;
	wet_gain = 1; //should actually be the mix pot
	dry_gain = 1 - wet_gain;

	for(uint32_t i = 0; i < HOPE_DSP_BUFFER_SIZE; i++)
	{
		output->left_channel_buffer[i] = dry_gain * input->left_channel_buffer[i] + wet_gain * read_head_data_left[i];
		
		output->right_channel_buffer[i] = dry_gain * input->right_channel_buffer[i] + wet_gain * read_head_data_right[i];
	}

	//the end?
	algo_n++;
}

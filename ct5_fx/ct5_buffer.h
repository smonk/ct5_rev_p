#ifndef CT5_BUFFER_H
#define CT5_BUFFER_H

#include <stdint.h>

#define CT5_BUFFER_MEM_SIZE_BYTES ( 8 * 1024 * 1024 )
#define CT5_BUFFER_MEM_SIZE_WORDS ( CT5_BUFFER_MEM_SIZE_BYTES / 4 )
//octospi1 base address
#define CT5_BUFFER_MEM_BASE_ADDRESS ( 0x90000000 )

typedef struct {
	
	//these are actual memory addresses
	float * physical_memory_start_address;
	float * physical_memory_end_address;

	volatile float * left_channel_physical_memory_start_address;
	volatile float * left_channel_physical_memory_end_address;
	volatile float * right_channel_physical_memory_start_address;
	volatile float * right_channel_physical_memory_end_address;

	//this should be the difference of end-start, divided by 2 since stereo, or can set lower in software.
	uint32_t buffer_size;

	//the block size, should be HOPE_DSP_BUFFER_SIZE, how many samples per block are processed
	uint32_t block_size;

	float float_read_head_address;

	uint32_t read_data_cache_base_address;

	float * read_data_cache; //a buffer of size HOPE_DSP_BUFFER_SIZE * HOPE_DSP_NUM_CHANNELS * 4. Becasue we need 2x in front and 2x in behind. integer read head addess is the middle

	//the write head address is always an integer and we always write in a continuous block
	uint32_t integer_write_head_address;


	uint32_t playback_counter;

	//the read head dir data
	float desired_dir; //comes form the pot or whatever control
	float current_dir; //is this needed?
	float dir_increment;
	float * dir_vector;

	//this is the read head gain.
	float * read_head_gain_vector;

	//this is the write head gain
	float * write_head_gain_vector;

	//we want some sort of playback boundaries
	uint32_t playback_start;
	uint32_t playback_end;
	uint32_t playback_length;

	uint32_t recording_start;
	uint32_t recording_end;
	uint32_t recording_length;
	uint32_t recording_wrapped;


} ct5_buffer_t;

#endif
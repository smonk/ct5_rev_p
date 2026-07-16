#ifndef CT5_FX_STATE_MACHINE_H
#define CT5_FX_STATE_MACHINE_H

#include "../hope_hal/hope_dsp_interface.h"

struct state_function_ponter;

typedef struct state_function_pointer (*state_function)( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output );

typedef struct state_function_pointer
{
	state_function next_state;
}state_function_pointer_t;

#endif
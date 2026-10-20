#ifndef CT5_FX_SELECTOR_H
#define CT5_FX_SELECTOR_H

#include "../hope_hal/hope_btn_and_sw.h"
#include "../hope_hal/hope_dsp_interface.h"

void ct5_fx_select( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output, hope_btn_and_sw_struct * btn_and_sw);

#endif
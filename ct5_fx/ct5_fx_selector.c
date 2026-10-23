#include "ct5_fx_selector.h"

#include "ct5_fx_ct5.h"
#include "ct5_fx_m1m.h"
#include "ct5_fx_m2r.h"
#include "ct5_fx_m3t.h"

#include "../hope_hal/hope_dsp_interface.h"
#include "../hope_hal/hope_btn_and_sw.h"
void ct5_fx_select(hope_dsp_buffer_struct *input, hope_dsp_buffer_struct *output,
              hope_btn_and_sw_struct *btn_and_sw)
{
    // check something about bypass?

    static uint32_t current_fx_number = 0;
    static uint32_t desired_fx_number = 0;
    static hope_dsp_control_struct control_struct;
    static hope_dsp_control_struct * ctl;
    static uint32_t ctl_is_init = 0;

    if( ctl_is_init == 0 )
    {
        ctl_is_init = 1;
        ctl = &control_struct;
        ctl->exit_request = 0;
        ctl->exit_acknowledge = 0;
    }
 
 
    // check m switch
    //0 IS M DOWN
    //1 IS M UP
    if ( btn_and_sw[0].is_pressed_flag ) 
    {
        desired_fx_number = 3;
    }
    else if ( btn_and_sw[1].is_pressed_flag ) 
    {
        desired_fx_number = 1;
    }
    else 
    {
        desired_fx_number = 2;
    }

    if( current_fx_number != desired_fx_number ) 
    {
        if( ctl->exit_acknowledge == 0 )
        {
            ctl->exit_request = 1;
        }
        else
        {
            ctl->exit_request = 0;
            ctl->exit_acknowledge = 0;
            current_fx_number = desired_fx_number;
            //clear the button flags here
            btn_and_sw[6].pressed_event_flag = 0;
            btn_and_sw[6].released_event_flag = 0;
            btn_and_sw[6].tap_event_flag = 0;
            btn_and_sw[6].hold_event_flag = 0;

            btn_and_sw[7].pressed_event_flag = 0;
            btn_and_sw[7].released_event_flag = 0;
            btn_and_sw[7].tap_event_flag = 0;
            btn_and_sw[7].hold_event_flag = 0;


        }
        
    }

    // a function pointer
    void (*fx_ptr)( hope_dsp_buffer_struct * input, hope_dsp_buffer_struct * output, hope_dsp_control_struct * ctl );

    switch( current_fx_number )
    {
        case 0:
            fx_ptr = ct5_fx_mute;
            break;
        case 1:
            fx_ptr = ct5_fx_m1m;
            break;
        case 2:
            fx_ptr = ct5_fx_m2r;
            // fx_ptr = ct5_fx_mute;
            break;
        case 3:
            fx_ptr = ct5_fx_m3t;
            break;
        default:
            fx_ptr = ct5_fx_mute;
            break;
    }

    fx_ptr( input, output, ctl );
}
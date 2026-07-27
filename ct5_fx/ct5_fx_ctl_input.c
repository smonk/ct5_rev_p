#include "ct5_fx_ctl_input.h"
#include "ct5_fx_m2r.h"
#include "ct5_fx_q_tables.h"
#include "hope_version.h"

#include "hope_btn_and_sw.h"
#include "hope_pot_and_cvin.h"

#include <math.h>

//the global hmi data
extern hope_btn_and_sw_struct my_btn_and_sw[ HOPE_NUM_BTN_AND_SW ];
extern hope_pot_and_cvin_struct my_pot_and_cvin[ HOPE_NUM_POTS_AND_CVIN ];

uint32_t interpret_n_switch( void );

float interpret_mix_pot_dry_gain( float );
float interpret_mix_pot_wet_gain( float );
float interpret_desired_dir( float );
float interpret_slice_length( float );
float interpret_start_randomization( float );

float interpret_playback_volume( float );
float interpret_overdub_decay( float );


#define CT5_FX_Q_COUNTER_MAX ( 5 )
static int32_t q_counter = 0;

float stash_pot_array[3][ HOPE_NUM_POTS_AND_CVIN ] = {0};

//1 means use stashed, 0 means use live
uint8_t stashed_or_live_array[3][ HOPE_NUM_POTS_AND_CVIN ] = {0};

void ct5_fx_get_m2r_variables(m2r_variables_t *m2r_variables)
{

	m2r_variables->m2r_n_switch = interpret_n_switch( );	
	m2r_variables->wet_gain = interpret_mix_pot_wet_gain( my_pot_and_cvin[ 3 ].normalized_value );
	m2r_variables->dry_gain = interpret_mix_pot_dry_gain( my_pot_and_cvin[ 3 ].normalized_value );
	m2r_variables->desired_dir = interpret_desired_dir( my_pot_and_cvin[ 2 ].normalized_value );
	m2r_variables->playback_slice_length = interpret_slice_length( my_pot_and_cvin[ 1 ].normalized_value );
	m2r_variables->playback_start_randomization = interpret_start_randomization( my_pot_and_cvin[ 0 ].normalized_value );
}

void ct5_fx_get_m3t_variables(m3t_variables_t *m3t_variables)
{
	m3t_variables->m3t_n_switch = interpret_n_switch();


	static uint32_t last_n_switch = 0;
	static uint32_t current_n_switch = 0;
	// uint32_t state_for_pot_selection = 0;
	//read the N switch and use it make number bettween 1 and 3.
	
	last_n_switch = current_n_switch;
    current_n_switch = m3t_variables->m3t_n_switch;

    if( last_n_switch != current_n_switch )
	{
		//this means n changed so we need to stash the pot values
		//the state we are leaving needs to get locked in
		uint32_t stash_state =  current_n_switch - 1;
		stash_pot_array[ stash_state ][ 0 ] = my_pot_and_cvin[ 0 ].normalized_value;
		stash_pot_array[ stash_state ][ 1 ] = my_pot_and_cvin[ 1 ].normalized_value;
		stash_pot_array[ stash_state ][ 2 ] = my_pot_and_cvin[ 2 ].normalized_value;
		stash_pot_array[ stash_state ][ 3 ] = my_pot_and_cvin[ 3 ].normalized_value;
		stash_pot_array[ stash_state ][ 4 ] = my_pot_and_cvin[ 4 ].normalized_value;

		stashed_or_live_array[ stash_state ][ 0 ] = 1;
		stashed_or_live_array[ stash_state ][ 1 ] = 1;
		stashed_or_live_array[ stash_state ][ 2 ] = 1;
		stashed_or_live_array[ stash_state ][ 3 ] = 1;
		stashed_or_live_array[ stash_state ][ 4 ] = 1;

	}

	//now we need to check if any of the pots have moved.
	//movement means dont use the stashed state
	//we determine they have moved if the absolute difference between live and stashed is greater than some delta
	uint32_t current_state = current_n_switch - 1;
	float delta = 0.01;

	for( uint32_t i = 0; i < HOPE_NUM_POTS_AND_CVIN; i++ )
	{
		// if( stashed_or_live_array[ current_state ][ i ] == 1 )
		// {
			if( fabs( my_pot_and_cvin[ i ].normalized_value - stash_pot_array[ current_state ][ i ] ) > delta )
			{
				stashed_or_live_array[ current_state ][ i ] = 0;
			}
		// }
	}



	if( stashed_or_live_array[ current_state ][ 3 ] == 1 )
	{
		// m3t_variables->wet_gain = interpret_mix_pot_wet_gain( stash_pot_array[ current_state ][ 3 ] );	
		// m3t_variables->dry_gain = interpret_mix_pot_dry_gain( stash_pot_array[ current_state ][ 3 ] );
	}
	else
	{
		m3t_variables->wet_gain = interpret_mix_pot_wet_gain( my_pot_and_cvin[ 3 ].normalized_value );	
		m3t_variables->dry_gain = interpret_mix_pot_dry_gain( my_pot_and_cvin[ 3 ].normalized_value );
	}

	if( stashed_or_live_array[ current_state ][ 2 ] == 1 )
	{
		// m3t_variables->desired_dir = interpret_desired_dir( stash_pot_array[ current_state ][ 2 ] );
	}
	else
	{
		m3t_variables->desired_dir = interpret_desired_dir( my_pot_and_cvin[ 2 ].normalized_value );
	}
	
	if( stashed_or_live_array[ current_state ][ 1 ] == 1 )
	{
		// m3t_variables->playback_volume = interpret_playback_volume( stash_pot_array[ current_state ][ 1 ] );
	}
	else
	{
		m3t_variables->playback_volume = interpret_playback_volume( my_pot_and_cvin[ 1 ].normalized_value );
	}

	if( stashed_or_live_array[ current_state ][ 0 ] == 1 )
	{
		// m3t_variables->overdub_decay = interpret_overdub_decay( stash_pot_array[ current_state ][ 0 ] );
	}
	else
	{
		m3t_variables->overdub_decay = interpret_overdub_decay( my_pot_and_cvin[ 0 ].normalized_value );
	}
	// m3t_variables->playback_volume = interpret_playback_volume();
	// m3t_variables->overdub_decay = interpret_overdub_decay();
}

// hope_btn_and_sw_struct_init(&l[0], GPIOB, 1 << 12, dbT, tht, "M_DOWN"); 
// hope_btn_and_sw_struct_init(&l[1], GPIOB, 1 << 13, dbT, tht, "M_UP"); 
// hope_btn_and_sw_struct_init(&l[2], GPIOB, 1 << 14, dbT, tht, "Q_DOWN");
// hope_btn_and_sw_struct_init(&l[3], GPIOB, 1 << 15, dbT, tht, "Q_UP");
// hope_btn_and_sw_struct_init(&l[4], GPIOD, 1 << 10, dbT, tht, "E_DOWN");
// hope_btn_and_sw_struct_init(&l[5], GPIOD, 1 << 11, dbT, tht, "E_UP");
// hope_btn_and_sw_struct_init(&l[6], GPIOD, 1 << 6, dbT, tht, "FTSW2");
// hope_btn_and_sw_struct_init(&l[7], GPIOD, 1 << 7, dbT, tht, "FTSW1");
// hope_btn_and_sw_struct_init(&l[8], GPIOD, 1 << 15, dbT, tht, "EFTSW1");
// hope_btn_and_sw_struct_init(&l[9], GPIOC, 1 << 6, dbT, tht, "EFTSW2");
// hope_btn_and_sw_struct_init(&l[10], GPIOC, 1 << 7, dbT, tht, "EFTSW_DET");
// hope_btn_and_sw_struct_init(&l[11], GPIOC, 1 << 8, dbT, tht, "EXP_DET");

uint32_t interpret_n_switch( )
{
	//up is 1, middle is 2, down is 3.
	if( my_btn_and_sw[ 5 ].is_pressed_flag )
	{
		return 1;
	}
	else if( my_btn_and_sw[ 4 ].is_pressed_flag )
	{
		return 3;
	}
	else
	{
		return 2;
	}

}

// hope_pot_and_cvin_struct_init( &l[0], "pot1" );
// hope_pot_and_cvin_struct_init( &l[1], "pot2" );
// hope_pot_and_cvin_struct_init( &l[2], "pot3" );
// hope_pot_and_cvin_struct_init( &l[3], "pot4" );
// hope_pot_and_cvin_struct_init( &l[4], "exp_in" );

float interpret_mix_pot_dry_gain( float val)
{
	float temp = val; //my_pot_and_cvin[ 3 ].normalized_value;

	//make sure temp is between 0 and 1
	if( temp < 0.0 )
	{
		temp = 0.0;
	}
	else if( temp > 1.0 )
	{
		temp = 1.0;
	}

	//now adjust its value
	if( temp < 0.5 )
	{
		return 1.0;
	}
	else
	{
		return ( -2.0*temp + 2.0 );
	}
}

float interpret_mix_pot_wet_gain( float val )
{
	float temp = val; //my_pot_and_cvin[ 3 ].normalized_value;

	//make sure temp is between 0 and 1
	if( temp < 0.0 )
	{
		temp = 0.0;
	}
	else if( temp > 1.0 )
	{
		temp = 1.0;
	}

	//now adjust its value
	if( temp < 0.5 )
	{
		return 2* temp;
	}
	else
	{
		return 1;
	}
}

float interpret_desired_dir( float val )
{
	float temp = val; //my_pot_and_cvin[ 2 ].normalized_value;

	//read the q switch for was tapped
	if( my_btn_and_sw[ 2 ].tap_event_flag )
	{
		my_btn_and_sw[ 2 ].tap_event_flag = 0;
		q_counter--;
		if( q_counter < 0 )
		{
			q_counter = CT5_FX_Q_COUNTER_MAX;
		}
	}

	if( my_btn_and_sw[ 3 ].tap_event_flag )
	{
		my_btn_and_sw[ 3 ].tap_event_flag = 0;
		q_counter++;
		if( q_counter > CT5_FX_Q_COUNTER_MAX )
		{
			q_counter = 0;
		}
	}
	//the q_counter controls which array we look at.
	//the pot value controls which value in that array we look at.
	return get_q_table_value( q_counter, temp );
}

float interpret_slice_length( float val )
{
	return val;//my_pot_and_cvin[ 1 ].normalized_value;
}

float interpret_start_randomization( float val )
{
	return val;//my_pot_and_cvin[ 0 ].normalized_value;	
}

float interpret_playback_volume( float val )
{
	float temp = val; //my_pot_and_cvin[ 1 ].normalized_value;
	if( temp <= 0.5 )
	{
		temp = temp *0.2;
	}
	else 
	{
		temp = 0.1 + ( temp - 0.5 )*(1.8);
	}
	return temp;
}

float interpret_overdub_decay( float val )
{
	return val; //my_pot_and_cvin[ 0 ].normalized_value;	
}

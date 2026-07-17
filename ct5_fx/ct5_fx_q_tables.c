#include <stdint.h>
#include "ct5_fx_q_tables.h"
#include "ct5_fx_m2r.h"

static const float q_table_chromatic[50] = 
{
   -2.0000,
   -1.8877,
   -1.7818,
   -1.6818,
   -1.5874,
   -1.4983,
   -1.4142,
   -1.3348,
   -1.2599,
   -1.1892,
   -1.1225,
   -1.0595,
   -1.0000,
   -0.9439,
   -0.8909,
   -0.8409,
   -0.7937,
   -0.7492,
   -0.7071,
   -0.6674,
   -0.6300,
   -0.5946,
   -0.5612,
   -0.5297,
   -0.5000,
    0.5000,
    0.5297,
    0.5612,
    0.5946,
    0.6300,
    0.6674,
    0.7071,
    0.7492,
    0.7937,
    0.8409,
    0.8909,
    0.9439,
    1.0000,
    1.0595,
    1.1225,
    1.1892,
    1.2599,
    1.3348,
    1.4142,
    1.4983,
    1.5874,
    1.6818,
    1.7818,
    1.8877,
    2.0000
};

static const float q_table_whole_tone[26] = 
{
   -2.0000,
   -1.7818,
   -1.5874,
   -1.4142,
   -1.2599,
   -1.1225,
   -1.0000,
   -0.8909,
   -0.7937,
   -0.7071,
   -0.6300,
   -0.5612,
   -0.5000,
    0.5000,
    0.5612,
    0.6300,
    0.7071,
    0.7937,
    0.8909,
    1.0000,
    1.1225,
    1.2599,
    1.4142,
    1.5874,
    1.7818,
    2.0000
};


static const float q_table_diminished[18] = 
{
   -2.0000,
   -1.6818,
   -1.4142,
   -1.1892,
   -1.0000,
   -0.8409,
   -0.7071,
   -0.5946,
    -0.5000,
   0.5000,
    0.5946,
    0.7071,
    0.8409,
    1.0000,
    1.1892,
    1.4142,
    1.6818,
    2.0000
};

static const float q_table_augmented[14] = 
{
   -2.0000,
   -1.5874,
   -1.2599,
   -1.0000,
   -0.7937,
   -0.6300,
   -0.5000,
    0.5000,
    0.6300,
    0.7937,
    1.0000,
    1.2599,
    1.5874,
    2.0000
};

static const float q_table_fifths_octaves[10] = {
  -2,
  -1.49830707687668,
  -1,
  -0.749153538438341,
  -0.500000000000000,
  0.500000000000000,
  0.749153538438341,
  1,
  1.49830707687668,
  2
};


float get_q_table_value( int32_t q_counter, float pot_value )
{
    float temp;
    switch ( q_counter )
    {
        case 0:

            break;
        case 1:
            temp = q_table_chromatic[(uint32_t)(pot_value * 49.9999)];
            break;
        case 2:
            temp = q_table_whole_tone[(uint32_t)(pot_value * 25.9999)];
            break;
        case 3:
            temp = q_table_diminished[(uint32_t)(pot_value * 17.9999)];
            break;
        case 4:
            temp = q_table_augmented[(uint32_t)(pot_value * 13.9999)];
            break;
        case 5:
            temp = q_table_fifths_octaves[(uint32_t)(pot_value * 9.9999)];
            break;
        default:
            temp = 1.0;
            break;
    }

}


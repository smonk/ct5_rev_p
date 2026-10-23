#include "ct5_fx_biquads.h"
#include <math.h>

void biquad_config_high_pass( BiquadIIR_t * f, float cutoff_freq, float Q, float sample_rate )
{
    // 1. Calculate angular frequency and filter parameters
    float w0 = 2.0f * 3.14159265f * cutoff_freq / sample_rate;
    float alpha = sinf(w0) / (2.0f * Q);
    float cos_w0 = cosf(w0);

    // 2. Generate raw intermediate coefficients
    float a0_raw = 1.0f + alpha;
    float b0_raw = (1.0f + cos_w0) / 2.0f;
    float b1_raw = -(1.0f + cos_w0);
    float b2_raw = (1.0f + cos_w0) / 2.0f;
    float a1_raw = -2.0f * cos_w0;
    float a2_raw = 1.0f - alpha;

    // 3. Normalize all values by dividing by a0_raw
    f->b0 = b0_raw / a0_raw;
    f->b1 = b1_raw / a0_raw;
    f->b2 = b2_raw / a0_raw;
    f->a1 = a1_raw / a0_raw;
    f->a2 = a2_raw / a0_raw;
}

void biquad_config_low_pass( BiquadIIR_t * f, float cutoff_freq, float Q, float sample_rate )
{
    // 1. Calculate angular frequency and filter parameters
    float w0 = 2.0f * 3.14159265f * cutoff_freq / sample_rate;
    float alpha = sinf(w0) / (2.0f * Q);
    float cos_w0 = cosf(w0);

    // 2. Generate raw intermediate coefficients
    float a0_raw = 1.0f + alpha;
    float b0_raw = (1.0f - cos_w0) / 2.0f;
    float b1_raw = (1.0f - cos_w0);
    float b2_raw = (1.0f - cos_w0) / 2.0f;
    float a1_raw = -2.0f * cos_w0;
    float a2_raw = 1.0f - alpha;

    // 3. Normalize all values by dividing by a0_raw
    f->b0 = b0_raw / a0_raw;
    f->b1 = b1_raw / a0_raw;
    f->b2 = b2_raw / a0_raw;
    f->a1 = a1_raw / a0_raw;
    f->a2 = a2_raw / a0_raw;
}


float biquad_process_sample( BiquadIIR_t * f, float input )
{
    float output =  ( f->b0 * input ) +
                    ( f->b1 * f->x1 ) +
                    ( f->b2 * f->x2 ) -
                    ( f->a1 * f->y1 ) -
                    ( f->a2 * f->y2 );
    
    f->x2 = f->x1;
    f->x1 = input;
    f->y2 = f->y1;
    f->y1 = output;

    return output;
}

void biquad_clear_buffer( BiquadIIR_t * f )
{
    f->x1 = 0.0f;
    f->x2 = 0.0f;
    f->y1 = 0.0f;
    f->y2 = 0.0f;
}
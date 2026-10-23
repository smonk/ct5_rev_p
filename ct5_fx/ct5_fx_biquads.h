#ifndef CT5_FX_BIQUADS_H
#define CT5_FX_BIQUADS_H

typedef struct {
    float b0; float b1; float b2;
    float a1; float a2;
    float x1; float x2;
    float y1; float y2;
} BiquadIIR_t;

void biquad_config_high_pass( BiquadIIR_t * f, float cutoff_freq, float Q, float sample_rate ); 
void biquad_config_low_pass( BiquadIIR_t * f, float cutoff_freq, float Q, float sample_rate );

float biquad_process_sample( BiquadIIR_t * f, float input );
void biquad_clear_buffer( BiquadIIR_t * f );
// inline float biquad_process_sample( BiquadIIR_t * f, float input )
// {
//     float output =  ( f->b0 * input ) +
//                     ( f->b1 * f->x1 ) +
//                     ( f->b2 * f->x2 ) -
//                     ( f->a1 * f->y1 ) -
//                     ( f->a2 * f->y2 );
    
//     f->x2 = f->x1;
//     f->x1 = input;
//     f->y2 = f->y1;
//     f->y1 = output;

//     return output;
// }

#endif
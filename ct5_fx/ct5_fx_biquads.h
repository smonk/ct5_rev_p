#ifndef CT5_FX_BIQUADS_H
#define CT5_FX_BIQUADS_H

typedef struct {
    float b0; float b1; float b2;
    float a1; float a2;
    float x1; float x2;
    float y1; float y2;
} BiquadIIR_t;

void biquad_config_high_pass( BiquadIIR_t * f, float cutoff_freq, float sample_rate ); 

#endif
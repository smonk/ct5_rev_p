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

//tilt filter section
// Structure to hold filter coefficients and history states
typedef struct {
    // Coefficients
    float b0;
    float b1;
    float a1;
    
    // State variables (history registers)
    float x1;
    float y1;
} tilt_filter_t;

// Function Prototypes
void tilt_init(tilt_filter_t* filter);
void tilt_update_coefficients(tilt_filter_t* filter, float sample_rate, float pivot_freq, float gain_db);
void tilt_reset(tilt_filter_t* filter);

// Inline function for per-sample processing
static inline float tilt_process_sample(tilt_filter_t* filter, float input) {
    // Direct Form 1 implementation
    float output = (filter->b0 * input) + (filter->b1 * filter->x1) - (filter->a1 * filter->y1);
    
    // Update history
    filter->x1 = input;
    filter->y1 = output;
    
    return output;
}


#endif
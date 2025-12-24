/*
 * processing.c
 *
 * Created on: Nov 6, 2025
 * Author: [Your Name]
 *
 * Source file for Task 6: FFT Data Processing
 */

#include "processing.h"
#include "arm_math.h" // <-- This is the CMSIS-DSP Library!
#include <stdio.h>

/* --- Private Variables for FFT --- */

// Input buffer for the FFT (converted from uint16_t)
static float32_t fft_input_buffer[FFT_SIZE];

// Output buffer for the FFT (complex numbers)
static float32_t fft_output_buffer[FFT_SIZE * 2]; // *2 for complex (real/imag)

// Buffer to hold the magnitude of the FFT output
static float32_t fft_magnitude_buffer[FFT_SIZE];

// Handle for the "Real FFT" (RFFT) instance
static arm_rfft_fast_instance_f32 fft_instance;

/**
 * @brief Initializes the FFT processing module.
 */
int8_t Processing_Init(void)
{
    // Initialize the RFFT instance for a 1024-point FFT
    if (arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE) != ARM_MATH_SUCCESS)
    {
        // Initialization failed
        return -1;
    }
    return 0;
}

/**
 * @brief Runs the FFT on a buffer of ADC data.
 */
float Processing_Calculate_FFT(uint16_t* adc_buffer)
{
    /* --- 1. Prepare Data --- */
    // The ADC gives a 0-4095 signal (uint16_t) with a 1.65V DC offset.
    // The FFT needs a 0-centered signal (float).
    for (int i = 0; i < FFT_SIZE; i++)
    {
        // 1. Convert 0-4095 to 0.0-1.0
        // 2. Subtract 0.5 to make it -0.5 to +0.5
        fft_input_buffer[i] = ((float32_t)adc_buffer[i] / 4095.0f) - 0.5f;
    }

    /* --- 2. Run the FFT --- */
    // This runs the FFT in-place.
    // Input (fft_input_buffer) is overwritten by output.
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0);

    /* --- 3. Calculate Magnitudes --- */
    // The FFT output is complex (real/imag). We need the magnitude.
    // We only care about the first half (FFT_SIZE / 2)
    arm_cmplx_mag_f32(fft_output_buffer, fft_magnitude_buffer, FFT_SIZE / 2);

    /* --- 4. Find the Peak Frequency --- */
    float32_t max_magnitude = 0.0f;
    uint32_t max_index = 0;

    // Search for the peak, starting at bin 1 (bin 0 is DC offset)
    for (int i = 1; i < (FFT_SIZE / 2); i++)
    {
        if (fft_magnitude_buffer[i] > max_magnitude)
        {
            max_magnitude = fft_magnitude_buffer[i];
            max_index = i;
        }
    }

    /* --- 5. Convert Peak Index to Frequency (Hz) --- */
    // Formula: freq = (bin_index * sampling_rate) / FFT_Size
    float dominant_frequency = ((float)max_index * (float)SAMPLING_RATE) / (float)FFT_SIZE;

    return dominant_frequency;
}

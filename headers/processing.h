/*
 * processing.h
 * Header for Task 6: FFT Data Processing
 */

#ifndef INC_PROCESSING_H_
#define INC_PROCESSING_H_

#include "main.h"

// Define FFT parameters
#define FFT_SIZE         1024  // Size of FFT (must be power of 2)
#define SAMPLING_RATE    2000  // Sample rate in Hz (e.g., 2kHz)

/**
 * @brief Initializes the FFT processing module.
 * @retval 0 on success, -1 on failure
 */
int8_t Processing_Init(void);

/**
 * @brief Runs the FFT on a buffer of ADC data.
 * @param adc_buffer: Pointer to an array of ADC data.
 * @retval The dominant frequency in Hz.
 */
float Processing_Calculate_FFT(uint16_t* adc_buffer);


#endif /* INC_PROCESSING_H_ */

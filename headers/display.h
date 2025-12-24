/*
 * display.h
 *
 * Description: Header file for Task 5 Display Logic.
 * Handles LCD printing, menu state, and button inputs.
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "main.h"

/**
 * @brief Initializes the Display module and the underlying LCD.
 * @param adc_handle: The ADC_HandleTypeDef* for the buttons (e.g., &hadc2)
 * @retval None
 */
void Display_Init(ADC_HandleTypeDef* adc_handle);

/**
 * @brief Updates the LCD screen with new data.
 * @note  Call this inside a non-blocking timer (e.g., every 200ms).
 * @param speed_mps: Current speed in m/s
 * @param voltage: Current voltage from ADC3
 * @param frequency_hz: Current frequency from comparator
 * @retval None
 */
void Display_Update(float speed_mps, float voltage, float frequency_hz);


#endif /* INC_DISPLAY_H_ */

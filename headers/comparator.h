/* comparator.h - Task 4: Comparator Control and Acquisition Header */

#ifndef COMPARATOR_H
#define COMPARATOR_H

#include "main.h"
#include <stdint.h>

/* Function prototypes */

/**
  * @brief Initialize comparator edge counting
  * @retval None
  */
void Comparator_Init_EdgeCounting(void);

/**
  * @brief Update edge counter by detecting state changes
  * @param currentState Current comparator output state (0 or 1)
  * @retval None
  */
void Comparator_Update_EdgeCount(uint8_t currentState);

/**
  * @brief Calculate frequency from edge count over sampling window
  * @retval 1 if calculation complete, 0 if still sampling
  */
uint8_t Comparator_Calculate_Frequency(void);

/**
  * @brief Calculate speed from Doppler frequency shift
  * @retval Speed in m/s
  */
float Comparator_Calculate_Speed(void);

/**
  * @brief Get current measured frequency
  * @retval Frequency in Hz
  */
float Comparator_Get_Frequency(void);

/**
  * @brief Get current calculated speed
  * @retval Speed in m/s
  */
float Comparator_Get_Speed(void);

/**
  * @brief Get current edge count
  * @retval Number of edges detected
  */
uint32_t Comparator_Get_EdgeCount(void);

/**
  * @brief Get sampling progress percentage
  * @retval Progress (0-100)
  */
uint8_t Comparator_Get_SamplingProgress(void);

#endif /* COMPARATOR_H */

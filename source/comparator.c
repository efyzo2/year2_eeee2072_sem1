/* comparator.c - Task 4: Comparator Control and Acquisition */

#include "comparator.h"
#include <math.h>

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static uint32_t edgeCount = 0;              // Number of rising edges detected
static uint8_t lastCompState = 0;           // Previous comparator state
static uint32_t samplingStartTime = 0;      // Start time of current sampling window
static float measuredFrequency = 0.0f;      // Calculated frequency in Hz
static float calculatedSpeed = 0.0f;        // Calculated speed in m/s

/* ========================================================================== */
/* DOPPLER RADAR CONFIGURATION                                                */
/* ========================================================================== */

// K-band Doppler radar specifications
#define RADAR_FREQUENCY_HZ    10.525e9f     // 10.525 GHz (K-band radar)
#define SPEED_OF_LIGHT        299792458.0f  // Speed of light in m/s
#define SAMPLING_WINDOW_MS    1000          // Sampling window duration (1 second)

/* ========================================================================== */
/* PUBLIC FUNCTIONS - INITIALIZATION                                          */
/* ========================================================================== */

/**
  * @brief Initialize comparator edge counting system
  * @note Call this once at startup before entering main loop
  * @retval None
  */
void Comparator_Init_EdgeCounting(void)
{
    edgeCount = 0;
    lastCompState = 0;
    samplingStartTime = HAL_GetTick();
    measuredFrequency = 0.0f;
    calculatedSpeed = 0.0f;
}

/* ========================================================================== */
/* PUBLIC FUNCTIONS - EDGE DETECTION                                          */
/* ========================================================================== */

/**
  * @brief Update edge counter by detecting state changes
  * @note Call this function every loop iteration with current comparator state
  * @param currentState Current comparator output state (0 or 1)
  * @retval None
  */
void Comparator_Update_EdgeCount(uint8_t currentState)
{
    // Detect rising edge (LOW to HIGH transition)
    if (currentState == 1 && lastCompState == 0)
    {
        edgeCount++;
    }

    // Update state for next comparison
    lastCompState = currentState;
}

/* ========================================================================== */
/* PUBLIC FUNCTIONS - FREQUENCY CALCULATION                                   */
/* ========================================================================== */

/**
  * @brief Calculate frequency from edge count over sampling window
  * @note Automatically resets counter after calculation
  * @retval 1 if calculation complete (new frequency available), 0 if still sampling
  */
uint8_t Comparator_Calculate_Frequency(void)
{
    uint32_t currentTime = HAL_GetTick();
    uint32_t elapsedTime = currentTime - samplingStartTime;

    // Check if sampling window is complete
    if (elapsedTime >= SAMPLING_WINDOW_MS)
    {
        // Calculate frequency in Hz
        // Formula: frequency = (number of rising edges) / (time in seconds)
        measuredFrequency = (float)edgeCount / ((float)elapsedTime / 1000.0f);

        // Reset counter for next measurement window
        edgeCount = 0;
        samplingStartTime = currentTime;

        return 1;  // Calculation complete - new data available
    }

    return 0;  // Still sampling - no new data yet
}

/* ========================================================================== */
/* PUBLIC FUNCTIONS - SPEED CALCULATION                                       */
/* ========================================================================== */

/**
  * @brief Calculate speed from Doppler frequency shift
  * @note Uses the Doppler equation: f_doppler = 2 * v * f_radar / c
  *       Solving for velocity: v = (f_doppler * c) / (2 * f_radar)
  * @retval Speed in m/s
  */
float Comparator_Calculate_Speed(void)
{
    // Apply Doppler equation
    // f_doppler = 2 * v * f_radar / c
    // Rearranging: v = (f_doppler * c) / (2 * f_radar)

    calculatedSpeed = (measuredFrequency * SPEED_OF_LIGHT) / (2.0f * RADAR_FREQUENCY_HZ);

    return calculatedSpeed;
}

/* ========================================================================== */
/* PUBLIC FUNCTIONS - DATA ACCESS                                             */
/* ========================================================================== */

/**
  * @brief Get current measured frequency
  * @retval Frequency in Hz
  */
float Comparator_Get_Frequency(void)
{
    return measuredFrequency;
}

/**
  * @brief Get current calculated speed
  * @retval Speed in m/s
  */
float Comparator_Get_Speed(void)
{
    return calculatedSpeed;
}

/**
  * @brief Get current edge count (for debugging)
  * @retval Number of edges detected in current sampling window
  */
uint32_t Comparator_Get_EdgeCount(void)
{
    return edgeCount;
}

/**
  * @brief Get sampling progress as percentage
  * @retval Progress from 0 to 100 (%)
  */
uint8_t Comparator_Get_SamplingProgress(void)
{
    uint32_t currentTime = HAL_GetTick();
    uint32_t elapsedTime = currentTime - samplingStartTime;

    uint8_t progress = (uint8_t)((elapsedTime * 100) / SAMPLING_WINDOW_MS);

    // Clamp to 100% maximum
    if (progress > 100)
        progress = 100;

    return progress;
}

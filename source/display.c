/*
 * display.c
 *
 * Description: Source file for Task 5 Display Logic.
 * Handles LCD printing, menu state, and button inputs.
 */

#include "display.h"
#include "lcd16x2.h" // <-- Include your *original* LCD driver
#include <stdio.h>
#include <string.h>

// --- Private Defines (Copied from your main.c) ---
// Calibrated 12-bit ADC thresholds
#define BTN_RIGHT_THRESHOLD     400
#define BTN_UP_THRESHOLD        1300
#define BTN_DOWN_THRESHOLD      2300
#define BTN_LEFT_THRESHOLD      3400

// Enum for our button types
typedef enum {
    BTN_NONE,
    BTN_RIGHT,
    BTN_UP,
    BTN_DOWN,
    BTN_LEFT
} ButtonType;

// Enum for our speed units (Task 5)
typedef enum {
    UNIT_MS,
    UNIT_KMH,
    UNIT_MPH
} SpeedUnit;

// --- Private (Static) Variables ---
static ADC_HandleTypeDef* button_adc_handle; // Handle for ADC2
static SpeedUnit currentUnit = UNIT_MS;
static uint8_t buttonWasPressed = 0; // For button debounce

// --- Private Function Prototypes ---
static ButtonType Read_Keypad_Button(void);

/**
 * @brief Initializes the Display module and the underlying LCD.
 */
void Display_Init(ADC_HandleTypeDef* adc_handle)
{
    button_adc_handle = adc_handle; // Store the ADC handle

    // Initialize LCD (Copied from your main.c)
    HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_SET);
    lcd16x2_init_4bits(RS_GPIO_Port, RS_Pin, E_GPIO_Port, E_Pin,
                       D4_GPIO_Port, D4_Pin, D5_GPIO_Port, D5_Pin,
                       D6_GPIO_Port, D6_Pin, D7_GPIO_Port, D7_Pin);
}

/**
 * @brief Updates the LCD screen with new data.
 */
void Display_Update(float speed_mps, float voltage, float frequency_hz)
{
    /* --- 1. Read Buttons and Update State --- */
    ButtonType pressedButton = Read_Keypad_Button();

    // Debounce logic
    if (pressedButton != BTN_NONE && buttonWasPressed == 0)
    {
        buttonWasPressed = 1; // Mark button as "being held"

        // Cycle units
        if (pressedButton == BTN_RIGHT)
        {
            if (currentUnit == UNIT_MS)     currentUnit = UNIT_KMH;
            else if (currentUnit == UNIT_KMH) currentUnit = UNIT_MPH;
            else if (currentUnit == UNIT_MPH) currentUnit = UNIT_MS;
        }
        else if (pressedButton == BTN_LEFT)
        {
            if (currentUnit == UNIT_MS)     currentUnit = UNIT_MPH;
            else if (currentUnit == UNIT_MPH) currentUnit = UNIT_KMH;
            else if (currentUnit == UNIT_KMH) currentUnit = UNIT_MS;
        }
    }
    else if (pressedButton == BTN_NONE)
    {
        buttonWasPressed = 0; // Mark button as "released"
    }

    /* --- 2. Prepare Data for Display --- */
    float displaySpeed = speed_mps;
    char unitString[5]; // Buffer for unit string

    if (currentUnit == UNIT_KMH)
    {
        displaySpeed = speed_mps * 3.6f;
        strcpy(unitString, "km/h");
    }
    else if (currentUnit == UNIT_MPH)
    {
        displaySpeed = speed_mps * 2.23694f;
        strcpy(unitString, "mph");
    }
    else // Default to m/s
    {
        displaySpeed = speed_mps;
        strcpy(unitString, "m/s");
    }

    /* --- 3. Print to LCD --- */
    lcd16x2_clear();

    // Line 1: Voltage and Frequency
    lcd16x2_1stLine();
    lcd16x2_printf("V:%.2fV F:%.0fHz", voltage, frequency_hz);

    // Line 2: Speed with new units
    lcd16x2_2ndLine();
    lcd16x2_printf("Speed: %.2f %s", displaySpeed, unitString);
}


/**
 * @brief  Reads the analog button shield (Copied from your main.c)
 */
static ButtonType Read_Keypad_Button(void)
{
    uint32_t adcVal = 0;

    // Perform ADC conversion on ADC2
    HAL_ADC_Start(button_adc_handle);
    HAL_ADC_PollForConversion(button_adc_handle, HAL_MAX_DELAY);
    adcVal = HAL_ADC_GetValue(button_adc_handle);
    HAL_ADC_Stop(button_adc_handle);

    // Compare raw 12-bit value to calibrated thresholds
    if (adcVal < BTN_RIGHT_THRESHOLD)     return BTN_RIGHT;
    if (adcVal < BTN_UP_THRESHOLD)        return BTN_UP;
    if (adcVal < BTN_DOWN_THRESHOLD)      return BTN_DOWN;
    if (adcVal < BTN_LEFT_THRESHOLD)      return BTN_LEFT;

    return BTN_NONE;
}

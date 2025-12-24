/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "comp.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "comparator.h"
#include "display.h"
#include "processing.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMP_OUT_Pin       GPIO_PIN_5
#define COMP_OUT_GPIO_Port GPIOC

// Constants for Doppler speed calculation
#define RADAR_FREQUENCY_HZ    10.525e9f
#define SPEED_OF_LIGHT        299792458.0f
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// --- Variables for Task 7 (Comparator) ---
float speed_mps = 0.0f;
float frequency_hz = 0.0f;

// --- Variables for Task 8 (FFT) ---
float fft_frequency_hz = 0.0f;
float fft_speed_mps = 0.0f;
float voltage = 0.0f;

// This buffer will be filled by the DMA in the background
uint16_t adc_buffer[FFT_SIZE];
// This buffer is for the FFT to use
uint16_t processing_buffer[FFT_SIZE];
// Volatile flags to signal when data is ready
volatile uint8_t data_full_ready_flag = 0;

volatile uint32_t fft_run_count = 0;
volatile uint32_t dma_error_count = 0;

uint32_t last_uart_tick = 0;
uint32_t last_display_tick = 0;
const uint32_t uart_interval_ms = 50;      // 50ms for UART = 20Hz update rate
const uint32_t display_interval_ms = 100;   // 100ms for LCD updates

// Enhanced voltage smoothing with dual-stage filtering
#define VOLTAGE_BUFFER_SIZE 16
float voltage_buffer[VOLTAGE_BUFFER_SIZE] = {0};
uint8_t voltage_buffer_index = 0;
uint8_t voltage_buffer_filled = 0;

// Track which buffer position we're displaying from
uint32_t display_read_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_COMP1_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

  HAL_COMP_Start(&hcomp1);
  Display_Init(&hadc2);
  Comparator_Init_EdgeCounting();

  /* --- Start Task 8 (FFT) --- */
  Processing_Init(); // Initialize the FFT

  // Start the ADC with DMA in interrupt mode
  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_buffer, FFT_SIZE) != HAL_OK) {
      Error_Handler();
  }

  // Start the Timer that triggers the ADC
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* --- Task 7: Comparator Logic (MUST RUN CONTINUOUSLY) --- */
      uint8_t compState = HAL_COMP_GetOutputLevel(&hcomp1);
      Comparator_Update_EdgeCount(compState);

      if (Comparator_Calculate_Frequency() == 1)
      {
          speed_mps = Comparator_Calculate_Speed();
          frequency_hz = Comparator_Get_Frequency();

          // Clamp comparator frequency
          if (frequency_hz > 1000.0f) {
              frequency_hz = 1000.0f;
          }
      }

      // Check if the DMA has filled the buffer
      if (data_full_ready_flag)
      {
          data_full_ready_flag = 0; // Clear the flag
          fft_run_count++;

          // Copy the entire buffer for FFT processing
          memcpy(processing_buffer, adc_buffer, FFT_SIZE * sizeof(uint16_t));

          // Run the FFT on the entire buffer
          fft_frequency_hz = Processing_Calculate_FFT(processing_buffer);

          // Clamp frequency to reasonable range
          if (fft_frequency_hz > 1000.0f) {
              fft_frequency_hz = 1000.0f;
          }

          fft_speed_mps = (fft_frequency_hz * SPEED_OF_LIGHT) / (2.0f * RADAR_FREQUENCY_HZ);

          // Pre-fill voltage buffer with smoothed samples from the processing buffer
          // This creates a stable "snapshot" we can read from later
          voltage_buffer_index = 0;
          for (uint32_t i = 0; i < VOLTAGE_BUFFER_SIZE && i < FFT_SIZE; i++)
          {
              // Sample at regular intervals through the buffer
              uint32_t sample_idx = (i * FFT_SIZE) / VOLTAGE_BUFFER_SIZE;
              voltage_buffer[i] = 3.3f * processing_buffer[sample_idx] / 4095.0f;
          }
          voltage_buffer_filled = 1;
          display_read_index = 0;  // Reset read position
      }

      /* --- Timed UART Update --- */
      uint32_t current_tick = HAL_GetTick();

      if (current_tick - last_uart_tick >= uart_interval_ms)
      {
          last_uart_tick = current_tick;

          // Read from the pre-filled voltage buffer if available
          if (voltage_buffer_filled)
          {
              // Cycle through the voltage buffer samples
              voltage = voltage_buffer[display_read_index];
              display_read_index = (display_read_index + 1) % VOLTAGE_BUFFER_SIZE;
          }
          else
          {
              // Fallback: read directly from ADC buffer (with safety margin)
              uint32_t dma_remaining = __HAL_DMA_GET_COUNTER(hadc3.DMA_Handle);
              uint32_t write_pos = (FFT_SIZE - dma_remaining) % FFT_SIZE;
              uint32_t read_pos = (write_pos + FFT_SIZE - 50) % FFT_SIZE;
              voltage = 3.3f * adc_buffer[read_pos] / 4095.0f;
          }

          /* --- UART Transmit for Plotter (4 values) --- */
          char msg[80];
          sprintf(msg, "%.3f,%lu,%.1f,%.1f\n",
                        voltage,                          // Value 1: Voltage
                        Comparator_Get_EdgeCount(),       // Value 2: Comparator Edge Count
                        frequency_hz,                     // Value 3: Comparator Freq
                        fft_frequency_hz);                // Value 4: FFT Freq
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      }

      /* --- Timed Display Update --- */
      if (current_tick - last_display_tick >= display_interval_ms)
      {
          last_display_tick = current_tick;
          Display_Update(fft_speed_mps, voltage, fft_frequency_hz);
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  DMA transfer error callback.
  */
void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == hadc3.DMA_Handle)
    {
        dma_error_count++;

        // Hard reset the entire chain
        HAL_TIM_Base_Stop(&htim2);
        HAL_ADC_Stop_DMA(&hadc3);

        // Re-calibrate just in case
        HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

        // Restart the chain
        if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_buffer, FFT_SIZE) != HAL_OK) {
            Error_Handler();
        }
        if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
            Error_Handler();
        }
    }
}

/**
  * @brief  Conversion DMA transfer complete callback.
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC3) {
        data_full_ready_flag = 1; // Signal the main loop
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

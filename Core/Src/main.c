/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <usbd_cdc_if.h>

#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SAMPLES 250

#define CPU_FREQUENCY 72000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile char rx_buffer[RX_BUF_LEN];
extern volatile uint8_t command_ready;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

enum time_base {TENTH_OF_MICROSECOND, MICROSECOND, TENTH_MILLISECOND};
enum signal_frequency {ONE_MHZ, ONE_HUNDRED_KHZ, TEN_KHZ, ONE_KHZ, ONE_HUNDRED_HZ, SIXTY_HZ};
enum signal_samples {TWENTY, TEN, FIVE, TWO};

volatile uint32_t DMA_ADC_buffer[MAX_SAMPLES] = {[0 ... MAX_SAMPLES-1] = 0x00000000};
volatile uint32_t DMA_ADC_timming_buffer[MAX_SAMPLES] = {[0 ... MAX_SAMPLES-1] = 0x00000000};

volatile uint32_t sample_count = 0;
volatile uint8_t adc_done = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// int __io_putchar(int ch);

int _write(int file, char *ptr, int len);

HAL_StatusTypeDef ADC_base_TIM3_config(TIM_HandleTypeDef *htim3,
                                       enum signal_frequency src_frequency,
                                       enum signal_samples samples_per_period);

HAL_StatusTypeDef time_base_config(TIM_HandleTypeDef *htim4, enum time_base time_base);

HAL_StatusTypeDef ADC_Dual_config(ADC_HandleTypeDef *adc1, ADC_HandleTypeDef *adc2);

HAL_StatusTypeDef Start_Dual_ADC_DMA(enum signal_frequency src_frequency,
  uint16_t samples_per_period,
  uint32_t number_of_samples,
  uint32_t* DMA_ADC_buffer);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function may conflict with syscalls.c, erase them in this case.

// int __io_putchar(int ch) {
//   while (ITM->PORT[0U].u32 == 0UL) {
//     __NOP();
//   }
//   ITM->PORT[0U].u8 = (uint8_t)ch;
//   return ch;
// }

// Function may conflict with syscalls.c, erase them in this case.
// int _write(int file, char *ptr, int len) {
//   for (int i = 0; i < len; i++) {
//     __io_putchar(*ptr);
//     ptr++;
//   }
//   return len;
// }

//@TODO checar retorno das funções
int _write(int file, char *ptr, int len) {
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    CDC_Transmit_FS((uint8_t*)ptr, len);
  }
  return len;
}

/**
 * Configures TIM3 to trigger samples_per_period*src_frequency times per second,
 * where src_frequency is based on signal_frequency enum and and samples_per_period
 * in signal_samples enum
 *
 * @param htim3 Handler for TIM3
 * @param src_frequency Frequency from the base signal for exitation
 * @param samples_per_period Number of samples per period of the source frequency
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef ADC_base_TIM3_config(TIM_HandleTypeDef *htim3,
                                       enum signal_frequency src_frequency,
                                       enum signal_samples samples_per_period)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  uint32_t timer_frequency = 0;
  switch (src_frequency) {
    case ONE_KHZ:
      timer_frequency = 1000;
    break;
    case ONE_HUNDRED_HZ:
      timer_frequency = 100;
    break;
    case SIXTY_HZ:
      timer_frequency = 60;
    break;
    default:
      return HAL_ERROR;
  }

  switch (samples_per_period) {
    case TWENTY:
      timer_frequency = timer_frequency * 20;
    break;
    case TEN:
      timer_frequency = timer_frequency * 10;
    break;
    case FIVE:
      timer_frequency = timer_frequency * 5;
    break;
    case TWO:
      timer_frequency = timer_frequency * 2;
    default:
      return HAL_ERROR;
  }

  uint32_t prescaler = 0;
  uint32_t period = 0;

  // (CPU_FREQUENCY/((1+prescaler)*(1+period)) == timer_frequency)
  if (CPU_FREQUENCY/timer_frequency > 0xFFFF) {
    period = 0xFFFF;
    if (CPU_FREQUENCY/((1+prescaler)*timer_frequency) - 1 > 0xFFFF) {
      printf("Error: Sampling frequency %lu beyond period limit for TIM3\r\n", timer_frequency);
      return HAL_ERROR;
    }
    prescaler = CPU_FREQUENCY/((1+prescaler)*timer_frequency) - 1;
  }
  else {
    prescaler = 0;
    period = CPU_FREQUENCY/timer_frequency - 1;
  }

  htim3->Instance = TIM3;
  htim3->Init.Prescaler = prescaler;
  htim3->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3->Init.Period = period;
  htim3->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(htim3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(htim3, &sClockSourceConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(htim3, &sMasterConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * Configures TIM4 for time base, each count is a time_base step, up to 0xFFFF where
 * the timer overflows and starts counting from 0.
 *
 * Counter must be reset with htim4.Instance->CNT = 0 to ensure the value
 * has a known reference.
 *
 * Read is done the same way, accessing htim4.Instance->CNT
 *
 * @param htim4 handler address for timer 4
 * @param time_base enum with clock step size
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef time_base_config(TIM_HandleTypeDef *htim4, enum time_base time_base) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  uint32_t timer_frequency = 0;
  switch (time_base) {
    case TENTH_OF_MICROSECOND:
      timer_frequency = 10000000;
    break;
    case MICROSECOND:
      timer_frequency = 1000000;
    break;
    case TENTH_MILLISECOND:
      timer_frequency = 10000;
    break;
    default:
      return HAL_ERROR;
  }

  uint32_t prescaler = 0;
  uint32_t period = 0;

  // (CPU_FREQUENCY/((1+prescaler)*(1+period)) == timer_frequency)
  if (CPU_FREQUENCY/timer_frequency > 0xFFFF) {
      printf("Error: Time base frequency %lu beyond prescaler limit for TIM4\r\n", timer_frequency);
      return HAL_ERROR;
  }
  // Count goes up un time base clock
  prescaler = CPU_FREQUENCY/timer_frequency - 1;
  period = 0xFFFF;

  htim4->Instance = TIM4;
  htim4->Init.Prescaler = prescaler;
  htim4->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4->Init.Period = period;
  htim4->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  return HAL_OK;
}

HAL_StatusTypeDef ADC_Dual_config(
  ADC_HandleTypeDef *hadc1,
  ADC_HandleTypeDef *hadc2)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1->Instance = ADC1;
  hadc1->Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1->Init.ContinuousConvMode = DISABLE;
  hadc1->Init.DiscontinuousConvMode = DISABLE;
  hadc1->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1->Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* ADC2 init function */
  /** Common config
  */
  hadc2->Instance = ADC2;
  hadc2->Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2->Init.ContinuousConvMode = DISABLE;
  hadc2->Init.DiscontinuousConvMode = DISABLE;
  hadc2->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2->Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(hadc2) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }

  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode);
  return HAL_OK;
}

/**
 * Configures and starts ADC in dual mode with TIM3 trigger, DMA to automatically save all acquisitions
 * in the DMA_ADC_buffer[] and TIM4 as time base for each acquisition.
 *
 * To use TIM4 as time base, reading the time of the ADC acquisition, read TIM4 on each ADC sample trigger
 * of TIM3 overflow by defining HAL_TIM_PeriodElapsedCallback() callback function. Use the callback to
 * accumulate sample_count, and use sample_count as index for DMA_ADC_timming_buffer[], which is to be
 * updated with TIM4 count.
 *
 * HAL_ADC_ConvCpltCallback() MUST be set to stop the timers TIM3 with interruptions and TIM4, stop
 * the DMA multimode conversions and also stop the ADC. It's on the conversion complete callback that
 * the flag adc_done must be set to '1', which is always cleared by this function.
 *
 * Error messages are sent with printf(). Change _write() to reflect the desired output.
 *
 * @param src_frequency signal_frequency enum to define frequency of the sampled signal
 * @param samples_per_period signal_samples enum to define total samples per period
 * @param number_of_samples total count of samples acquired on dual mode, must be less than MAX_SAMPLES¸
 * @param DMA_ADC_buffer buffer of size MAX_SIZE of type uint32_t for acquired values.
 * ADC1 is on lower 16 bits and ADC2 on higher 16 bits.
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef Start_Dual_ADC_DMA(
  const enum signal_frequency src_frequency,
  const uint16_t samples_per_period,
  const uint32_t number_of_samples,
  uint32_t* DMA_ADC_buffer) {
  if (number_of_samples > MAX_SAMPLES) {
    printf ("Error: Number of samples is greater than MAX_SAMPLES\r\n");
    return HAL_ERROR;
  }

  if (ADC_base_TIM3_config(&htim3, src_frequency, samples_per_period) != HAL_OK ||
      time_base_config(&htim4, MICROSECOND) != HAL_OK) {
    printf("Error: Failed to configure timers\r\n");
    return HAL_ERROR;
      }

  if (ADC_Dual_config(&hadc1, &hadc2) != HAL_OK) {
    printf("Error: Failed to configure ADC dual mode\r\n");
    return HAL_ERROR;
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK || HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) {
    printf("Error: Failed to calibrate ADC\r\n");
    return HAL_ERROR;
  }

  if (HAL_ADC_Start(&hadc2) != HAL_OK) {
    printf("Error: Failed to start ADC2\r\n");
    return HAL_ERROR;
  }

  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, DMA_ADC_buffer, number_of_samples*2) != HAL_OK) {
    printf("Error: Failed to start ADC1\r\n");
    return HAL_ERROR;
  }

  // Clean state flags
  adc_done = 0;
  sample_count = 0;

  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  if (HAL_TIM_Base_Start(&htim4) != HAL_OK || HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    printf("Error: Failed to start timers\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_TIM_Base_Stop(&htim4);
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    HAL_ADC_Stop(&hadc2);
    adc_done = 1;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
      DMA_ADC_timming_buffer[sample_count] = htim4.Instance->CNT;
      sample_count++;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  printf("[%lu ms] Error: ", HAL_GetTick());
  if (hadc->Instance == ADC1) {
    printf("ADC1 ");
    }
  else if (hadc->Instance == ADC2) {
      printf("ADC2 ");
  }
  printf("Code: 0x%lX\r\n", hadc->ErrorCode);
  if (hadc->ErrorCode & HAL_ADC_ERROR_OVR) {
      printf(" - Overrun error (new data before old was read)\r\n");
  }

  if (hadc->ErrorCode & HAL_ADC_ERROR_DMA) {
      printf(" - DMA transfer error\r\n");
  }

  if (hadc->ErrorCode & HAL_ADC_ERROR_INTERNAL) {
      printf(" - Internal ADC HAL error\r\n");
  }

  if (hadc->ErrorCode == HAL_ADC_ERROR_NONE) {
      printf(" - No error\r\n");
  }
}

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  printf("\r"); // start serial
  printf("Start Program:\r\n");

  Start_Dual_ADC_DMA(ONE_KHZ, TEN, MAX_SAMPLES, DMA_ADC_buffer);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (adc_done)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);

      printf("Sample total: %u\r\n", sample_count + 1);

      for (int i = 0; i < MAX_SAMPLES; i++) {
        uint16_t val_adc1 = DMA_ADC_buffer[i] & 0xFFFF;
        uint16_t val_adc2 = (DMA_ADC_buffer[i] >> 16) & 0xFFFF;

        printf("\r\n[%04lu us] reading:\r\n", DMA_ADC_timming_buffer[i]);
        printf("ADC1[%03d] \t%04X\r\n", i, val_adc1);
        printf("ADC2[%03d] \t%04X\r\n", i, val_adc2);
      }


    }
    if (command_ready) {
      if (strcmp(rx_buffer, "1") == 0) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
      else if (strcmp(rx_buffer, "2") == 0) {
        Start_Dual_ADC_DMA(ONE_KHZ,
          TEN,
          MAX_SAMPLES,
          DMA_ADC_buffer);
      }
      char menu[] =
        "\r\n=== Main Menu ===\r\n"
        "1. Blink LED\r\n"
        "2. Read ADC\r\n"
        "3. Reset MCU\r\n"
        "Enter choice: ";
      CDC_Transmit_FS((uint8_t*)menu, strlen(menu));
      command_ready = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  printf("[%lu ms] Error! \r\n", HAL_GetTick());
  while (1)
  {
    __NOP();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

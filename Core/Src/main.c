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
#include <math.h>
#include <usbd_cdc_if.h>

#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SAMPLES 256
// #define SWO_DEBUG
#define CPU_FREQUENCY 72000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t Rx[APP_RX_DATA_SIZE];
extern volatile uint8_t command_ready;
extern volatile uint8_t cmd_index;

// typedef enum {
//   TENTH_OF_MICROSECOND,
//   MICROSECOND,
//   TENTH_MILLISECOND
// } time_base_t;

typedef enum {
  FIVE_MHZ=5000000,
  ONE_MHZ=1000000,
  ONE_HUNDRED_KHZ=100000,
  TEN_KHZ=10000,
  ONE_KHZ=1000,
  ONE_HUNDRED_HZ=100,
  SIXTY_HZ=60
} signal_frequency_t;

typedef enum {TWENTY_SAMPLES=20,
  TEN_SAMPLES=10,
  FIVE_SAMPLES=5,
  TWO_SAMPLES=2
} signal_period_samples_t ;

typedef enum {
  TWO_HUNDRED_OHMS=200,
  ONE_K_OHMS=1000,
  TEN_K_OHMS=10000,
  ONE_HUNDRED_K_OHMS=100000
} resistance_values_t;

typedef enum {
  INDUCTOR_LR,
  CAPACITOR_RC,
  RESISTOR_LR
} element_configuration_t;

typedef struct z_t {
  long double real;
  long double imaginary;
} z_t;

uint32_t samples_per_period_conf = TWENTY_SAMPLES;
uint32_t source_frequency_conf = ONE_KHZ;
double sampling_frequency_conf = 0.; // defined on TIM3 config
uint32_t resistor_conf = TEN_K_OHMS;
element_configuration_t element_conf = CAPACITOR_RC;

uint32_t DMA_ADC_buffer[MAX_SAMPLES] = {0};
//float ADC_timming_buffer_us[MAX_SAMPLES] = {0};

int8_t zero_crossings_1[MAX_SAMPLES] = {0};
int8_t zero_crossings_2[MAX_SAMPLES] = {0};
//float phase_shift[MAX_SAMPLES] = {0};

volatile uint32_t sample_count = 0;
volatile uint8_t adc_done = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// int __io_putchar(int ch);

int _write(int file, char *ptr, int len);

HAL_StatusTypeDef ADC_base_TIM3_config(TIM_HandleTypeDef *htim3,
                                       signal_frequency_t src_frequency,
                                       signal_period_samples_t samples_per_period);

// HAL_StatusTypeDef time_base_config(TIM_HandleTypeDef *htim4, time_base_t time_base);

HAL_StatusTypeDef ADC_Dual_config(ADC_HandleTypeDef *adc1, ADC_HandleTypeDef *adc2);

HAL_StatusTypeDef Start_Dual_ADC_DMA(signal_frequency_t src_frequency,
  signal_period_samples_t samples_per_period,
  uint32_t number_of_samples,
  uint32_t* DMA_ADC_buffer);

double get_linear_root(double y_0, double y_f, double time_delta);

double get_phase_shift(double signal1[MAX_SAMPLES], double signal2[MAX_SAMPLES],
 signal_frequency_t frequency, signal_period_samples_t samples_per_period);

double get_peak_ac(double signal1[MAX_SAMPLES], signal_period_samples_t samples_per_period);

z_t get_z(double V1, double V2, double V2_phase_degrees,
  resistance_values_t series_resistance, element_configuration_t config);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef SWO_DEBUG
// Function may conflict with syscalls.c, erase them in this case.
 int __io_putchar(int ch) {
   while (ITM->PORT[0U].u32 == 0UL) {
     __NOP();
   }
   ITM->PORT[0U].u8 = (uint8_t)ch;
   return ch;
 }

// Function may conflict with syscalls.c, erase them in this case.
 int _write(int file, char *ptr, int len) {
   for (int i = 0; i < len; i++) {
     __io_putchar(*ptr);
     ptr++;
   }
   return len;
 }
#endif
#ifndef SWO_DEBUG
int _write(int file, char *ptr, int len) {
	(void)file;
    while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY) {
    	__NOP();
    }
    return len;
}
#endif

/**
 * Configures TIM3 to trigger samples_per_period*src_frequency times per second,
 * where src_frequency is based on signal_frequency enum and samples_per_period
 * in signal_samples enum
 *
 * @param htim3 Handler for TIM3
 * @param src_frequency Frequency from the base signal for excitation
 * @param samples_per_period Number of samples per period of the source frequency
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef ADC_base_TIM3_config(TIM_HandleTypeDef *htim3,
                                       signal_frequency_t src_frequency,
                                       signal_period_samples_t samples_per_period) {
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};

   uint32_t sample_frequency = src_frequency*samples_per_period;

   uint32_t prescaler = 0;
   uint32_t period = 0;

   // (CPU_FREQUENCY/((1+prescaler)*(1+period)) == timer_frequency)
   // Adjust sampling frequency if too high
   if (src_frequency == 5000000 ) {
     // Captura uma amostra a cada 8 períodos do sinal, de modo que mesmo com sinal de 5 MHz,
     // o sampling rate é de ~ 620.7 kHz
     prescaler = 0;
     period = 116 - 1; // virtual 18 samples per period, one sample each 8 periods
     samples_per_period_conf = 18;
   }
   else if (src_frequency == 1000000) {
     prescaler = 0;
     period = 147 - 1; // virtual 24 Msps or 24 samples
     samples_per_period_conf = 24;
   }
   else if (src_frequency == 100000) {
     prescaler = 0;
     period = 756 - 1; // virtal 2 Msps
     samples_per_period_conf = 20;
   }else {
    if (CPU_FREQUENCY/sample_frequency > 0xFFFF) {
      period = 0xFFFF;
      if (CPU_FREQUENCY/((1+prescaler)*sample_frequency) - 1 > 0xFFFF) {
        printf("ERROR: Sampling frequency %lu beyond period limit for TIM3\r\n", sample_frequency);
        return HAL_ERROR;
      }
      prescaler = CPU_FREQUENCY/((1+prescaler)*sample_frequency) - 1;
    }
    else {
      prescaler = 0;
      period = CPU_FREQUENCY/sample_frequency - 1;
    }
  }
//
  sampling_frequency_conf = samples_per_period_conf*src_frequency;
  printf("WARNING: Sample rate override:\r\n"
		"-> Samples per period: %lu\r\n"
		"-> Sampling frequency: %.0f MHz\r\n",
   samples_per_period_conf, sampling_frequency_conf/1000000.0);

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

// /**
//  * Configures TIM4 for time base, each count is a time_base step, up to 0xFFFF where
//  * the timer overflows and starts counting from 0.
//  *
//  * Counter must be reset with htim4.Instance->CNT = 0 to ensure the value
//  * has a known reference.
//  *
//  * Read is done the same way, accessing htim4.Instance->CNT
//  *
//  * @param htim4 handler address for timer 4
//  * @param time_base enum with clock step size
//  * @return HAL_OK on success, HAL_ERROR on failure
//  */
// HAL_StatusTypeDef time_base_config(TIM_HandleTypeDef *htim4, time_base_t time_base) {
//   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//   TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//   uint32_t timer_frequency = 0;
//   switch (time_base) {
//     case TENTH_OF_MICROSECOND:
//       timer_frequency = 10000000;
//     break;
//     case MICROSECOND:
//       timer_frequency = 1000000;
//     break;
//     case TENTH_MILLISECOND:
//       timer_frequency = 10000;
//     break;
//     default:
//       return HAL_ERROR;
//   }
//
//   uint32_t prescaler = 0;
//   uint32_t period = 0;
//
//   // (CPU_FREQUENCY/((1+prescaler)*(1+period)) == timer_frequency)
//   if (CPU_FREQUENCY/timer_frequency > 0xFFFF) {
//       printf("ERROR: Time base frequency %lu beyond prescaler limit for TIM4\r\n", timer_frequency);
//       return HAL_ERROR;
//   }
//   // Count goes up un time base clock
//   prescaler = CPU_FREQUENCY/timer_frequency - 1;
//   period = 0xFFFF;
//
//   htim4->Instance = TIM4;
//   htim4->Init.Prescaler = prescaler;
//   htim4->Init.CounterMode = TIM_COUNTERMODE_UP;
//   htim4->Init.Period = period;
//   htim4->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   htim4->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//
//   if (HAL_TIM_Base_Init(htim4) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//   if (HAL_TIM_ConfigClockSource(htim4, &sClockSourceConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(htim4, &sMasterConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   return HAL_OK;
// }

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
  const signal_frequency_t src_frequency,
  const signal_period_samples_t samples_per_period,
  const uint32_t number_of_samples,
  uint32_t* DMA_ADC_buffer) {
  if (number_of_samples > MAX_SAMPLES) {
    printf ("ERROR: Number of samples is greater than MAX_SAMPLES\r\n");
    return HAL_ERROR;
  }
  // Clean state flags
  adc_done = 0;
  sample_count = 0;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  if (ADC_base_TIM3_config(&htim3, src_frequency, samples_per_period) != HAL_OK) {
    printf("ERROR: Failed to configure timers\r\n");
    return HAL_ERROR;
  }

  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, DMA_ADC_buffer, number_of_samples) != HAL_OK) {
    printf("ERROR: Failed to start ADC1\r\n");
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COUNTER(&htim3, 0);
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    printf("ERROR: Failed to start timer3\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    // HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim3);
    adc_done = 1;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
      sample_count++;
    }
}


/**
 * Applies to signal a band-pass filter spanning two octaves, from half src_frequency to twice the src_frequency.
 *
 * @param order Filter order, i.e. number of times each 1st order filter is applied
 * @param signal Signal to be filtered
 * @param src_frequency Frequency of the signal
 * @param samples_per_period Number of samples of the signal per period
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef band_pass_filter(const uint8_t order,
                                   double signal[MAX_SAMPLES],
                                   signal_frequency_t src_frequency,
                                   signal_period_samples_t samples_per_period) {
  double signal_frequency = src_frequency;
  double sample_frequency = src_frequency*samples_per_period;

  double low_cutoff_frequency = signal_frequency/2;
  double high_cutoff_frequency = signal_frequency*2;
  double alpha = exp(- 2*M_PI*high_cutoff_frequency/sample_frequency);
  double beta = 1/(1+(2*M_PI*low_cutoff_frequency/sample_frequency));

  // Low pass filter
  for (int j = 0; j < order; j++) {
    for (size_t i = 1; i < MAX_SAMPLES; i++) {
      signal[i] = (1 - alpha)*signal[i] + alpha*signal[i-1];
    }
  }

  // high pass filter
  double high_pass_buffer[MAX_SAMPLES];
  high_pass_buffer[0] = 0;
  for (int j = 0; j < order; j++) {
    for (size_t i = 1; i < MAX_SAMPLES; i++) {
      high_pass_buffer[i] = beta*(high_pass_buffer[i-1]+signal[i]-signal[i-1]);
    }
    for (size_t i = 0; i < MAX_SAMPLES; i++) {
      signal[i] = high_pass_buffer[i];
    }
  }
  return HAL_OK;
}

/**
 * Calculate time for linear signal to cross 0
 * @param y_0 signal at t = 0
 * @param y_f signal at t = time_delta
 * @param time_delta time between y_0 and y_f
 * @return time difference between y = y_0 and y = 0
 */
inline double get_linear_root(const double y_0, const double y_f, const double time_delta) {
  return time_delta*(-y_0)/(y_f-y_0);
}

/**
 * Calcula a diferença de fase entre dois sinais amostrados.
 * @param signal1 Amplitude do sinal de referência em função da contagem de amostras
 * @param signal2 Amplitude do sinal com variação de fase em função da contagem de amostras, sincronizado com signal1
 * @param frequency Frequência dos sinais
 * @param samples_per_period Quantidade de amostras por período do sinal
 * @return Fase média entre signal1 e signal2, entre -179 e 179 graus
 */
double get_phase_shift(double signal1[MAX_SAMPLES],
                       double signal2[MAX_SAMPLES],
                       const signal_frequency_t frequency,
                       const signal_period_samples_t samples_per_period) {

  if (MAX_SAMPLES < 2*samples_per_period) {
    printf("ERROR: Less than 2 periods of sampling. Unable to filter properly.\r\n");
    return NAN;
  }

  double time_step = 1./(frequency*samples_per_period);

//  int8_t zero_crossings_1[MAX_SAMPLES] = {0};
//  int8_t zero_crossings_2[MAX_SAMPLES] = {0};

  for (long i = 1; i < MAX_SAMPLES; i++) {
    int8_t change_of_polarity1 = 0;
    int8_t change_of_polarity2 = 0;

    if (signal1[i] > 0. && signal1[i-1] < 0.) {
      change_of_polarity1 = 1;
    }
    else if (signal1[i] < 0. && signal1[i-1] > 0.) {
      change_of_polarity1 = -1;
    }

    if (signal2[i] > 0. && signal2[i-1] < 0.) {
      change_of_polarity2 = 1;
    }
    else if (signal2[i] < 0. && signal2[i-1] > 0.) {
      change_of_polarity2 = -1;
    }

    zero_crossings_1[i] = change_of_polarity1;
    zero_crossings_2[i] = change_of_polarity2;
  }

  printf("Phase differences:\r\n");

  double time_last_rising_edge_s1 = 0;
  double time_last_falling_edge_s1 = 0;

  double time_last_rising_edge_s2 = 0;
  double time_last_falling_edge_s2 = 0;

  double avg_phase_shift = 0.;
  long avg_cnt = 0;

  // Calcula relação atraso e utiliza angulo inverso caso a diferença seja maior
  // que 180 graus, assim definindo qual está atrasado em relação ao outro.
  for (long i = samples_per_period; i < MAX_SAMPLES; i++) {
    double time_rising_edge_s1 = 0.;
    double time_falling_edge_s1 = 0.;

    double time_rising_edge_s2 = 0.;
    double time_falling_edge_s2 = 0.;

    if (zero_crossings_1[i] == 1) {
      const double t1 = get_linear_root(signal1[i-1], signal1[i], time_step);
      time_rising_edge_s1 = (i-1)*time_step + t1;
    }
    else if (zero_crossings_1[i] == -1) {
      const double t1 = get_linear_root(signal1[i-1], signal1[i], time_step);
      time_falling_edge_s1 = (i-1)*time_step + t1;
    }

    if (zero_crossings_2[i] == 1) {
      const double t2 = get_linear_root(signal2[i-1], signal2[i], time_step);
      time_rising_edge_s2 = (i-1)*time_step + t2;
    }
    else if (zero_crossings_2[i] == -1) {
      const double t2 = get_linear_root(signal2[i-1], signal2[i], time_step);
      time_falling_edge_s2 = (i-1)*time_step + t2;
    }

    // @TODO testar diferença entre tempo calculado e definido do período
    if (time_last_rising_edge_s1 != 0 && time_last_rising_edge_s2 != 0 && zero_crossings_1[i] == 1) {
      double phase_diff = 360*((time_rising_edge_s1 - time_last_rising_edge_s2)/(time_rising_edge_s1 - time_last_rising_edge_s1));
      if (phase_diff > 180.0)
        phase_diff -= 360.0;
      printf("R: %3.2f\r\n", phase_diff);
//      phase_shift[i] = phase_diff;

      avg_phase_shift += phase_diff;
      avg_cnt++;
    }
    else if (time_last_falling_edge_s1 != 0 && time_last_falling_edge_s2 != 0 && zero_crossings_1[i] == -1) {
      double phase_diff = 360*(time_falling_edge_s1 - time_last_falling_edge_s2)/(time_falling_edge_s1 - time_last_falling_edge_s1);
      if (phase_diff > 180.0)
        phase_diff -= 360.0;
      printf("F: %3.2f\r\n", phase_diff);
//      phase_shift[i] = phase_diff;

      avg_phase_shift += phase_diff;
      avg_cnt++;
    }

    if (zero_crossings_1[i] == 1)
      time_last_rising_edge_s1 = time_rising_edge_s1;
    else if (zero_crossings_1[i] == -1)
      time_last_falling_edge_s1 = time_falling_edge_s1;

    if (zero_crossings_2[i] == 1)
      time_last_rising_edge_s2 = time_rising_edge_s2;
    else if (zero_crossings_2[i] == -1)
      time_last_falling_edge_s2 = time_falling_edge_s2;
  }

  printf("\r\nAverages made: %ld\r\n", avg_cnt);
  return (avg_cnt > 0) ? avg_phase_shift / avg_cnt : NAN;
}

/**
 * Get average signal peak per period
 * @param signal1 Siganl to find average peak
 * @param samples_per_period Number of points per period
 * @return Average of peak values per period of samples
 */
double get_peak_ac(double signal1[MAX_SAMPLES], signal_period_samples_t samples_per_period) {
   double last_peak = 0.;
   double accum_peak = 0.;
   uint32_t cnt = 0;
   for (size_t i = 1; i < MAX_SAMPLES; i++) {
     if (i % (samples_per_period) == 0) {
       cnt++;
       accum_peak += last_peak;
       last_peak = 0.;
     }
     last_peak = last_peak < fabs(signal1[i]) ? fabs(signal1[i]) : last_peak;
   }

   // Impede divisão por zero:
   return (cnt > 0) ? (accum_peak / cnt) : 0.0;
 }

#ifndef SWO_DEBUG
/**
 * Calculates the complex impedance in series with a known resistor from the voltages known
 * @param V1 Excitation voltage
 * @param V2 Voltage divided
 * @param V2_phase_degrees Phase difference between V1 and V2
 * @param series_resistance Resistance value between V2 and ground
 * @param config Tipe of mesurment circuit LR (INDUCTOR_LR) or RC (CAPACITOR_RC)
 * @return Complex form impedance form as Z_t structure
 */
z_t get_z(const double V1, const double V2, const double V2_phase_degrees,
          const resistance_values_t series_resistance,
          const element_configuration_t config) {
   const long double phase = (M_PI*V2_phase_degrees) / 180.0;

   // printf("\nPhase rad: %Lf\n rad", phase);

   const long double Re_V2 = V2 * cosl(phase);
   const long double Im_V2 = V2 * sinl(phase);
   // printf("Re{V2} = %Lf\n", Re_V2);
   // printf("Im{V2} = %Lf\n", Im_V2);

   long double Re_Vz;
   long double Im_Vz;

   long double Re_I, Im_I;

   if (config == INDUCTOR_LR) {
     Re_I = Re_V2 / series_resistance;
     Im_I = Im_V2 / series_resistance;

     // printf("Re{I} = %Lf\n", Re_I);
     // printf("Im{I} = %Lf\n", Im_I);

     Re_Vz = (V1 - Re_V2);
     Im_Vz = - Im_V2;

     // printf("Re{Vz} = %Lf\n", Re_Vz);
     // printf("Im{Vz} = %Lf\n", Im_Vz);
   } else { // CAPACITOR_RC
     Re_I = (V1 - Re_V2) / series_resistance;
     Im_I = -Im_V2 / series_resistance;

     // printf("Re{I} = %Lf\n", Re_I);
     // printf("Im{I} = %Lf\n", Im_I);

     Re_Vz = Re_V2;
     Im_Vz = Im_V2;

     // printf("Re{Vz} = %Lf\n", Re_Vz);
     // printf("Im{Vz} = %Lf\n", Im_Vz);
   }

   // Impedance Z = Vz/I. Rectangular division to increase numeric stability
   const long double denominator = Re_I * Re_I + Im_I * Im_I;
   const long double Re_Z = (Re_Vz * Re_I + Im_Vz * Im_I) / denominator;
   const long double Im_Z = (Im_Vz * Re_I - Re_Vz * Im_I) / denominator;

   z_t result = {Re_Z, Im_Z};
   return result;
 }
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
void main(void)
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
  HAL_Delay(100);
  printf("\r\n"); // start serial
  command_ready = 0xFF;
  samples_per_period_conf = TWENTY_SAMPLES;
  source_frequency_conf = ONE_MHZ;
  resistor_conf = TEN_K_OHMS;

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK || HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) {
    printf("ERROR: Failed to calibrate ADC\r\n");
  }

  if (HAL_ADC_Start(&hadc2) != HAL_OK) {
    printf("ERROR: Failed to start ADC2\r\n");
  }

#ifdef SWO_DEBUG
  Rx[0] = '2';
  Rx[1] = '\0';
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (adc_done)
    {
      double adc1[MAX_SAMPLES];
      double adc2[MAX_SAMPLES];

      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
      adc_done = 0;

      printf("Acquired %.0f samples. \r\n", (float)sample_count);
      printf("%-9s %-5s %-5s\r\n", "Time", "ADC1", "ADC2");

//      for (int i = 0; i < MAX_SAMPLES; i++)
//        ADC_timming_buffer_us[i] = i*1e6/(samples_per_period_conf*source_frequency_conf*1.);

      for (int i = 0; i < MAX_SAMPLES; i++) {
        const uint16_t val_adc1 = (DMA_ADC_buffer[i] & 0xFFFF);
        const uint16_t val_adc2 = (DMA_ADC_buffer[i] >> 16) & 0xFFFF;

        adc1[i] = round(val_adc1*(330.0/0x0FFF))/100;
        adc2[i] = round(val_adc2*(330.0/0x0FFF))/100;

        // printf("%09.2f %+01.3f %+01.3f \r\n", i*1e6/(samples_per_period_conf*source_frequency_conf*1.) , adc1[i], adc2[i]);

      }

      band_pass_filter(1, adc1, source_frequency_conf, samples_per_period_conf);
      band_pass_filter(1, adc2, source_frequency_conf, samples_per_period_conf);

      printf("\r\nPhase readings:\r\n");
      double phase = -get_phase_shift(adc1, adc2, source_frequency_conf, samples_per_period_conf);

      printf("\r\nFiltered readings:\r\n");
      printf("%-9s %-6s %-6s %-6s %-6s \r\n", "Time,", "ADC1,", "ADC2,","S1+-,", "S2+-");

      for (int i = 0; i < MAX_SAMPLES; i++) {
		printf("%06.02f, %+06.03f, %+06.03f, %+5d, %+5d\r\n",
			  i*1e6/(samples_per_period_conf*source_frequency_conf*1.),
			  adc1[i],
			  adc2[i],
			  zero_crossings_1[i], zero_crossings_2[i]);
      }

      double peak_sig1 = get_peak_ac(adc1, samples_per_period_conf);
      double peak_sig2 = get_peak_ac(adc2, samples_per_period_conf);

      printf("\r\nFrequency: %lu Hz\r\n", source_frequency_conf);
      printf("Samples per period: %lu\r\n", samples_per_period_conf);

      printf("Phase: %f degrees\r\n", phase);
      printf("Peak (source): %f volts\r\n", peak_sig2);
      printf("Peak (division node): %f volts\r\n", peak_sig1);
      printf("Series resistor: %lu ohms\r\n\r\n", resistor_conf);
#ifndef SWO_DEBUG

      switch (element_conf) {
        case INDUCTOR_LR: {
          const z_t impedance = get_z(peak_sig2, peak_sig1, phase, resistor_conf, element_conf);
          printf("|Z| = %3.3Le ohms\r\n", hypotl(impedance.real, impedance.imaginary));
          printf("ESR = %3.3Le ohms\r\n", impedance.real);
          long double L = impedance.imaginary / (2 * M_PI * source_frequency_conf);
          printf("L = %3.3Le H\r\n", L);
          break;
        }
        case CAPACITOR_RC: {
        const z_t impedance = get_z(peak_sig2, peak_sig1, phase, resistor_conf, element_conf);
        printf("|Z| = %3.3Le ohms\r\n", hypotl(impedance.real, impedance.imaginary));
        printf("ESR = %3.3Le ohms\r\n", impedance.real);
        long double C = -1.0 / (2 * M_PI * source_frequency_conf * impedance.imaginary);
        printf("C = %3.3Le F\r\n", C);
        break;
        }
        case RESISTOR_LR:
          long double R = resistor_conf*(peak_sig1/(peak_sig2 - peak_sig1));
          printf("R = %3.3Le ohms\r\n", R);
      }


#endif
      command_ready = 0xFF;
      HAL_Delay(100U);
      // NVIC_SystemReset();
    }
    if (command_ready == 0xFF) {
      uint8_t show_menu = 0xFF;

      if (strcmp((char*)Rx, "1") == 0) {
        printf("Toggle LED\r\n");
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
      else if (strcmp((char*)Rx, "2") == 0) {
        char reading_options[] =
        "\r\n===== Measure =====\r\n"
          "21. Inductor (LR circuit)\r\n"
          "22. Capacitor (RC circuit)\r\n"
          "23. Resistor (LR circuit)\r\n"
          "0. Go back\r\n"
          "Enter choice: ";
        while (CDC_Transmit_FS((uint8_t*)reading_options, strlen(reading_options)) == USBD_BUSY) {}
        show_menu = 0x00;
      }
      else if (strcmp((char*)Rx, "21") == 0) {
        element_conf = INDUCTOR_LR;

        if (Start_Dual_ADC_DMA(source_frequency_conf,
			  samples_per_period_conf,
			  MAX_SAMPLES,
			  DMA_ADC_buffer) != HAL_OK) {
            printf("ERROR: Failed to run ADC and DMA\r\n");
        }

        HAL_Delay(100U);
      }
      else if (strcmp((char*)Rx, "22") == 0) {
        element_conf = CAPACITOR_RC;

        if (Start_Dual_ADC_DMA(source_frequency_conf,
			  samples_per_period_conf,
			  MAX_SAMPLES,
			  DMA_ADC_buffer) != HAL_OK) {
            printf("ERROR: Failed to run ADC and DMA\r\n");
        }
        HAL_Delay(100U);
      }
      else if (strcmp((char*)Rx, "23") == 0) {
        element_conf = RESISTOR_LR;

        if (Start_Dual_ADC_DMA(source_frequency_conf,
        samples_per_period_conf,
        MAX_SAMPLES,
        DMA_ADC_buffer) != HAL_OK) {
          printf("ERROR: Failed to run ADC and DMA\r\n");
        }
        HAL_Delay(100U);
      }
      else if (strcmp((char*)Rx, "3") == 0) {
        printf("\r\n=== Settings ===\r\n"
          "Signal frequency: %lu Hz\r\n"
          "Samples per period: %lu\r\n"
          "Series resistor: %lu ohms\r\n",
          source_frequency_conf,
          samples_per_period_conf,
          resistor_conf
          );
        char settings[] =
          "31. Define signal frequency\r\n"
          "32. Set samples per period\r\n"
		      "33. Change series resistor\r\n"
          "0. Go back\r\n"
          "Enter choice: ";
        while (CDC_Transmit_FS((uint8_t*)settings, strlen(settings)) == USBD_BUSY) {}
        show_menu = 0x00;
      }
      else if (strcmp((char*)Rx, "31") == 0) {
        char signal_frequencies[] =
          "\r\n=== Excitation signal frequency ===\r\n"
          "311. 60 Hz\r\n"
          "312. 100 Hz\r\n"
          "313. 1 kHz\r\n"
          "314. 10 kHz\r\n"
          "315. 100 kHz\r\n"
          "316. 1 MHz\r\n"
          "317. 5 MHz\r\n"
          "0. Go back\r\n"
          "Enter choice: ";
        while (CDC_Transmit_FS((uint8_t*)signal_frequencies, strlen(signal_frequencies)) == USBD_BUSY) {}
        show_menu = 0x00;
      }
      else if (strcmp((char*)Rx, "311") == 0) {
        source_frequency_conf = SIXTY_HZ;
      }
      else if (strcmp((char*)Rx, "312") == 0) {
        source_frequency_conf = ONE_HUNDRED_HZ;
      }
      else if (strcmp((char*)Rx, "313") == 0) {
        source_frequency_conf = ONE_KHZ;
      }
      else if (strcmp((char*)Rx, "314") == 0) {
        source_frequency_conf = TEN_KHZ;
      }
      else if (strcmp((char*)Rx, "315") == 0) {
        source_frequency_conf = ONE_HUNDRED_KHZ;
      }
      else if (strcmp((char*)Rx, "316") == 0) {
        source_frequency_conf = ONE_MHZ;
      }
      else if (strcmp((char*)Rx, "317") == 0) {
        source_frequency_conf = FIVE_MHZ;
      }
      else if (strcmp((char*)Rx, "32") == 0) {
        char samplings[] =
          "\r\n=== Samples per period ===\r\n"
          "321. Two\r\n"
          "322. Five\r\n"
          "323. Ten\r\n"
          "324. Twenty\r\n"
          "0. Go back\r\n"
          "Enter choice: ";
        while (CDC_Transmit_FS((uint8_t*)samplings, strlen(samplings)) == USBD_BUSY) {
          __NOP();
        }
        show_menu = 0x00;
      }
      else if (strcmp((char*)Rx, "321") == 0) {
        samples_per_period_conf = TWO_SAMPLES;
      }
      else if (strcmp((char*)Rx, "322") == 0) {
        samples_per_period_conf = FIVE_SAMPLES;
      }
      else if (strcmp((char*)Rx, "323") == 0) {
        samples_per_period_conf = TEN_SAMPLES;
      }
      else if (strcmp((char*)Rx, "324") == 0) {
        samples_per_period_conf = TWENTY_SAMPLES;
      }
      else if (strcmp((char*)Rx, "33") == 0) {
        char resistors[] =
          "\r\n=== Series Resistance (ohms) ===\r\n"
           "331. 200\r\n"
           "332. 1000\r\n"
           "333. 10000\r\n"
           "334. 100000\r\n"
           "0. Go back\r\n"
           "Enter choice: ";
        while (CDC_Transmit_FS((uint8_t*)resistors, strlen(resistors)) == USBD_BUSY) {
          __NOP();
        }
        show_menu = 0x00;
      }
      else if (strcmp((char*)Rx, "331") == 0) {
        resistor_conf = TWO_HUNDRED_OHMS;
      }
      else if (strcmp((char*)Rx, "332") == 0) {
        resistor_conf = ONE_K_OHMS;
      }
      else if (strcmp((char*)Rx, "333") == 0) {
        resistor_conf = TEN_K_OHMS;
      }
      else if (strcmp((char*)Rx, "334") == 0) {
        resistor_conf = ONE_HUNDRED_K_OHMS;
      }
      else if (strcmp((char*)Rx, "0") == 0) {
        __NOP();
      }
      else if (strcmp((char*)Rx, "4") == 0) {
        printf("Reset...\r\n");

        HAL_Delay(100);
        cmd_index = 0;
        NVIC_SystemReset();
      }

      if (show_menu == 0xFF) {
        char menu[] =
        "\r\n=== Main Menu ===\r\n"
        "1. Blink LED\r\n"
        "2. Read ADC\r\n"
        "3. Settings\r\n"
        "4. Reset MCU\r\n"
        "Enter choice: ";
#ifndef SWO_DEBUG
        while (CDC_Transmit_FS((uint8_t*)menu, strlen(menu)) == USBD_BUSY) {
          __NOP();
        }
#endif
      }

      cmd_index = 0;
      command_ready = 0x00;
      memset(Rx, 0, sizeof(Rx));
#ifdef SWO_DEBUG
  Rx[0] = '2';
  Rx[1] = '\0';
#endif
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
  printf("[%lu ms] Error! \r\n", HAL_GetTick());
  __disable_irq();
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

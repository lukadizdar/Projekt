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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "audio.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 512
#define NUM_TAPS 512 //ZA FILTER KOLIKO JE IDEALAN
#define SAMPLE_FREQ 43478
#define BLOCK_SIZE 64 //
#define ECHO_DELAY 512 // Delay in samples
#define ECHO_STRENGTH 0.7f // Echo strength (0.0 to 1.0)
#define FFT_SIZE 512

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */






/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint16_t fake_signal[BUFFER_SIZE]; // Buffer za dummy signal
uint16_t adc_signal[BUFFER_SIZE];
q15_t output_signal[BUFFER_SIZE];
q15_t fake_signal[BUFFER_SIZE];
volatile uint8_t fx_ready = 0;
uint32_t last_systick = 0;
q15_t conv_signal[BUFFER_SIZE];
q15_t filtered_signal[BUFFER_SIZE];
q15_t i2s_signal[BUFFER_SIZE*2];
//q15_t output_signal[BUFFER_SIZE];
//q15_t output_signal[BUFFER_SIZE];
//256 bitna verzija
//float32_t firCoeffs[NUM_TAPS] = {
//    0.0001, 0.0000, -0.0000, -0.0000, -0.0000, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0005, 0.0004, 0.0003, 0.0002, 0.0001, 0.0000, 0.0000,
//    0.0001, 0.0002, 0.0004, 0.0006, 0.0008, 0.0009, 0.0009, 0.0007, 0.0005, 0.0003, 0.0001, -0.0000, -0.0000, 0.0001, 0.0004, 0.0007, 0.0011,
//    0.0014, 0.0015, 0.0014, 0.0011, 0.0006, 0.0002, -0.0002, -0.0005, -0.0004, -0.0002, 0.0003, 0.0009, 0.0015, 0.0018, 0.0019, 0.0016, 0.0009,
//    0.0001, -0.0007, -0.0014, -0.0018, -0.0017, -0.0012, -0.0004, 0.0006, 0.0014, 0.0018, 0.0017, 0.0010, -0.0002, -0.0016, -0.0030, -0.0041, -0.0045,
//    -0.0042, -0.0033, -0.0020, -0.0005, 0.0006, 0.0011, 0.0007, -0.0006, -0.0026, -0.0049, -0.0070, -0.0085, -0.0089, -0.0083, -0.0066, -0.0043, -0.0021,
//    -0.0004, 0.0002, -0.0007, -0.0030, -0.0062, -0.0098, -0.0129, -0.0148, -0.0151, -0.0135, -0.0105, -0.0066, -0.0028, -0.0002, 0.0005, -0.0012, -0.0051,
//    -0.0106, -0.0164, -0.0213, -0.0240, -0.0237, -0.0202, -0.0140, -0.0064, 0.0008, 0.0058, 0.0069, 0.0034, -0.0048, -0.0163, -0.0290, -0.0400, -0.0462,
//    -0.0452, -0.0352, -0.0161, 0.0109, 0.0428, 0.0759, 0.1059, 0.1286, 0.1408, 0.1408, 0.1286, 0.1059, 0.0759, 0.0428, 0.0109, -0.0161, -0.0352,
//    -0.0452, -0.0462, -0.0400, -0.0290, -0.0163, -0.0048, 0.0034, 0.0069, 0.0058, 0.0008, -0.0064, -0.0140, -0.0202, -0.0237, -0.0240, -0.0213, -0.0164,
//    -0.0106, -0.0051, -0.0012, 0.0005, -0.0002, -0.0028, -0.0066, -0.0105, -0.0135, -0.0151, -0.0148, -0.0129, -0.0098, -0.0062, -0.0030, -0.0007, 0.0002,
//    -0.0004, -0.0021, -0.0043, -0.0066, -0.0083, -0.0089, -0.0085, -0.0070, -0.0049, -0.0026, -0.0006, 0.0007, 0.0011, 0.0006, -0.0005, -0.0020, -0.0033,
//    -0.0042, -0.0045, -0.0041, -0.0030, -0.0016, -0.0002, 0.0010, 0.0017, 0.0018, 0.0014, 0.0006, -0.0004, -0.0012, -0.0017, -0.0018, -0.0014, -0.0007,
//    0.0001, 0.0009, 0.0016, 0.0019, 0.0018, 0.0015, 0.0009, 0.0003, -0.0002, -0.0004, -0.0005, -0.0002, 0.0002, 0.0006, 0.0011, 0.0014, 0.0015,
//    0.0014, 0.0011, 0.0007, 0.0004, 0.0001, -0.0000, -0.0000, 0.0001, 0.0003, 0.0005, 0.0007, 0.0009, 0.0009, 0.0008, 0.0006, 0.0004, 0.0002,
//    0.0001, 0.0000, 0.0000, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0005, 0.0004, 0.0003, 0.0002, 0.0001, -0.0000, -0.0000, -0.0000, 0.0000,
//    0.0001
//};
float32_t firCoeffs[NUM_TAPS] = {
    0.0001, 0.0000, 0.0000, -0.0000, 0.0000, 0.0000, 0.0001, 0.0001, 0.0002, 0.0002, 0.0002, 0.0002, 0.0001, 0.0001, 0.0000, -0.0000, -0.0000,
    -0.0000, 0.0000, 0.0001, 0.0002, 0.0002, 0.0002, 0.0002, 0.0002, 0.0001, 0.0000, -0.0000, -0.0001, -0.0001, -0.0001, -0.0000, 0.0001, 0.0001,
    0.0002, 0.0002, 0.0002, 0.0001, 0.0000, -0.0001, -0.0002, -0.0002, -0.0002, -0.0002, -0.0001, -0.0000, 0.0001, 0.0001, 0.0001, 0.0001, -0.0000,
    -0.0002, -0.0003, -0.0004, -0.0005, -0.0005, -0.0004, -0.0003, -0.0001, -0.0000, 0.0001, 0.0000, -0.0000, -0.0002, -0.0004, -0.0006, -0.0007, -0.0007,
    -0.0007, -0.0006, -0.0004, -0.0002, -0.0001, -0.0000, -0.0000, -0.0002, -0.0004, -0.0006, -0.0008, -0.0010, -0.0010, -0.0009, -0.0007, -0.0005, -0.0002,
    -0.0000, 0.0001, 0.0000, -0.0002, -0.0004, -0.0007, -0.0010, -0.0011, -0.0011, -0.0010, -0.0007, -0.0003, 0.0000, 0.0003, 0.0004, 0.0003, 0.0001,
    -0.0003, -0.0006, -0.0009, -0.0011, -0.0010, -0.0007, -0.0003, 0.0002, 0.0006, 0.0010, 0.0010, 0.0009, 0.0006, 0.0001, -0.0003, -0.0007, -0.0008,
    -0.0006, -0.0002, 0.0004, 0.0010, 0.0016, 0.0019, 0.0020, 0.0017, 0.0013, 0.0007, 0.0001, -0.0002, -0.0003, -0.0001, 0.0005, 0.0013, 0.0021,
    0.0027, 0.0030, 0.0030, 0.0026, 0.0019, 0.0012, 0.0005, 0.0001, 0.0000, 0.0004, 0.0011, 0.0021, 0.0030, 0.0037, 0.0040, 0.0038, 0.0032,
    0.0022, 0.0012, 0.0004, -0.0001, -0.0001, 0.0004, 0.0014, 0.0025, 0.0035, 0.0043, 0.0044, 0.0040, 0.0031, 0.0018, 0.0004, -0.0006, -0.0012,
    -0.0011, -0.0004, 0.0008, 0.0022, 0.0034, 0.0041, 0.0041, 0.0033, 0.0019, 0.0002, -0.0015, -0.0028, -0.0034, -0.0032, -0.0022, -0.0006, 0.0010,
    0.0024, 0.0031, 0.0028, 0.0017, -0.0003, -0.0026, -0.0047, -0.0063, -0.0069, -0.0064, -0.0049, -0.0029, -0.0007, 0.0009, 0.0016, 0.0010, -0.0008,
    -0.0035, -0.0065, -0.0092, -0.0110, -0.0115, -0.0105, -0.0083, -0.0054, -0.0025, -0.0005, 0.0002, -0.0009, -0.0035, -0.0073, -0.0114, -0.0149, -0.0171,
    -0.0172, -0.0153, -0.0118, -0.0073, -0.0031, -0.0002, 0.0005, -0.0013, -0.0056, -0.0114, -0.0175, -0.0227, -0.0254, -0.0250, -0.0211, -0.0146, -0.0066,
    0.0009, 0.0060, 0.0071, 0.0034, -0.0049, -0.0166, -0.0295, -0.0405, -0.0468, -0.0456, -0.0355, -0.0162, 0.0109, 0.0430, 0.0762, 0.1062, 0.1289,
    0.1412, 0.1412, 0.1289, 0.1062, 0.0762, 0.0430, 0.0109, -0.0162, -0.0355, -0.0456, -0.0468, -0.0405, -0.0295, -0.0166, -0.0049, 0.0034, 0.0071,
    0.0060, 0.0009, -0.0066, -0.0146, -0.0211, -0.0250, -0.0254, -0.0227, -0.0175, -0.0114, -0.0056, -0.0013, 0.0005, -0.0002, -0.0031, -0.0073, -0.0118,
    -0.0153, -0.0172, -0.0171, -0.0149, -0.0114, -0.0073, -0.0035, -0.0009, 0.0002, -0.0005, -0.0025, -0.0054, -0.0083, -0.0105, -0.0115, -0.0110, -0.0092,
    -0.0065, -0.0035, -0.0008, 0.0010, 0.0016, 0.0009, -0.0007, -0.0029, -0.0049, -0.0064, -0.0069, -0.0063, -0.0047, -0.0026, -0.0003, 0.0017, 0.0028,
    0.0031, 0.0024, 0.0010, -0.0006, -0.0022, -0.0032, -0.0034, -0.0028, -0.0015, 0.0002, 0.0019, 0.0033, 0.0041, 0.0041, 0.0034, 0.0022, 0.0008,
    -0.0004, -0.0011, -0.0012, -0.0006, 0.0004, 0.0018, 0.0031, 0.0040, 0.0044, 0.0043, 0.0035, 0.0025, 0.0014, 0.0004, -0.0001, -0.0001, 0.0004,
    0.0012, 0.0022, 0.0032, 0.0038, 0.0040, 0.0037, 0.0030, 0.0021, 0.0011, 0.0004, 0.0000, 0.0001, 0.0005, 0.0012, 0.0019, 0.0026, 0.0030,
    0.0030, 0.0027, 0.0021, 0.0013, 0.0005, -0.0001, -0.0003, -0.0002, 0.0001, 0.0007, 0.0013, 0.0017, 0.0020, 0.0019, 0.0016, 0.0010, 0.0004,
    -0.0002, -0.0006, -0.0008, -0.0007, -0.0003, 0.0001, 0.0006, 0.0009, 0.0010, 0.0010, 0.0006, 0.0002, -0.0003, -0.0007, -0.0010, -0.0011, -0.0009,
    -0.0006, -0.0003, 0.0001, 0.0003, 0.0004, 0.0003, 0.0000, -0.0003, -0.0007, -0.0010, -0.0011, -0.0011, -0.0010, -0.0007, -0.0004, -0.0002, 0.0000,
    0.0001, -0.0000, -0.0002, -0.0005, -0.0007, -0.0009, -0.0010, -0.0010, -0.0008, -0.0006, -0.0004, -0.0002, -0.0000, -0.0000, -0.0001, -0.0002, -0.0004,
    -0.0006, -0.0007, -0.0007, -0.0007, -0.0006, -0.0004, -0.0002, -0.0000, 0.0000, 0.0001, -0.0000, -0.0001, -0.0003, -0.0004, -0.0005, -0.0005, -0.0004,
    -0.0003, -0.0002, -0.0000, 0.0001, 0.0001, 0.0001, 0.0001, -0.0000, -0.0001, -0.0002, -0.0002, -0.0002, -0.0002, -0.0001, 0.0000, 0.0001, 0.0002,
    0.0002, 0.0002, 0.0001, 0.0001, -0.0000, -0.0001, -0.0001, -0.0001, -0.0000, 0.0000, 0.0001, 0.0002, 0.0002, 0.0002, 0.0002, 0.0002, 0.0001,
    0.0000, -0.0000, -0.0000, -0.0000, 0.0000, 0.0001, 0.0001, 0.0002, 0.0002, 0.0002, 0.0002, 0.0001, 0.0001, 0.0000, 0.0000, -0.0000, 0.0000,
    0.0000, 0.0001
};
q15_t firCoeffsQ15[NUM_TAPS];
q15_t firStateQ15[NUM_TAPS + BLOCK_SIZE - 1]; // FIR state buffer

arm_fir_instance_q15 S;
//IIR
//q15_t band1_coeffs[5] = {8, 0, -16, 0, 8};       // Band 1 ZA 80-8000
//q15_t band2_coeffs[5] = {76, 0, -152, 0, 76};    // Band 2
//q15_t band3_coeffs[5] = {551, 0, -1102, 0, 551}; // Band 3
//q15_t band4_coeffs[5] = {1140, 0, -2280, 0, 1140}; // Band 4
//q15_t band5_coeffs[5] = {3672, 0, -7344, 0, 3672}; // Band 5

q15_t band1_coeffs[5] = {7, 0, -13, 0, 7};       // Narrower Band 1
q15_t band2_coeffs[5] = {26, 0, -51, 0, 26};     // Narrower Band 2
q15_t band3_coeffs[5] = {98, 0, -197, 0, 98};    // Narrower Band 3
q15_t band4_coeffs[5] = {213, 0, -426, 0, 213};  // Narrower Band 4
q15_t band5_coeffs[5] = {56, 0, -113, 0, 56};
volatile uint32_t buffer_overrun_count = 0;
volatile uint8_t flag_uart = 0;
volatile uint8_t call_uart_once = 1;

q15_t band1_state[4] = {0};
q15_t band2_state[4] = {0};
q15_t band3_state[4] = {0};
q15_t band4_state[4] = {0};
q15_t band5_state[4] = {0};
arm_biquad_casd_df1_inst_q15 eqBands[5];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  q15_t test = -2022;   // Skaliraj 2022 na Q15 format
  float izbor = 0.1;
  q15_t multiplier = (q15_t)(0x7FFF * izbor);         // Q15 format za 1.0
  q15_t rezultat = 0;
  arm_mult_q15(&test, &multiplier, &rezultat, 1);

  configAudio();
  init_fir_filter();

  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, adc_signal, BUFFER_SIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    if (fx_ready == 1) {
		fx_ready = 0;
		 convert_to_q15(adc_signal, conv_signal, BUFFER_SIZE);

        last_systick = HAL_GetTick();
//		echo_effect(conv_signal, output_signal, BUFFER_SIZE, 0.5, 0.5);
//		echo_effect_q15(conv_signal, output_signal, BUFFER_SIZE);
//		tremolo_effect(conv_signal, output_signal, BUFFER_SIZE, 440);
//		smooth_signal_q15(conv_signal, output_signal, BUFFER_SIZE, 5);
//		last_systick = HAL_GetTick();

		last_systick = HAL_GetTick();
//		amplify_signal_q15(conv_signal, gained_signal, BUFFER_SIZE, 1.0f);
		last_systick = HAL_GetTick();

		fir_filter(conv_signal, filtered_signal);
//		amplify_q15_cmsis(filtered_signal, output_signal, BUFFER_SIZE, 0.1f);
		convert_to_i2s(filtered_signal, i2s_signal, BUFFER_SIZE);
		if (flag_uart) callUart(i2s_signal);
		flag_uart = 0;
//		process_equalizer(filtered_signal, output_signal, BUFFER_SIZE);
//		amplify_signal_q15(filtered_signal, output_signal, BUFFER_SIZE, 1.4f);

//		tremolo_effect(filtered_signal, output_signal, BUFFER_SIZE, 440);
//		echo_effect(filtered_signal, output_signal, BUFFER_SIZE, 0.5, 0.5);


		last_systick = HAL_GetTick();




        last_systick = HAL_GetTick();

	}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//ECHO EFEKT
void echo_effect(uint16_t *buffer, uint16_t *outputBuffer, int size, float echo_strength, int delay) {
    static uint16_t echo_buffer[BUFFER_SIZE * 2] = {0}; // Povećan buffer za "delay"
    for (int i = 0; i < size; i++) {
        int delayed_index = i - delay;
        if (delayed_index >= 0) {
            outputBuffer[i] = buffer + (uint16_t)(echo_strength * echo_buffer[delayed_index]);
        }
        echo_buffer[i] = buffer[i];
    }
}

void echo_effect_q15(q15_t *input, q15_t *output, int size) {
    static q15_t echo_buffer[ECHO_DELAY] = {0}; // Circular buffer for delay
    static int echo_index = 0;

    q15_t scaled_echo;
    q15_t echo_multiplier = (q15_t)(ECHO_STRENGTH * 32767); // Convert strength to Q15 format

    for (int i = 0; i < size; i++) {
        // Scale the delayed sample by echo strength
//        arm_scale_q15(&echo_buffer[echo_index], echo_multiplier, 0, &scaled_echo, 1);
        arm_scale_q15(&scaled_echo, 0.8 * 32767, 0, &scaled_echo, 1);


        // Add scaled echo to the input signal
        output[i] = __SSAT((int32_t)(input[i] * 0.8f + scaled_echo), 16);


        // Store the current sample in the circular buffer
        echo_buffer[echo_index] = input[i];

        // Increment and wrap the index
        echo_index = (echo_index + 1) % ECHO_DELAY;
    }
}

void tremolo_effect(q15_t *input, q15_t *output, int size, float freq) {
    static float phase = 0.0f;
    float increment = (2.0f * PI * freq) / SAMPLE_FREQ;

    for (int i = 0; i < size; i++) {
        float modulation = 0.5f * (1.0f + arm_sin_f32(phase)); // Modulation range: 0.5 to 1
        output[i] = __SSAT((int32_t)(input[i] * modulation), 16); //ssat sprijeci clipping
        phase += increment;
        if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
}

void amplify_q15_cmsis(const q15_t *input_signal, q15_t *output_signal, size_t length, float gain_percent) {
    q15_t gain = (q15_t)(0x7FFF * gain_percent);
	q15_t gain_array[length]; // Array of gains
    for (size_t i = 0; i < length; i++) {
        gain_array[i] = gain;
    }
    // Use CMSIS function for vectorized Q15 multiplication
    arm_mult_q15(input_signal, gain_array, output_signal, length);
}


//FIR FILTER
void convert_to_q15(uint16_t *rawInput, q15_t *convertedSignal, int size) {
    int i = 0;
        // Map uint16_t (0 to 65535) to q15_t (-32768 to 32767)
//        convertedSignal[i] = (q15_t)((int32_t)(rawInput[i] - 32768));
        for (i = 0; i < size; i++) {
        	convertedSignal[i] = (q15_t)((int32_t)(rawInput[i]));
        }
//    	convertedSignal[i] = (q15_t)((int32_t)(rawInput[i]));
//    	convertedSignal[i] = (q15_t)((rawInput[i] * 32767) / 4096);
//    	for (int j = 0; j < size*2; j += 2) {
//    		convertedSignal[j] = (q15_t)(rawInput[i]);
//    		convertedSignal[j+1] = (q15_t)(rawInput[i]);
//    		i++;
//    	}

}

void convert_to_i2s(q15_t *rawInput, q15_t *convertedSignal, int size) {
    int i = 0;
    	for (int j = 0; j < size*2; j += 2) {
    		convertedSignal[j] = (q15_t)(rawInput[i]);
    		convertedSignal[j+1] = (q15_t)(rawInput[i]);
    		i++;
    	}
}



void callUart(uint16_t* input) {
	  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)input, sizeof(input));
	  HAL_I2S_Transmit_DMA(&hi2s3, input, BUFFER_SIZE);
}




void init_fir_filter(void) {
    arm_float_to_q15(firCoeffs, firCoeffsQ15, NUM_TAPS);
    arm_fir_init_q15(&S, NUM_TAPS, firCoeffsQ15, firStateQ15, BLOCK_SIZE);
}
void fir_filter(q15_t *input, q15_t *output) {
    for (int i = 0; i < BUFFER_SIZE; i += BLOCK_SIZE) {
        arm_fir_q15(&S, &input[i], &output[i], BLOCK_SIZE);
    }
}
void rfft(q15_t *inputSignal, q15_t *fftOutput, q15_t *magnitudeSpectrum) {
    arm_rfft_instance_q15 rfftInstance;

    arm_rfft_init_q15(&rfftInstance, FFT_SIZE, 0, 1);

    arm_rfft_q15(&rfftInstance, inputSignal, fftOutput); //rfft buffer izgleda jako cudno tho, DC, Nyquist, real1, imag1,

    arm_cmplx_mag_q15(fftOutput, magnitudeSpectrum, FFT_SIZE / 2); //ovo je za magnitudes, mora biti /2 jer je simetrično, nyquistov dijagram iz automatskog samo poz frekv
}


void init_filters() {
    arm_biquad_cascade_df1_init_q15(&eqBands[0], 1, band1_coeffs, band1_state, 0);
    arm_biquad_cascade_df1_init_q15(&eqBands[1], 1, band2_coeffs, band2_state, 0);
    arm_biquad_cascade_df1_init_q15(&eqBands[2], 1, band3_coeffs, band3_state, 0);
    arm_biquad_cascade_df1_init_q15(&eqBands[3], 1, band4_coeffs, band4_state, 0);
    arm_biquad_cascade_df1_init_q15(&eqBands[4], 1, band5_coeffs, band5_state, 0);
}
void process_equalizer(q15_t *input, q15_t *output, uint32_t blockSize) {
    q15_t bandOutputs[5][blockSize];   // Temporary storage for each band output
    q15_t gains[5] = {0xF000, 0xF800, 0xF800, 0xF000, 0xE000};  // MASSIVE boosts (up to ~8x per band)

    // Apply each band filter to the input signal
    for (int i = 0; i < 5; i++) {
        arm_biquad_cascade_df1_q15(&eqBands[i], input, bandOutputs[i], blockSize);
    }

    // Combine outputs of all bands into the final signal
    for (uint32_t n = 0; n < blockSize; n++) {
        int32_t sum = 0;

        // Apply gain to each band and sum
        for (int b = 0; b < 5; b++) {
            sum += ((int32_t)bandOutputs[b][n] * gains[b]) >> 15; // Apply gain to Q15 band outputs
        }

        // Saturate the summed value to prevent overflow
        output[n] = (q15_t)__SSAT(sum, 16); // Clipping to Q15 range
    }
}




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
	fx_ready = 1;
	if (call_uart_once = 1) {
		flag_uart =  1;
	}
	call_uart_once = 0;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI3) {
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Example: Check if buffer is full
    if (fx_ready) {
        buffer_overrun_count++;
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
  __disable_irq();
  while (1)
  {
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

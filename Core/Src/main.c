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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "audio.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 1024
#define SAMPLE_FREQ 44000
#define SIGNAL1_FREQ 2000 // Frequency of the first sine wave in Hz
#define SIGNAL2_FREQ 6000 // Frequency of the second sine wave in Hz
#define NUM_TAPS 44 //ZA FILTER
#define BLOCK_SIZE 32 // Start with 32, adjust as needed
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t fake_signal[BUFFER_SIZE]; // Buffer za dummy signal
uint16_t real_signal[BUFFER_SIZE];
uint32_t last_systick = 0;
uint32_t time_diff = 0;
volatile uint8_t fx_ready = 0;
q15_t convInputSignal[BUFFER_SIZE];
q15_t outputSignal[BUFFER_SIZE];
float32_t firCoeffs32[NUM_TAPS] = {
    -0.0003, -0.0010, -0.0016, -0.0019, -0.0015,  0.0000,  0.0027,  0.0060,
     0.0084,  0.0082,  0.0038, -0.0048, -0.0158, -0.0256, -0.0290, -0.0214,
     0.0000,  0.0347,  0.0781,  0.1225,  0.1590,  0.1796,  0.1796,  0.1590,
     0.1225,  0.0781,  0.0347,  0.0000, -0.0214, -0.0290, -0.0256, -0.0158,
    -0.0048,  0.0038,  0.0082,  0.0084,  0.0060,  0.0027,  0.0000, -0.0015,
    -0.0019, -0.0016, -0.0010, -0.0003
};
q15_t firCoeffsQ15[NUM_TAPS];
q15_t firStateQ15[NUM_TAPS + BLOCK_SIZE - 1]; // FIR state buffer
arm_fir_instance_q15 S;
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
  last_systick = HAL_GetTick();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  configAudio();
  last_systick = HAL_GetTick(); //traje 5 milisekundi
  generate_test_signal(fake_signal, BUFFER_SIZE); //traje 1 milisekundu
//  last_systick = HAL_GetTick();
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_I2S_Transmit_IT(&hi2s3, fake_signal, BUFFER_SIZE);
  last_systick = HAL_GetTick();

  last_systick = HAL_GetTick();
  //uint16_t signal;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    last_systick = HAL_GetTick();
    if (fx_ready == 1) {
		fx_ready = 0;
		 last_systick = HAL_GetTick();
		 convert_to_q15(real_signal, convInputSignal, BUFFER_SIZE);
		 last_systick = HAL_GetTick();

        last_systick = HAL_GetTick();
//		echo_effect(convInputSignal, BUFFER_SIZE, 10, 0.5);
		last_systick = HAL_GetTick();

//        init_fir_filter();
//        fir_filter();

	}
	}
  }
  /* USER CODE END 3 */

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
void generate_test_signal(uint16_t *buffer, int size) {
    for (int i = 0; i < size; i++) {
        // Generate a 2 kHz sine wave
        float sine_wave1 = sinf(2 * M_PI * SIGNAL1_FREQ * i / SAMPLE_FREQ);

        // Generate a 6 kHz sine wave
        float sine_wave2 = sinf(2 * M_PI * SIGNAL2_FREQ * i / SAMPLE_FREQ);

        // Combine the two sine waves
        float combined_signal = sine_wave1 + sine_wave2;

        // Add some random noise to simulate MEMS microphone behavior
        float noise = ((float)rand() / RAND_MAX) * 0.1f - 0.05f; // Noise in the range [-0.05, 0.05]

        // Scale the combined signal to uint16_t range [0, 65535]
        buffer[i] = (uint16_t)((combined_signal + noise) * 32767.0f + 32768.0f);
        buffer[i] = 0;
    }
}
//ECHO EFEKT
void echo_effect(uint16_t *buffer, int size, float echo_strength, int delay) {
    static uint16_t echo_buffer[BUFFER_SIZE * 2] = {0}; // Povećan buffer za "delay"
    for (int i = 0; i < size; i++) {
        int delayed_index = i - delay;
        if (delayed_index >= 0) {
            buffer[i] += (uint16_t)(echo_strength * echo_buffer[delayed_index]);
        }
        echo_buffer[i] = buffer[i];
    }
}
//FIR FILTER
void convert_to_q15(uint16_t *rawInput, q15_t *convertedSignal, int size) {
    for (int i = 0; i < size; i++) {
        // Map uint16_t (0 to 65535) to q15_t (-32768 to 32767)
        convertedSignal[i] = (q15_t)((int16_t)(rawInput[i] - 32768));
    }
}

void init_fir_filter(void) {
    arm_float_to_q15(firCoeffs32, firCoeffsQ15, NUM_TAPS);
    arm_fir_init_q15(&S, NUM_TAPS, firCoeffsQ15, firStateQ15, BLOCK_SIZE);
}
void fir_filter(void) {
    for (int i = 0; i < BUFFER_SIZE; i += BLOCK_SIZE) {
        arm_fir_q15(&S, &convInputSignal[i], &outputSignal[i], BLOCK_SIZE);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		//sada je generate fake signal napunio real buffer
    		memcpy(real_signal, fake_signal, BUFFER_SIZE);
    		fx_ready = 1;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI3) { // Provjeri je li I2S3
    	last_systick = HAL_GetTick();
    	HAL_I2S_Transmit_IT(&hi2s3, convInputSignal, BUFFER_SIZE);
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

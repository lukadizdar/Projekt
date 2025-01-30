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
#define ADC_SIZE 1024
#define SAMPLE_FREQ 45455
#define ECHO_DELAY 512 // Delay in samples
#define ECHO_STRENGTH 0.7f // Echo strength (0.0 to 1.0)
#define FFT_SIZE 512
#define ECHO_BUFFER_SIZE 4096 // Adjust as needed for delay length

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */






/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Signals
uint16_t adc_buffer[ADC_SIZE];
q15_t conv_signal[ADC_SIZE/2];
q15_t output_signal[ADC_SIZE/2];
q15_t filtered_signal[ADC_SIZE/2];
q15_t echo_signal[ADC_SIZE/2];
q15_t i2s_signal[ADC_SIZE*2];
uint8_t master_volume = 8;
q15_t distortion(q15_t sample, int bitDepth);


//FFT signals
q15_t fft_signal[ADC_SIZE/2];
q15_t fftOutput[FFT_SIZE*2];
q15_t magnitudeSpectrum[FFT_SIZE/2];
q15_t peakVal = 0;
uint16_t peakHz = 0;
q15_t offset;

//Flags
volatile uint8_t fx_ready = 0;
volatile uint8_t adc_half_flag = 0;
volatile uint8_t adc_done_flag = 0;
volatile uint8_t conv_flag = 0;
volatile uint8_t i2s_send_flag = 0;
volatile uint8_t flag_uart = 0;
volatile uint8_t call_uart_once = 1;
volatile uint8_t distortion_flag = 0;
volatile uint8_t distortion_strength = 6;

arm_rfft_instance_q15 rfft_instance;
q15_t highPassCoeffsQ15[5] = {16384, -16384, 0, 16364, -16359};
arm_biquad_casd_df1_inst_q15 highPassFilter;
q15_t highPassState[2] = {0};




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

	q15_t number = 394;
	int num = 2;
	q15_t output = number * 2;

	float da = 0.5f;

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
  configAudio();
  arm_rfft_init_q15(&rfft_instance, FFT_SIZE, 0, 1);

  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, ADC_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
	  if (adc_half_flag == 1) {
    	adc_half_flag = 0;
    	conv_flag = 0;
    	init_highpass_filter();
    	convert_to_q15(adc_buffer, conv_signal, ADC_SIZE);
    	center_signal(conv_signal, ADC_SIZE/2);
    	apply_highpass_filter(conv_signal, filtered_signal, ADC_SIZE/2);
    	center_signal(filtered_signal, ADC_SIZE/2);

    	memcpy(fft_signal, filtered_signal, FFT_SIZE*sizeof(q15_t));
		rfft(fft_signal, fftOutput, magnitudeSpectrum);
		peak_values_fft(magnitudeSpectrum);
    	offset_signal(filtered_signal, output_signal, ADC_SIZE/2);

    	if (distortion_flag == 1) {
        	for (int i = 0; i < ADC_SIZE/2; i++) {
        		output_signal[i] = distortion(output_signal[i], distortion_strength);
        	}
    	}




		i2s_send_flag = 0; //DA KRENE PRVU POLOVICU

		convert_to_i2s(output_signal, i2s_signal, ADC_SIZE);

    }

    if (adc_done_flag == 1) {
    	adc_done_flag = 0;
    	conv_flag = 1;
    	init_highpass_filter();
    	convert_to_q15(adc_buffer, conv_signal, ADC_SIZE);
    	center_signal(conv_signal, ADC_SIZE/2);
    	apply_highpass_filter(conv_signal, filtered_signal, ADC_SIZE/2);
    	center_signal(filtered_signal, ADC_SIZE/2);


    	memcpy(fft_signal, filtered_signal, FFT_SIZE*sizeof(q15_t));
		rfft(fft_signal, fftOutput, magnitudeSpectrum);
		peak_values_fft(magnitudeSpectrum);
    	offset_signal(filtered_signal, output_signal, ADC_SIZE/2);

    	if (distortion_flag == 1) {
        	for (int i = 0; i < ADC_SIZE/2; i++) {
        		output_signal[i] = distortion(output_signal[i], distortion_strength);
        	}
    	}



    	i2s_send_flag = 1; //DA KRENE DRUGU POLOVICU
    	convert_to_i2s(output_signal, i2s_signal, ADC_SIZE);
		if (flag_uart == 1) {
			callUart(i2s_signal);
		}
		flag_uart = 0;

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


// Function to apply distortion effect (e.g., hard clipping)
q15_t distortion(q15_t sample, int bitDepth) {
    int shift = 15 - bitDepth;
    return (sample >> shift) << shift;
}

void convert_to_q15(uint16_t *rawInput, q15_t *convertedSignal, int size) {
    int i;
    if (conv_flag == 0) {
    	for (i = 0; i < size / 2; i++) {
    		convertedSignal[i] = (q15_t)((rawInput[i]) * master_volume);
        }
    }
    if (conv_flag == 1) {
    	for (i = size / 2; i < size; i++) {
    		convertedSignal[i] = (q15_t)((rawInput[i]) * master_volume);
        }
    }
}
void center_signal(q15_t *input, uint16_t size) {
    q15_t mean;
    arm_mean_q15(input, size, &mean);
    for (uint16_t i = 0; i < size; i++) {
        input[i] -= mean; // Oduzimanje DC offseta
    }
}
void offset_signal(q15_t *input, q15_t *output, uint16_t size, q15_t offset) {
	arm_offset_q15(input, -offset, output, size);
}
void convert_to_i2s(q15_t *rawInput, q15_t *convertedSignal, int size) {
    int i = 0;
    	if (i2s_send_flag == 0) {
        	for (int j = 0; j < size; j += 2) {
        		convertedSignal[j] = (q15_t)(rawInput[i]);
        		convertedSignal[j+1] = (q15_t)(rawInput[i]);
        		i++;
        	}
    	}

    	if (i2s_send_flag == 1) {
        	for (int j = size; j < size*2; j += 2) {
        		convertedSignal[j] = (q15_t)(rawInput[i]);
        		convertedSignal[j+1] = (q15_t)(rawInput[i]);
        		i++;
        	}
    	}
}
void apply_highpass_filter(q15_t *input, q15_t *output, uint32_t blockSize) {
    arm_biquad_cascade_df1_q15(&highPassFilter, input, output, blockSize);
}
void init_highpass_filter() {
    arm_biquad_cascade_df1_init_q15(&highPassFilter, 1, highPassCoeffsQ15, highPassState, 0);
}
void rfft(q15_t *inputSignal, q15_t *fftOutput, q15_t *magnitudeSpectrum) {

	arm_rfft_q15(&rfft_instance, inputSignal, fftOutput); //rfft buffer izgleda jako cudno tho, DC, Nyquist, real1, imag1,
	offset = fftOutput[0];
	float result;
	for (int i = 0; i < 256; i++) {
		result = (float)(fftOutput[i+2]*fftOutput[i+2] + fftOutput[i+3]*fftOutput[i+3]);
		magnitudeSpectrum[i] = (q15_t)sqrt(result);
	}

 //ovo je za magnitudes, mora biti /2 jer je simetrično, nyquistov dijagram iz automatskog samo poz frekv
}
void peak_values_fft(q15_t *magnitudeSpectrum) {
	for (int i = 0; i < FFT_SIZE/2; i++) {
		if (magnitudeSpectrum[i] > peakVal) {
			peakVal = magnitudeSpectrum[i];
			peakHz = (uint16_t)(i * SAMPLE_FREQ / (float)FFT_SIZE);
		}
	}
}



void callUart(q15_t *input) {
	  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)input, sizeof(input));
	  HAL_I2S_Transmit_DMA(&hi2s3, input, ADC_SIZE*2);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc1) {
	adc_half_flag = 1;
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
	adc_done_flag = 1;
	if (call_uart_once == 1) {
		flag_uart =  1;
	}
	call_uart_once = 0;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI3) {
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

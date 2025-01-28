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
q15_t soft_clip(q15_t sample);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SIZE 1024
#define NUM_TAPS 64 //ZA FILTER KOLIKO JE IDEALAN
#define SAMPLE_FREQ 45455
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

//Signals
uint16_t adc_signal[ADC_SIZE];
q15_t conv_signal[ADC_SIZE/2];
q15_t output_signal[ADC_SIZE/2];
q15_t filtered_signal[ADC_SIZE/2];
q15_t eq_signal[ADC_SIZE/2];
q15_t finish_signal[ADC_SIZE/2];
q15_t i2s_signal[ADC_SIZE*2];
q15_t dummy[ADC_SIZE/2];
q15_t dummy_out[ADC_SIZE/2];


//FFT signals
q15_t fftOutput[FFT_SIZE];
q15_t magnitudeSpectrum[FFT_SIZE/2];
q15_t peakVal = 0;
uint16_t peakHz;


//Flags
volatile uint8_t fx_ready = 0;
volatile uint8_t adc_half_flag = 0;
volatile uint8_t adc_done_flag = 0;
volatile uint8_t conv_flag = 0;
volatile uint8_t i2s_send_flag = 0;
volatile uint8_t flag_uart = 0;
volatile uint8_t call_uart_once = 1;
volatile uint8_t q15_conv_flag = 0;
volatile uint32_t last_systick = 0;

//FFT
arm_rfft_instance_q15 rfftInstance;


//256 tapni FIR LOWPASS
arm_fir_instance_q15 S;
q15_t firCoeffsQ15[NUM_TAPS];
q15_t firStateQ15[NUM_TAPS + BLOCK_SIZE - 1]; // FIR state buffer
//float firCoeffs[NUM_TAPS] = {
//    -0.0000, -0.0001, -0.0002, -0.0002, -0.0002, -0.0001, -0.0000, 0.0001, 0.0002, 0.0002, 0.0002, 0.0002, 0.0001, -0.0001, -0.0002, -0.0003,
//    -0.0003, -0.0003, -0.0001, 0.0000, 0.0002, 0.0004, 0.0004, 0.0004, 0.0003, 0.0000, -0.0002, -0.0004, -0.0006, -0.0006, -0.0005, -0.0002,
//    0.0001, 0.0005, 0.0007, 0.0008, 0.0007, 0.0004, -0.0000, -0.0005, -0.0009, -0.0011, -0.0010, -0.0007, -0.0002, 0.0004, 0.0010, 0.0014,
//    0.0014, 0.0012, 0.0006, -0.0002, -0.0010, -0.0016, -0.0019, -0.0017, -0.0011, -0.0002, 0.0009, 0.0018, 0.0023, 0.0023, 0.0018, 0.0007,
//    -0.0006, -0.0018, -0.0027, -0.0030, -0.0026, -0.0015, 0.0001, 0.0017, 0.0031, 0.0038, 0.0036, 0.0025, 0.0007, -0.0014, -0.0033, -0.0045,
//    -0.0047, -0.0038, -0.0019, 0.0007, 0.0032, 0.0052, 0.0060, 0.0054, 0.0034, 0.0004, -0.0029, -0.0057, -0.0074, -0.0074, -0.0055, -0.0022,
//    0.0020, 0.0060, 0.0089, 0.0099, 0.0084, 0.0048, -0.0004, -0.0060, -0.0107, -0.0132, -0.0126, -0.0088, -0.0024, 0.0054, 0.0128, 0.0180,
//    0.0193, 0.0159, 0.0079, -0.0035, -0.0161, -0.0269, -0.0331, -0.0319, -0.0218, -0.0025, 0.0245, 0.0566, 0.0899, 0.1199, 0.1427, 0.1550,
//    0.1550, 0.1427, 0.1199, 0.0899, 0.0566, 0.0245, -0.0025, -0.0218, -0.0319, -0.0331, -0.0269, -0.0161, -0.0035, 0.0079, 0.0159, 0.0193,
//    0.0180, 0.0128, 0.0054, -0.0024, -0.0088, -0.0126, -0.0132, -0.0107, -0.0060, -0.0004, 0.0048, 0.0084, 0.0099, 0.0089, 0.0060, 0.0020,
//    -0.0022, -0.0055, -0.0074, -0.0074, -0.0057, -0.0029, 0.0004, 0.0034, 0.0054, 0.0060, 0.0052, 0.0032, 0.0007, -0.0019, -0.0038, -0.0047,
//    -0.0045, -0.0033, -0.0014, 0.0007, 0.0025, 0.0036, 0.0038, 0.0031, 0.0017, 0.0001, -0.0015, -0.0026, -0.0030, -0.0027, -0.0018, -0.0006,
//    0.0007, 0.0018, 0.0023, 0.0023, 0.0018, 0.0009, -0.0002, -0.0011, -0.0017, -0.0019, -0.0016, -0.0010, -0.0002, 0.0006, 0.0012, 0.0014,
//    0.0014, 0.0010, 0.0004, -0.0002, -0.0007, -0.0010, -0.0011, -0.0009, -0.0005, -0.0000, 0.0004, 0.0007, 0.0008, 0.0007, 0.0005, 0.0001,
//    -0.0002, -0.0005, -0.0006, -0.0006, -0.0004, -0.0002, 0.0000, 0.0003, 0.0004, 0.0004, 0.0004, 0.0002, 0.0000, -0.0001, -0.0003, -0.0003,
//    -0.0003, -0.0002, -0.0001, 0.0001, 0.0002, 0.0002, 0.0002, 0.0002, 0.0001, -0.0000, -0.0001, -0.0002, -0.0002, -0.0002, -0.0001, -0.0000
//};
float firCoeffs[NUM_TAPS] = {
    0.0002, 0.0006, 0.0009, 0.0011, 0.0011, 0.0007, -0.0001, -0.0012, -0.0025, -0.0036, -0.0039, -0.0031, -0.0009, 0.0023, 0.0061, 0.0094, 0.0110,
    0.0097, 0.0052, -0.0025, -0.0119, -0.0210, -0.0270, -0.0271, -0.0192, -0.0023, 0.0229, 0.0540, 0.0872, 0.1179, 0.1415, 0.1544, 0.1544, 0.1415,
    0.1179, 0.0872, 0.0540, 0.0229, -0.0023, -0.0192, -0.0271, -0.0270, -0.0210, -0.0119, -0.0025, 0.0052, 0.0097, 0.0110, 0.0094, 0.0061, 0.0023,
    -0.0009, -0.0031, -0.0039, -0.0036, -0.0025, -0.0012, -0.0001, 0.0007, 0.0011, 0.0011, 0.0009, 0.0006, 0.0002
};



//IIR HIGHPASS
float32_t highPassCoeffs[5] = {0.500037, -1.0, 0.500037, 0.999892, -0.492886};
q15_t highPassState[4] = {0};
arm_biquad_casd_df1_inst_q15 highPassFilter;
q15_t highPassCoeffsQ15[5];
float32_t hann_window[ADC_SIZE/2];

float32_t lowPassCoeffs[5] = {0.044927, 0.089853, 0.044927, 1.000000, -0.408279};
q15_t lowPassState[4] = {0};
arm_biquad_casd_df1_inst_q15 lowPassFilter;
q15_t lowPassCoeffsQ15[5];


//IIR


float32_t band1_coeffs[5] = {0.000039, 0.000000, -0.000077, 0.673938, -1.000000};
float32_t band2_coeffs[5] = {0.000200, 0.000000, -0.000400, 0.683758, -1.000000};
float32_t band3_coeffs[5] = {0.001178, 0.000000, -0.002356, 0.711083, -1.000000};
float32_t band4_coeffs[5] = {0.003599, 0.000000, -0.007199, 0.755061, -1.000000};
float32_t band5_coeffs[5] = {0.010781, 0.000000, -0.021561, 0.828920, -1.000000};


q15_t band1_coeffs_q15[5];
q15_t band2_coeffs_q15[5];
q15_t band3_coeffs_q15[5];
q15_t band4_coeffs_q15[5];
q15_t band5_coeffs_q15[5];

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
//  MX_USB_HOST_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
// TESTING ZONE

//  q15_t test = -2022;   // Skaliraj 2022 na Q15 format
//  float izbor = 0.1;
//  q15_t multiplier = (q15_t)(0x7FFF * izbor);         // Q15 format za 1.0
//  q15_t rezultat = 0;
//  arm_mult_q15(&test, &multiplier, &rezultat, 1);
  arm_rfft_init_q15(&rfftInstance, FFT_SIZE, 0, 1);
  configAudio();


  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, adc_signal, ADC_SIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  arm_rfft_init_q15(&rfftInstance, FFT_SIZE, 0, 1);
//	  init_iir_filter();
	  if (adc_half_flag == 1) {
    	adc_half_flag = 0;
    	conv_flag = 0;
    	last_systick = HAL_GetTick();

    	init_iir_filter();
    	init_fir_filter();
    	convert_to_q15(adc_signal, conv_signal, ADC_SIZE);

		fir_filter(conv_signal, filtered_signal, ADC_SIZE/2, BLOCK_SIZE);

    	iir_filter(filtered_signal, output_signal, ADC_SIZE/2);


//		iir_filter_lowpass(filtered_signal, output_signal, ADC_SIZE/2);

		process_equalizer(output_signal, eq_signal, ADC_SIZE/2);

    	last_systick = HAL_GetTick();
//		rfft(finish_signal, fftOutput, magnitudeSpectrum);
    	last_systick = HAL_GetTick();
    	last_systick = HAL_GetTick();
//		peak_values_fft(magnitudeSpectrum); // Look for unexpected peaks


		i2s_send_flag = 0; //DA KRENE PRVU POLOVICU

		convert_to_i2s(eq_signal, i2s_signal, ADC_SIZE);

    }

    if (adc_done_flag == 1) {
    	adc_done_flag = 0;
    	conv_flag = 1;
    	init_iir_filter();
    	init_fir_filter();
    	convert_to_q15(adc_signal, conv_signal, ADC_SIZE);
    	fir_filter(conv_signal, filtered_signal, ADC_SIZE/2, BLOCK_SIZE);
    	iir_filter(filtered_signal, output_signal, ADC_SIZE/2);


//		iir_filter_lowpass(filtered_signal, output_signal, ADC_SIZE/2);
//		fir_filter(filtered_signal, output_signal, ADC_SIZE/2, BLOCK_SIZE);

		process_equalizer(output_signal, eq_signal, ADC_SIZE/2);

		i2s_send_flag = 1; //DA KRENE DRUGU POLOVICU
		convert_to_i2s(eq_signal, i2s_signal, ADC_SIZE);
		if (flag_uart == 1) {
			callUart(i2s_signal);
		}
		flag_uart = 0;

    }

//		process_equalizer(filtered_signal, output_signal, ADC_SIZE);
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

//ECHO EFEKT
void echo_effect(uint16_t *buffer, uint16_t *outputBuffer, int size, float echo_strength, int delay) {
    static uint16_t echo_buffer[ADC_SIZE * 2] = {0}; // Povećan buffer za "delay"
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


void convert_to_q15(uint16_t *rawInput, q15_t *convertedSignal, int size) {
    int i;
    if (conv_flag == 0) {
        for (i = 0; i < size / 2; i++) {
            // Convert and saturate to Q15

            convertedSignal[i] = (q15_t)__SSAT(((rawInput[i] - 2048) * 8), 16);
            if (convertedSignal[i] > 7800*2) convertedSignal[i] = 7800*2;
            if (convertedSignal[i] < -7800*2) convertedSignal[i] = -7800*2;
        }
    }
    if (conv_flag == 1) {
        for (i = size / 2; i < size; i++) {
            // Convert and saturate to Q15

            convertedSignal[i] = (q15_t)__SSAT(((rawInput[i] - 2048) * 8), 16);
            if (convertedSignal[i] > 7000*2) convertedSignal[i] = 7000*2;
            if (convertedSignal[i] < -7000*2) convertedSignal[i] = -7000*2;
        }
    }
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



void callUart(q15_t *input) {
	  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)input, sizeof(input));
	  HAL_I2S_Transmit_DMA(&hi2s3, input, ADC_SIZE*2);
}

void init_fir_filter() {
	arm_float_to_q15(firCoeffs, firCoeffsQ15, NUM_TAPS);
    memset(firStateQ15, 0, sizeof(firStateQ15));
    arm_fir_init_q15(&S, NUM_TAPS, firCoeffsQ15, firStateQ15, BLOCK_SIZE);
}

void fir_filter(q15_t *input, q15_t *output, uint16_t length, uint16_t block_size) {
    for (int i = 0; i < length; i += block_size) {
        arm_fir_q15(&S, &input[i], &output[i], block_size);
    }
}


void init_iir_filter() {
	arm_float_to_q15(highPassCoeffs, highPassCoeffsQ15, 5);
	arm_biquad_cascade_df1_init_q15(&highPassFilter, 1, highPassCoeffsQ15, highPassState, 0);
	arm_float_to_q15(lowPassCoeffs, lowPassCoeffsQ15, 5);
	arm_biquad_cascade_df1_init_q15(&lowPassFilter, 1, lowPassCoeffsQ15, lowPassState, 0);
}

void iir_filter(q15_t *input, q15_t *output, uint32_t blockSize) {
	arm_biquad_cascade_df1_q15(&highPassFilter, input, output, blockSize);
}
void iir_filter_lowpass(q15_t *input, q15_t *output, uint32_t blockSize) {
	arm_biquad_cascade_df1_q15(&lowPassFilter, input, output, blockSize);
}


void rfft(q15_t *inputSignal, q15_t *fftOutput, q15_t *magnitudeSpectrum) {
    arm_rfft_q15(&rfftInstance, inputSignal, fftOutput); //rfft buffer izgleda jako cudno tho, DC, Nyquist, real1, imag1,
    arm_cmplx_mag_q15(fftOutput, magnitudeSpectrum, FFT_SIZE / 2); //ovo je za magnitudes, mora biti /2 jer je simetrično, nyquistov dijagram iz automatskog samo poz frekv
}
void peak_values_fft(q15_t *magnitudeSpectrum) {
	for (int i = 0; i < FFT_SIZE/2; i++) {
		if (magnitudeSpectrum[i] > peakVal) {
			peakVal = magnitudeSpectrum[i];
			peakHz = (uint16_t)(i * SAMPLE_FREQ / (float)FFT_SIZE);
		}
	}
}



void process_equalizer(q15_t *input, q15_t *output, uint32_t blockSize) {
    q15_t bandOutputs[5][blockSize];   // Temporary storage for each band output
    q15_t gainsQ15[5];
    float32_t gains[5] = {0.2, 0.8, 1.0, 1.0, 0.4};

    arm_float_to_q15(gains, gainsQ15, 5);

    arm_float_to_q15(band1_coeffs, band1_coeffs_q15, 5);
    arm_float_to_q15(band2_coeffs, band2_coeffs_q15, 5);
    arm_float_to_q15(band3_coeffs, band3_coeffs_q15, 5);
    arm_float_to_q15(band4_coeffs, band4_coeffs_q15, 5);
    arm_float_to_q15(band5_coeffs, band5_coeffs_q15, 5);

    arm_biquad_cascade_df1_init_q15(&eqBands[0], 1, band1_coeffs, band1_state, 0);
    arm_biquad_cascade_df1_init_q15(&eqBands[1], 1, band2_coeffs, band2_state, 0);
    arm_biquad_cascade_df1_init_q15(&eqBands[2], 1, band3_coeffs, band3_state, 0);
    arm_biquad_cascade_df1_init_q15(&eqBands[3], 1, band4_coeffs, band4_state, 0);
    arm_biquad_cascade_df1_init_q15(&eqBands[4], 1, band5_coeffs, band5_state, 0);

    // Apply each band filter to the input signal
    for (int i = 0; i < 5; i++) {
        arm_biquad_cascade_df1_q15(&eqBands[i], input, bandOutputs[i], blockSize);
    }

    for (uint32_t n = 0; n < blockSize; n++) {
           int32_t sum = 0;

           for (int b = 0; b < 5; b++) {
               q15_t scaledSample;
               arm_mult_q15(&bandOutputs[b][n], &gainsQ15[b], &scaledSample, 1); // Scale band output
               sum += scaledSample; // Sum all bands
           }

           // Saturate to Q15 range
           output[n] = (q15_t)__SSAT(sum, 16);
       }
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

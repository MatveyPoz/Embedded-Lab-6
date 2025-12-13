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
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIN_FREQ		1000
#define SAMPLING_RATE	48000
#define BUFFER_LENGTH	SAMPLING_RATE / SIN_FREQ

#define WAVETABLE_SIZE	1024

#define AMPLITUDE		32767

#define ZERO_OFFSET		32768

//#define DMA_BUFFER_SIZE (2 * WAVETABLE_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

/* USER CODE BEGIN PV */
uint16_t SineWaveTable[WAVETABLE_SIZE];
uint16_t dataI2S[DMA_BUFFER_SIZE];

typedef struct {
	uint16_t frequency;
	uint16_t duration_ms;
} RTTTLNote_t;

RTTTLNote_t melody[] = {
		{988, 353}, // B5
		{0, 1},
		{988, 353}, // B5
		{0, 1},
		{988, 353}, // B5
		{0, 353},   // Pause

		{988, 353}, // B5
		{0, 1},
		{988, 353},
		{0, 1},
		{988, 353},
		{0, 353},   // Pause

		{988, 353},
		{1175, 353},
		{784, 530},
		{880, 176},
		{988, 1059},
		{0, 176},

		{1047, 353},
		{0, 1},
		{1047, 353},
		{0, 1},
		{1047, 530},
		{0, 1},
		{1047, 176},
		{0, 1},
		{1047, 353},
		{988, 353},
		{0, 1},
		{988, 353},
		{0, 1},
		{988, 176},
		{0, 1},
		{988, 176},
		{0, 1},
		{988, 353},

		{880, 353},
		{0, 1},
		{880, 353},
		{0, 1},
		{988, 353},
		{880, 706},
		{1175, 706},

		{0, 500},		// Pause

		{0, 0}			// End
};
#define MELODY_LENGTH (sizeof(melody) / sizeof(RTTTLNote_t))

volatile uint32_t current_note_index = 0;
volatile uint32_t note_time_remaining_ms = 0;
volatile double current_phase = 0.0;
uint32_t last_irq_time = 0;

volatile uint8_t melody_repeats = 3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
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
  cs43l22_init();
  /* USER CODE BEGIN 2 */
  GenerateSineWaveTable();

  note_time_remaining_ms = melody[0].duration_ms;
  last_irq_time = HAL_GetTick();

  FillDMABuffer(melody[0].frequency, 0);
  FillDMABuffer(melody[0].frequency, WAVETABLE_SIZE);

  //HAL_I2S_Transmit_DMA(&hi2s3, dataI2S, DMA_BUFFER_SIZE);
  cs43l22_play(dataI2S, DMA_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS43L22_RESET_GPIO_Port, CS43L22_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS43L22_RESET_Pin */
  GPIO_InitStruct.Pin = CS43L22_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS43L22_RESET_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void GenerateSineWaveTable(void)
{
	for (int i = 0; i < WAVETABLE_SIZE; i++)
	{
		double sample = sin((double)i / WAVETABLE_SIZE * 2.0 * M_PI);
		SineWaveTable[i] = (uint16_t)(ZERO_OFFSET + AMPLITUDE * sample);
	}
}

void FillDMABuffer(uint16_t target_frequency, uint32_t offset)
{
	double increment = (double)WAVETABLE_SIZE * target_frequency / (double)SAMPLING_RATE;

	if (target_frequency == 0) increment = 0;

	uint32_t half_size = WAVETABLE_SIZE;

	for (int i = 0; i < half_size / 2; i++)
	{
		uint16_t sample;

		if (target_frequency == 0)
		{
			sample = ZERO_OFFSET;
		}
		else
		{
			int index = (int)current_phase;
			sample = SineWaveTable[index];
		}

		dataI2S[offset + i * 2] = sample;
		dataI2S[offset + i * 2 + 1] = sample;

		current_phase += increment;

		if (target_frequency != 0){
			current_phase += increment;
			if(current_phase >= WAVETABLE_SIZE) current_phase -= WAVETABLE_SIZE;
		}

	}
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	if (hi2s->Instance == SPI3)
	{
		uint32_t elapsed_time = HAL_GetTick() - last_irq_time;
		note_time_remaining_ms = (note_time_remaining_ms > elapsed_time) ? (note_time_remaining_ms - elapsed_time) : 0;
		last_irq_time = HAL_GetTick();

		if (note_time_remaining_ms == 0) {
			if (current_note_index + 1 >= MELODY_LENGTH)
			{
				if (melody_repeats > 0) melody_repeats--;
				if (melody_repeats == 0)
				{
					HAL_GPIO_WritePin(GPIOD, CS43L22_RESET_Pin, GPIO_PIN_RESET);
					HAL_I2S_DMAStop(&hi2s3);
					current_note_index = 0;
					return;
				}
				current_note_index = 0;
			}
			else current_note_index++;
			//current_note_index = (current_note_index + 1) % MELODY_LENGTH;
			note_time_remaining_ms = melody[current_note_index].duration_ms;
		}

		FillDMABuffer(melody[current_note_index].frequency, 0);
	}
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s){
	if (hi2s->Instance == SPI3)
	{
		uint32_t elapsed_time = HAL_GetTick() - last_irq_time;
		note_time_remaining_ms = (note_time_remaining_ms > elapsed_time) ? (note_time_remaining_ms - elapsed_time) : 0;
		last_irq_time = HAL_GetTick();

		if (note_time_remaining_ms == 0) {
			if (current_note_index + 1 >= MELODY_LENGTH)
			{
				if (melody_repeats > 0) melody_repeats--;
				if (melody_repeats == 0)
				{
					HAL_GPIO_WritePin(GPIOD, CS43L22_RESET_Pin, GPIO_PIN_RESET);
					HAL_I2S_DMAStop(&hi2s3);
					current_note_index = 0;
					return;
				}
				current_note_index = 0;
			}
			else current_note_index++;
			//current_note_index = (current_note_index + 1) % MELODY_LENGTH;
			note_time_remaining_ms = melody[current_note_index].duration_ms;
		}

		FillDMABuffer(melody[current_note_index].frequency, DMA_BUFFER_SIZE / 2);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

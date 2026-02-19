/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "stdbool.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Enum WaveType is three supported waveform shapes.
typedef enum {
	WAVE_TRIANGLE = 0,
	WAVE_SAW = 1,
	WAVE_SINE = 2
}WaveType;

// EnumMode between fixed and temperature outputs
typedef enum {
  MODE_WAVE = 0,  // LED off; output fixed waveform.
  MODE_TEMP = 1  // LED on; waveform depends on temperature sensor.
} Mode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define Statements for Flow Control
#define ON  1
#define OFF  0

// Choose ON/OFF
#define BUTTON_LIGHT 	OFF

// Choose single wave + delay is automatically included
#define WAVE_SAWTOOTH_DEF	OFF
#define WAVE_TRIANGLE_DEF	OFF
#define WAVE_SINE_DEF		OFF

#if WAVE_SAWTOOTH_DEF || WAVE_TRIANGLE_DEF || WAVE_SINE_DEF
	#define WAVE_DELAY   	ON
#else
	#define WAVE_DELAY		OFF
#endif

// Choose ON/OFF
#define ADC_SAMPLE   	OFF

// Choose ON/OFF
#define PART4			ON



// Define Constants for Temperature Sensor

#define TS_CAL1_TEMP 	30.0f
#define TS_CAL2_TEMP 	130.0f

// 32 bit address pointing to 16 bit memory spaces
#define TS_CAL1 		*(uint16_t*)(uint32_t)0x1FFF75A8
#define TS_CAL2 		*(uint16_t*)(uint32_t)0x1FFF75CA
#define VREFINT_CAL 	*(uint16_t*)(uint32_t)0x1FFF75AA


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

/* USER CODE BEGIN PV */

// Wave Generation Variables
uint8_t 	sawtooth = 0;
uint8_t 	triangle = 0;
uint8_t 	sine = 0;
float 		radians = 0.0;
float 		radians_increment = (5.625*3.14)/180;
bool 		triangle_up = true;
bool 		filler = true;
uint8_t 	amplitude = 0; // used when generating temperature-dependant waves

// Temperature Sensor Variables
float 	    vrefRatio = 1.0; 	 // (VREF+/VREFINT)
float 	    temperature = 25.0;  // store actual temperature value
uint16_t    tempAdc = 0; 	     // store ADC output corresponding to temperature value
uint16_t 	vrefAdc	= 0;	  	 // store ADC output corresponding to voltage reference
float		calibTempAdc = 0; 	 // V_TEMP * (VREF+/VREFINT)
static uint32_t last_adc_sample_ms = 0; //
// static uint32_t current_time_ms = 0; // in button debounce section


// Global application state.
static Mode mode = MODE_WAVE;
static WaveType fixedWave = WAVE_TRIANGLE;  // Currently selected fixed waveform.
static WaveType tempWave  = WAVE_SINE;      // Base waveform for temp controlled mode.

// Button debounce state.
static uint8_t 	button_prev_state = 0;
static uint8_t 	button_curr_state = 0;
static uint32_t last_button_toggle_ms = 0;
static uint32_t current_time_ms = 0; // also used for ADC



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */

// User Function Prototypes
	// See User Code Area 4 for definitions
static void Sawtooth_Wave(uint8_t amplitude);
static void Triangle_Wave(void);
static void Sine_Wave(void);

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

	// Constant used for calculating temperature from ADC reading
	uint16_t ts_cal1_val = TS_CAL1; // FOR DEBUGGING
	uint16_t ts_cal2_val = TS_CAL2; // FOR DEBUGGING
	uint16_t vrefint_val = VREFINT_CAL;	// FOR DEBUGGING
	float tempConst = (float)(TS_CAL2_TEMP - TS_CAL1_TEMP) / (float)(TS_CAL2 - TS_CAL1);

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
  MX_DAC1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // Start DAC Channels 1-2
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if BUTTON_LIGHT == ON

	  //If button is pressed
	  if (!HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin)) {
		  // Turn LED ON
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  }
	  else {
		  // Turn LED off when button is not pressed
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  }

#endif

#if WAVE_SAWTOOTH_DEF == ON
	  // Sawtooth Wave Output to Speaker
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, sawtooth);
	  if (sawtooth < 252) {
		  sawtooth += 4;
	  }
	  else {
		  sawtooth=0;
	  }
#endif

#if WAVE_TRIANGLE_DEF == ON
	  // Triangle Wave Output to Speaker
	  //DAC change value is double of sawtooth so that they have the same period
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, triangle);
	  //Ascending section of triangle wave
	  if(triangle_up==true){
		  if (triangle < 248) {
			triangle += 8;
		  }
		  else {
			triangle_up=false;
		  }
	  }
	  //Descending section of triangle wave
	  else{
		  if (triangle > 0) {
			  triangle -= 8;
		  } else {
			  triangle_up=true;
		  }
	  }
#endif

#if WAVE_SINE_DEF == ON
	  //Sine Wave Output to Speaker
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sine);
	  if (radians < 6.27) {
	  		  radians += radians_increment;
	  	  } else {
	  		  radians=0;
	  	  }
	  sine = roundf(127.0 * (1.0 + arm_sin_f32(radians)));

#endif

#if WAVE_DELAY == ON
	  //Loop to delay time between increments (for all wave generation)
	  for(uint8_t i = 0; i<127; i++) // iterator max (255 -> uint8)
	  {
		  //Filler logic to stop compiler from deleting this loop
		  if(filler==true){
			  filler = false;
		  }
		  else{
			  filler = true;
		  }
	  }
	  //HAL_Delay(1); // keep commented

#endif

#if ADC_SAMPLE == ON


	// Poll Temperature Sensor (Injected)
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedPollForConversion(&hadc1, HAL_MAX_DELAY);
	tempAdc = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
	HAL_ADCEx_InjectedStop(&hadc1);

	// Poll VREFINT (Non-Injected)
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	vrefAdc = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// Calculate (VREF+/VREFINT), or default to 1
	if (vrefAdc != 0) {
		vrefRatio = (float)VREFINT_CAL / (float)vrefAdc;
	}
	else {
		vrefRatio = 1;
	}

	// Calculate temperature with all scaling applied
	float calibAdc = (float)tempAdc * vrefRatio;
	temperature = tempConst * (calibAdc - (float)TS_CAL1) + TS_CAL1_TEMP;

	// Wait ~200ms
	HAL_Delay(200);

#endif

#if PART4 == ON

// Check if button is pressed with debouncing
	button_curr_state = !HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin);

	// If button experiences state change
	if (button_curr_state && !button_prev_state) {

		current_time_ms = HAL_GetTick();

		// && Enough time has elapsed
		if ((current_time_ms - last_button_toggle_ms) > 200) {

			last_button_toggle_ms = current_time_ms;

			// Then change mode
			if (mode == MODE_WAVE) {

				mode = MODE_TEMP;
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			} // if (mode == MODE_WAVE)
			else {

				mode = MODE_WAVE;
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				fixedWave = (fixedWave + 1) % 3; // keep range within 0-2 enum
			} // else(mode == MODE_TEMP)

		} // if ((current_time_ms - last_button_toggle_ms) > 200)

	} // if (button_current && !button_prev_state)

// Update button state
	button_prev_state = button_curr_state;

// Send value to DAC based on mode

	// Temperature Mode
	if (mode == MODE_TEMP) {

		// Sample temperature
		current_time_ms = HAL_GetTick();


		if ((current_time_ms - last_adc_sample_ms) > 200) {

			// Update time variables
			last_adc_sample_ms = current_time_ms;

			// Poll Temperature Sensor (Injected)
			HAL_ADCEx_InjectedStart(&hadc1);
			HAL_ADCEx_InjectedPollForConversion(&hadc1, HAL_MAX_DELAY);
			tempAdc = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
			HAL_ADCEx_InjectedStop(&hadc1);

			// Poll VREFINT (Non-Injected)
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			vrefAdc = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			// Calculate (VREF+/VREFINT), or default to 1
			if (vrefAdc != 0) {
				vrefRatio = (float)VREFINT_CAL / (float)vrefAdc;
			} // if (vrefAdc != 0)
			else {
				vrefRatio = 1;
			} // else

			// Calculate temperature with all scaling applied
			float calibAdc = (float)tempAdc * vrefRatio;
			temperature = tempConst * (calibAdc - (float)TS_CAL1) + TS_CAL1_TEMP;

			// Map temperature to wave amplitude (20 -40 C -> 0-255)
				// x = temperature (min(20), max(40))
				// y = amplitude (min(0), max(255))
				// y = [(x - xmin)/(xmax - xmin)] *  (ymax - ymin) + ymin

			amplitude = (uint8_t)((temperature - 20.0f) * 12.75f);

			if (amplitude > 255) {
				amplitude = 255;
			}
			else if (amplitude < 0) {
				amplitude = 0;
			}

		} // if ((current_time_ms - last_adc_sample_ms) > 200)

		// Send value to DAC (every loop iteration)
		Sawtooth_Wave(amplitude);

	} // if (mode == MODE_TEMP)

	// Wave Mode
	else if (mode == MODE_WAVE) {

		switch(fixedWave) {
		case WAVE_SAW:
			Sawtooth_Wave(255); // use full amplitude
			break;
		case WAVE_TRIANGLE:
			Triangle_Wave();
			break;
		case WAVE_SINE:
			Sine_Wave();
			break;

		} // switch(fixedWave)

	} // else if (mode == MODE_WAVE)


// Timing Delay between DAC Updates

	for(uint8_t i = 0; i<127; i++) // iterator max (255 -> uint8)
		{
		//Filler logic to stop compiler from deleting this loop
		if(filler==true){
			filler = false;
		} // if
		else{
			filler = true;
		} // else
	  } // for

#endif // PART 4


  } // while(1)


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc1);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_TEMPSENSOR;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// User Defined Functions

static void Sawtooth_Wave(uint8_t amplitude) {

	// make sure ramp doesn't go above on first call
	if (amplitude == 255) {
		amplitude = 252;
	}

	  // Sawtooth Wave Output to Speaker
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, sawtooth);
	  if (sawtooth < amplitude) {
		  sawtooth += 4;
	  }
	  else {
		  sawtooth=0;
	  }


}

static void Triangle_Wave(void) {

  // Triangle Wave Output to Speaker
  //DAC change value is double of sawtooth so that they have the same period
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, triangle);
  //Ascending section of triangle wave
  if(triangle_up==true){
	  if (triangle < 248) {
		triangle += 8;
	  }
	  else {
		triangle_up=false;
	  }
  }
  //Descending section of triangle wave
  else{
	  if (triangle > 0) {
		  triangle -= 8;
	  } else {
		  triangle_up=true;
	  }
  }

	// put code here to publish 1 DAC value for a triangle wave

}

static void Sine_Wave(void) {

  //Sine Wave Output to Speaker
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sine);
  if (radians < 6.27) {
		  radians += radians_increment;
	  } else {
		  radians=0;
	  }
  sine = roundf(127.0 * (1.0 + arm_sin_f32(radians)));

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

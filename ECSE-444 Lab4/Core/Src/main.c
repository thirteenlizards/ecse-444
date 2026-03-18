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
#include "octospi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_qspi.h"
#include "stm32l4s5i_iot01.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l4xx_it.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ifdef Defines
#define Part PART_1
#define PART_1 1
#define PART_2 2
#define PART_3 3
#define PART_4 4

// Pin Defines
#define BUTTON_PIN GPIO_PIN_13
#define BUTTON_PORT GPIOC

// Number of Samples
#define NUM_SAMPLES 100

// Number of Sensors
#define NUM_SENSORS 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Create data structure to store sensor data
char uartBuf[100];
float tempSample, pressureSample;
int16_t magSample[3], accelSample[3];
volatile uint8_t sensorState;
enum {Temperature, Pressure, Magneto, Accelero, All};

// Create data structure to store many samples of the data
float tempData[NUM_SAMPLES];
float pressureData[NUM_SAMPLES];
int16_t magData[NUM_SAMPLES][3];
int16_t accelData[NUM_SAMPLES][3];

// Counter for number of samples taken from each sensor
uint16_t sampleCount[NUM_SENSORS] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// User Functions

/**
 * @name Uart_Print_Sensor
 *
 * Reads the sensor indicated by the global @p sensor_toggle and sends a
 * human-readable string to the serial terminal via @p huart1.
 *
 * @param[in] Global: sensor_toggle (enum) Selects the active sensor to poll.
 * @param[in,out] Global: huart1 (handle) The UART interface for transmission.
 */

void Uart_Print_Sensor(){

	char uartBuffer[64];


	// Choose what data to TX
	switch (sensorState){
		case(Temperature):{
			tempSample = BSP_TSENSOR_ReadTemp();
			int len = snprintf(uartBuffer, sizeof(uartBuffer), "Temperature: %.2f \r\n", tempSample);
			HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Pressure):{
			  pressureSample = 	BSP_PSENSOR_ReadPressure();
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Pressure: %.2f \r\n", pressureSample);
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Magneto):{
			  int16_t magSample[3];
			  BSP_MAGNETO_GetXYZ(magSample);
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Magneto: X: %d, Y: %d, Z: %d\r\n",
			                     magSample[0], magSample[1], magSample[2]);
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Accelero):{
			  int16_t accData[3];
			  BSP_ACCELERO_AccGetXYZ(accData);
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Accelero: X: %d, Y: %d, Z: %d\r\n",
					  accData[0], accData[1], accData[2]);
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
	} // switch

} // Uart_Print_Sensor



void Read_Sensor(){

	// Choose what data to TX
	switch (sensorState){
		case(Temperature):{
			tempSample = BSP_TSENSOR_ReadTemp(); // get sample from sensor
			tempData[sampleCount[Temperature]] = tempSample; // store sensor sample in data array at count index
			break;
		}
		case(Pressure):{
			pressureSample = 	BSP_PSENSOR_ReadPressure(); // get sample from sensor
			pressureData[sampleCount[Pressure]] = pressureSample; // store sensor sample in data array at count index
			break;
		}
		case(Magneto):{
			  BSP_MAGNETO_GetXYZ(magSample); // get sample from sensor

			  for (uint8_t i = 0; i < 3; i++) {
				  magData[sampleCount[Magneto]][i] = magSample[i]; // store sensor sample in data array at count index
			  }
			break;
		}
		case(Accelero):{
			  BSP_ACCELERO_AccGetXYZ(accelSample); // get sample from sensor

			  for (uint8_t i = 0; i < 3; i++) {
				  accelData[sampleCount[Accelero]][i] = accelSample[i]; // store sensor sample in data array at count index
			  }
			break;
		}
	} // switch

	// Increment sampleCount at polled sensor index
	sampleCount[sensorState]++;

	// If filled, reset buffer back to 0 and next iteration will start overwriting
	if (sampleCount[sensorState] >= NUM_SAMPLES) {
		sampleCount[sensorState] = 0;
	}


} // void Read_Sensor(){

//Button Interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

if (GPIO_Pin == BUTTON_PIN) {
	// iterate to next sensor enum each button press
	sensorState = (sensorState + 1) % 4;
	Uart_Print_Sensor();
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
  MX_OCTOSPI1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  BSP_TSENSOR_Init();
  BSP_MAGNETO_Init();
  BSP_ACCELERO_Init();
  BSP_PSENSOR_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*
	   * Initial Testing for Part 1 Without Button
#if Part == PART_1

	  // Get temperature data and UART TX
	  tempSample = BSP_TSENSOR_ReadTemp();
	  char output[50];
	  int len = snprintf(output, sizeof(output), "Temperature: %.2f \r\n", tempSample);
	  HAL_UART_Transmit(&huart1, (uint8_t *)output, len, 100);
	  HAL_Delay(1000);


	  // Get magneto data and UART TX
	  int16_t magSample[3];
	  BSP_MAGNETO_GetXYZ(magSample);
	  char output1[64];
	  len = snprintf(output1, sizeof(output1), "Magneto: X: %d, Y: %d, Z: %d\r\n",
	                     magSample[0], magSample[1], magSample[2]);
	  HAL_UART_Transmit(&huart1, (uint8_t *)output1, len, 100);
	  HAL_Delay(1000);

	  // Get accelero data and UART TX
	  int16_t accData[3];
	  BSP_ACCELERO_AccGetXYZ(accData);
	  char output2[64];
	  len = snprintf(output2, sizeof(output2), "Accelero: X: %d, Y: %d, Z: %d\r\n",
			  accData[0], accData[1], accData[2]);
	  HAL_UART_Transmit(&huart1, (uint8_t *)output2, len, 100);
	  HAL_Delay(1000);

	  // Get pressure data and UART TX
	  pressure = 	BSP_PSENSOR_ReadPressure();
	  char output3[50];
	  len = snprintf(output3, sizeof(output3), "pressureSample: %.2f \r\n", pressureSample);
	  HAL_UART_Transmit(&huart1, (uint8_t *)output3, len, 100);
	  HAL_Delay(1000);

	  //#if Part == PART_1
#endif
*/


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

/* USER CODE BEGIN 4 */
int __io__putchar(int ch){
	HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

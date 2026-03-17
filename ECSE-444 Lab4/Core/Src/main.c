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
#define Part PART_1
#define PART_1 1
#define PART_2 2
#define PART_3 3
#define PART_4 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Create data structure to store sensor data
char uartBuf[100];
float tempData, pressureData;
int16_t magData[3], accelData[3];
volatile uint8_t sensorToggle;
enum {Temperature, Pressure, Magneto, Accelero, All};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// User Functions

void Uart_Print_Sensor(){

	char uartBuffer[64];

	// Choose what data to TX
	switch (sensorToggle){
		case(Temperature):{
			float temp = BSP_TSENSOR_ReadTemp();
			int len = snprintf(uartBuffer, sizeof(uartBuffer), "Temperature: %.2f \r\n", temp);
			HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Pressure):{
			  float pressure = 	BSP_PSENSOR_ReadPressure();
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Pressure: %.2f \r\n", pressure);
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Magneto):{
			  int16_t magData[3];
			  BSP_MAGNETO_GetXYZ(magData);
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Magneto: X: %d, Y: %d, Z: %d\r\n",
			                     magData[0], magData[1], magData[2]);
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

	// TX Data
	HAL_UART_Transmit(&huart1, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);

} // Uart_Print_Sensor

//Button Interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

if (GPIO_Pin == BUTTON_Pin) {
	// iterate to next sensor enum each button press
	sensorToggle = (sensorToggle + 1) % 4;
	Uart_Print_Sensor();
	HAL_Delay(1000); // TODO: remove delay
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

#if Part == PART_1

	  // Get temperature data and UART TX
	  float temp = BSP_TSENSOR_ReadTemp();
	  char output[50];
	  int len = snprintf(output, sizeof(output), "Temperature: %.2f \r\n", temp);
	  HAL_UART_Transmit(&huart1, (uint8_t *)output, len, 100);
	  HAL_Delay(1000);


	  // Get magneto data and UART TX
	  int16_t magData[3];
	  BSP_MAGNETO_GetXYZ(magData);
	  char output1[64];
	  len = snprintf(output1, sizeof(output1), "Magneto: X: %d, Y: %d, Z: %d\r\n",
	                     magData[0], magData[1], magData[2]);
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
	  float pressure = 	BSP_PSENSOR_ReadPressure();
	  char output3[50];
	  len = snprintf(output3, sizeof(output3), "Pressure: %.2f \r\n", pressure);
	  HAL_UART_Transmit(&huart1, (uint8_t *)output3, len, 100);
	  HAL_Delay(1000);

	  //#if Part == PART_1
#endif



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

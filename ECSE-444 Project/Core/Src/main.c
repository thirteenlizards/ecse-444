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
#include "i2c.h"
#include "octospi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// States of UART processing
typedef enum {
	Waiting,
	Receiving,
}UartState;

// Data structure to store incoming UART data
typedef struct {
	union {
		float wrench[6]; // wrench can be accessed as array or by name, i.e.
		struct {
			float force_x;  // WrenchPacket.force_x
			float force_y;
			float force_z;
			float torque_x;
			float torque_y;
			float torque_z;
		};
	};

	bool  isValid;			// WrenchPacket.isValid
}WrenchPacket;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_TIMEOUT 100
#define UART_TX_TIMEOUT 100
#define UART_RX_BUFFER_SIZE 128
#define UART_TX_BUFFER_SIZE 128
#define WRENCH_SIZE 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char uartRxBuffer[(uint8_t)UART_RX_BUFFER_SIZE]; // UART RX Buffer
char uartTxBuffer[(uint8_t)UART_TX_BUFFER_SIZE]; // UART RX Buffer
uint8_t uartByte; // single byte of received UART data
WrenchPacket wrenchPacket = {0}; // instance of wrench data
uint8_t byte = 0; // single UART byte

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
WrenchPacket UART_Parse_Wrench(uint8_t byte);
void Test_Uart_Processing();

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
  MX_OCTOSPI1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // UART write-back to test parsing of input strings
	  Test_Uart_Processing();

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

// User Functions


void Test_Uart_Processing() {
	  // If byte received:
	  if (HAL_UART_Receive(&huart1, &byte, sizeof(byte), (uint32_t)UART_RX_TIMEOUT)
		  == HAL_OK)
	  {
		  wrenchPacket = UART_Parse_Wrench(byte);

		  if (wrenchPacket.isValid) {



			  // Send back UART transmission
				uint8_t len = snprintf(uartTxBuffer, sizeof(uartTxBuffer), "%f, %f, %f, %f, %f, %f\r\n",
						wrenchPacket.force_x, wrenchPacket.force_y,
						wrenchPacket.force_z, wrenchPacket.torque_x,
						wrenchPacket.torque_y, wrenchPacket.torque_z);



			  HAL_UART_Transmit(
					  &huart1,						// specified UART module.
					  (uint8_t *)uartTxBuffer,		// pointer to RX data buffer
					  len,			// amount of data elements
					  (uint32_t)UART_TX_TIMEOUT);   // timeout duration
		  } // isValid

	  } // HAL_OK
}


/**
 * State machine to manage wrench packets over UART, call repeatedly to
 * process next byte.
 * @param byte 		next byte from uart
 * @return 			UART_Parse_Wrench struct (6 vals + flag)
 */
WrenchPacket UART_Parse_Wrench(uint8_t byte) {

	// Assume that incoming packets are in the format:
	// <force_x, force_y, force_z, torque_x, torque_y, torque_z>

	// Initialize state variable, set to Waiting in first iteration
	static UartState uartState = Waiting;

	// Increment up to WRENCH_SIZE = 6, set to 0 in first iteration
	static uint8_t wrenchIdx = 0;

	// Initialize wrenchPacket to 0 every time, this sets the isValid flag to FALSE
	WrenchPacket wrench = {0};

	switch(uartState) {

	case(Waiting):
			if (byte == '<') { // packet is starting
				uartState = Receiving; // set state
				wrenchIdx = 0;		   // set index
				memset(uartRxBuffer, 0, sizeof(uartRxBuffer)); // clear buffer
			} // if

			break;

	case(Receiving):
		if (byte == '>') { // packet is ending

			// End buffer location with null character
			uartRxBuffer[wrenchIdx] = '\0';

			// Parse the wrench values
			uint8_t numValues = sscanf(uartRxBuffer, "%f, %f, %f, %f, %f, %f",
					&wrench.force_x, &wrench.force_y,
					&wrench.force_z, &wrench.torque_x,
					&wrench.torque_y, &wrench.torque_z);

			if (numValues == (uint8_t)WRENCH_SIZE) {
				wrench.isValid = true;
			} // if

			// Return state to waiting
			uartState = Waiting;

		} // if

		else { // just a regular data byte
			if (wrenchIdx < (sizeof(uartRxBuffer)-1)) { // make sure didn't get fucked up packet
				uartRxBuffer[wrenchIdx++] = byte;
			}
			else { // packet was too long, you silly goose. you're so silly
				uartState = Waiting; // get kicked out. idiot.
			}
		} // else

			break;

		} // switch

	return(wrench);
} // WrenchPacket UART_Parse_Wrench(uint8_t byte)


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

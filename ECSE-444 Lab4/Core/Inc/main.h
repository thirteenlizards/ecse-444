/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
typedef enum {
    Temperature,
    Pressure,
    Magneto,
    Accelero,
    Statistics,
    StateCount
} SensorState;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Read_Sensor();
void Uart_Display_Statistics();
void Uart_Print_Sensor();
void Sensors_Write_to_Flash();
void Sensors_Read_from_Flash();
void Uart_Print_Sensor_State(SensorState state);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

// Pin Defines
#define BUTTON_PIN GPIO_PIN_13
#define BUTTON_PORT GPIOC

// Number of Samples
#define NUM_SAMPLES 100

// Number of Sensors
#define NUM_SENSORS 4

// Flash addresses for sensor data
#define FLASH_BLOCK_ADDR 		0x000000
#define TEMP_FLASH_ADDR			0x000000
#define PRESSURE_FLASH_ADDR  	0x001000
#define MAG_FLASH_ADDR			0x002000
#define ACCEL_FLASH_ADDR		0x003000


// Create data structure to store sensor data
extern char uartBuffer[256];
extern float tempSample, pressureSample;
extern int16_t magSample[3], accelSample[3];
extern volatile uint8_t sensorState;

// Create data structure to store samples from sensor
extern float tempData[NUM_SAMPLES];
extern float pressureData[NUM_SAMPLES];
extern int16_t magData[NUM_SAMPLES][3];
extern int16_t accelData[NUM_SAMPLES][3];

// Counter for number of samples taken from each sensor
extern uint16_t sampleCount[StateCount];

// Create data structure to store sensor data read from flash
extern float tempDataFlashRead[NUM_SAMPLES];
extern float pressureDataFlashRead[NUM_SAMPLES];
extern int16_t magDataFlashRead[NUM_SAMPLES][3];
extern int16_t accelDataFlashRead[NUM_SAMPLES][3];

extern volatile bool printDataFlag;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

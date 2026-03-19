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
  ******************************************************************************
	TODO:
	* make sure read and write in correct format from QSPI flash for all sensors (functions written, not yet tested) -> DONE
	*
	* calculate statistics for sensor values
	* function to display statistics for sensor values (modify Uart_Print_Sensor and sensorState logic)
	* make sure sequence of get data -> store in flash -> read from flash -> display all is WORKING
	* add OS and fuck everything up
	*
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
#define Part PART_35
#define PART_1 1
#define PART_2 2
#define PART_3 3
#define PART_35 5
#define PART_4 4

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


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Create data structure to store sensor data
char uartBuffer[100];
float tempSample, pressureSample;
int16_t magSample[3], accelSample[3];
volatile uint8_t sensorState;
enum {Temperature, Pressure, Magneto, Accelero, Statistics};

// Create data structure to store samples from sensor
float tempData[NUM_SAMPLES];
float pressureData[NUM_SAMPLES];
int16_t magData[NUM_SAMPLES][3];
int16_t accelData[NUM_SAMPLES][3];

// Counter for number of samples taken from each sensor
uint16_t sampleCount[NUM_SENSORS] = {0};

// Create data structure to store sensor data read from flash
float tempDataFlashRead[NUM_SAMPLES];
float pressureDataFlashRead[NUM_SAMPLES];
int16_t magDataFlashRead[NUM_SAMPLES][3];
int16_t accelDataFlashRead[NUM_SAMPLES][3];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Sensors_Read_from_Flash();
void Sensors_Write_to_Flash();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// User Functions

#if Part == PART_1
/**
 * @name Uart_Read_Print_Sensor
 *
 * Reads the sensor indicated by the global @p sensor_toggle and sends a
 * human-readable string to the serial terminal via @p huart1.
 *
 * @param[in] Global: sensorState (enum) Selects the active sensor to poll.
 * @param[in,out] Global: huart1 (handle) The UART interface for transmission.
 */

void Uart_Read_Print_Sensor(){


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
			  int16_t accelData[3];
			  BSP_ACCELERO_AccGetXYZ(accelData);
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Accelero: X: %d, Y: %d, Z: %d\r\n",
					  accelData[0], accelData[1], accelData[2]);
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
	} // switch

} // Uart_Read_Print_Sensor

#endif


/**
 * @name Read_Sensor
 *
 * Read all sensors
 *
 * @param[in] Global: sensorState (enum) Selects the active sensor to poll.
 */

void Read_Sensor(){

	// Choose what data to TX
	tempSample = BSP_TSENSOR_ReadTemp(); // get sample from sensor
	tempData[sampleCount[Temperature]] = tempSample; // store sensor sample in data array at count index

	pressureSample = 	BSP_PSENSOR_ReadPressure(); // get sample from sensor
	pressureData[sampleCount[Pressure]] = pressureSample; // store sensor sample in data array at count index

	BSP_MAGNETO_GetXYZ(magSample); // get sample from sensor

   for (uint8_t i = 0; i < 3; i++) {
	  magData[sampleCount[Magneto]][i] = magSample[i]; // store sensor sample in data array at count index
    } // for

	BSP_ACCELERO_AccGetXYZ(accelSample); // get sample from sensor

    for (uint8_t i = 0; i < 3; i++) {
	   accelData[sampleCount[Accelero]][i] = accelSample[i]; // store sensor sample in data array at count index
    } // for


	// Increment sampleCount at polled sensor index
    	// originially had seperate indices because took samples at different times adn now don't
	sampleCount[0]++;
	sampleCount[1]++;
	sampleCount[2]++;

	// If filled, reset buffer back to 0 and next iteration will start overwriting
	if (sampleCount[sensorState] > (NUM_SAMPLES - 1)) {

		// reset iterations back to 0
		sampleCount[0] = 0;
		sampleCount[1] = 0;
		sampleCount[2] = 0;

		// save sensor data to flash
		Sensors_Write_to_Flash();
	} // if

} // void Read_Sensor()

/**
 * @name Uart_Display_Statistics
 *
 * Calculate statistics from data stored in flash and display on serial terminal via @p huart1.
 *
 * @param[in] Global: sensorState (enum) Selects which sensor data to print
 * @param[in] Global: xSample: takes most recent sample from given sensor (DOES NOT GRAB FROM xData BUFFERS)
 */

void Uart_Display_Statistics(){

	// sources:
		// https://arm-software.github.io/CMSIS-DSP/main/group__VarianceExample.html


	// Pull all 4 buffers from flash
		// assume by the time a human has time to do the 4th button press, we have enough data in flash to display statistics
	Sensors_Read_from_Flash();

	// Calculate statistics
		// For each sensor, display: number of samples, sample mean, and sample variance

	float tempDataSum = 0;
	float pressureDataSum = 0;
	float magDataSum[3] = {0};
	float accelDataSum[3] = {0};

	float tempDataMean = 0;
	float pressureDataMean = 0;
	float magDataMean[3] = {0};
	float accelDataMean[3] = {0};

	float tempDataVariance = 0;
	float pressureDataVariance = 0;
	float magDataVariance[3] = {0};
	float accelDataVariance[3] = {0};


	// Calculate the sum
	for (uint8_t i = 0; i < NUM_SAMPLES; i++) {

		tempDataSum += tempDataFlashRead[i];
		pressureDataSum += pressureDataFlashRead[i];

		for (uint8_t j = 0; j < 3; j++) {
			magDataSum[j] += (float)magDataFlashRead[i][j];
			accelDataSum[j] += (float)accelDataFlashRead[i][j];

		} // for j
	} // for i


	// Calculate the mean
		//  x' = (x[0] + x[1] + ... + x[n-1]) / N
	tempDataMean = tempDataSum/(float)NUM_SAMPLES;
	pressureDataMean = pressureDataSum/100.0f;

	for (uint8_t j = 0; j < 3; j++) {
		magDataMean[j] = magDataSum[j]/(float)NUM_SAMPLES;
		accelDataMean[j] = accelDataSum[j]/(float)NUM_SAMPLES;
	} // for j


	// Calculate the variance
		// variance = ((x[0] - x') * (x[0] - x') + (x[1] - x') * (x[1] - x') + ... + * (x[n-1] - x') * (x[n-1] - x')) / (N)

	for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
		tempDataVariance = tempDataFlashRead[i] - tempDataMean;
		pressureDataVariance = pressureDataFlashRead[i] - pressureDataMean;

		for (uint8_t j = 0; j < 3; j++) {
			magDataVariance[j] = (float)magDataFlashRead[i][j] - magDataMean[j];
			accelDataVariance[j] = (float)accelDataFlashRead[i][j] - accelDataMean[j];
		} // for j
	} // for i


	// Display statistics

	//number of samples, sample mean, and sample variance

		// display number of samples
	uint8_t len = snprintf(uartBuffer, sizeof(uartBuffer), "NUMBER OF SAMPLES: %d/r/n/n", (uint8_t)NUM_SAMPLES);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);


		// display sample mean
	len = snprintf(uartBuffer, sizeof(uartBuffer), "TEMPERATURE MEAN: %f/r/n", tempDataMean);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

	len = snprintf(uartBuffer, sizeof(uartBuffer), "PRESSURE MEAN: %f/r/n", pressureDataMean);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

	len = snprintf(uartBuffer, sizeof(uartBuffer), "MAGNETO MEAN: X: %f, Y: %f, Z: %f\r\n",
			magDataMean[0], magDataMean[1], magDataMean[2]);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

	len = snprintf(uartBuffer, sizeof(uartBuffer), "ACCEL MEAN: X: %f, Y: %f, Z: %f\r\n\n",
			accelDataMean[0], accelDataMean[1], accelDataMean[2]);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

		// display sample variance
	len = snprintf(uartBuffer, sizeof(uartBuffer), "TEMPERATURE VARIANCE: %f/r/n", tempDataVariance);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

	len = snprintf(uartBuffer, sizeof(uartBuffer), "PRESSURE VARIANCE: %f/r/n", pressureDataVariance);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

	len = snprintf(uartBuffer, sizeof(uartBuffer), "MAGNETO VARIANCE: X: %f, Y: %f, Z: %f\r\n",
			magDataVariance[0], magDataVariance[1], magDataVariance[2]);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

	len = snprintf(uartBuffer, sizeof(uartBuffer), "ACCEL VARIANCE: X: %f, Y: %f, Z: %f\r\n\n",
			accelDataVariance[0], accelDataVariance[1], accelDataVariance[2]);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

} // void Uart_Display_Statistics(){


/**
 * @name Uart_Print_Sensor
 *
 * Sends a human-readable string to the serial terminal via @p huart1.
 *
 * @param[in] Global: sensorState (enum) Selects which sensor data to print
 * @param[in] Global: xSample: takes most recent sample from given sensor (DOES NOT GRAB FROM xData BUFFERS)
 */

void Uart_Print_Sensor(){


	// Choose what data to TX
	switch (sensorState){
		case(Temperature):{
			int len = snprintf(uartBuffer, sizeof(uartBuffer), "Temperature: %.2f \r\n", tempSample);
			HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Pressure):{
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Pressure: %.2f \r\n", pressureSample);
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Magneto):{
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Magneto: X: %d, Y: %d, Z: %d\r\n",
			                     magSample[0], magSample[1], magSample[2]);
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Accelero):{
			  int len = snprintf(uartBuffer, sizeof(uartBuffer), "Accelero: X: %d, Y: %d, Z: %d\r\n",
					  accelSample[0], accelSample[1], accelSample[2]);
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
			break;
		}
		case(Statistics): {
			Uart_Display_Statistics();
			break;
		}
	} // switch

} // Uart_Print_Sensor


/**
 * @name HAL_GPIO_EXTI_Callback
 *
 * Callback for GPIO EXTI interrupts
 *
 * @param[in] Global: GPIO_Pin
 * @param[in] Global: sensorState: if BUTTON, toggle to next sensor
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {


	if (GPIO_Pin == BUTTON_PIN) {

		// toggle to next sensor or stats
		sensorState = (sensorState + 1) % 5;
#if Part == PART_3
		// REMOVED TO MAKE READ_SENSOR BE CONSTANTLY HAPPENING
		// if sensor state isn't asking for statistics, don't display
		if (sensorState != Statistics) {
			// read sensor
			Read_Sensor();
		} // if
#endif

		// write sensor data to uart
		Uart_Print_Sensor();
	} // if

} //  HAL_GPIO_EXTI_Callback


/**
 * @name Sensors_Write_to_Flash() {
 *
 * Writes data from sensor data buffers to flash
 * TODO: add error checking
 *
 * @param[in] Global: Sensor Data Buffers: tempData, pressureData, magData, accelSample
 */
void Sensors_Write_to_Flash() {

	BSP_QSPI_Erase_Block(FLASH_BLOCK_ADDR);

	// tempData[100] = 100 floats * 4B/float = 400B
    BSP_QSPI_Write((uint8_t*)tempData, TEMP_FLASH_ADDR, sizeof(tempData));

    // pressureData[100] = 100 floats * 4B/float = 400B
    BSP_QSPI_Write((uint8_t*)pressureData, PRESSURE_FLASH_ADDR, sizeof(pressureData));

    // magData[100][3] = 100 floats * 3 * 4B/float = 1200B
    BSP_QSPI_Write((uint8_t*)magData, MAG_FLASH_ADDR, sizeof(magData));

    // accelData[100][3] = 100 floats * 3 * 4B/float = 1200B
    BSP_QSPI_Write((uint8_t*)accelData, ACCEL_FLASH_ADDR, sizeof(accelData));

}

/**
 * @name Sensors_Read_from_Flash() {s
 *
 * Reads sensor data from flash to other data buffers
 *
 * @param[out] Global: Sensor Data Output Buffers: tempDataFlashRead, pressureDataFlashRead, magDataFlashRead, accelDataFlashRead
 */

void Sensors_Read_from_Flash() {

	// tempData[100] = 100 floats * 4B/float = 400B
	if (BSP_QSPI_Read((uint8_t*)tempDataFlashRead, TEMP_FLASH_ADDR, sizeof(tempDataFlashRead)) != QSPI_OK){
	      Error_Handler();
	}

    // pressureData[100] = 100 floats * 4B/float = 400B
	if (BSP_QSPI_Read((uint8_t*)pressureDataFlashRead, PRESSURE_FLASH_ADDR, sizeof(pressureDataFlashRead)) != QSPI_OK){
	      Error_Handler();
	}

    // magData[100][3] = 100 floats * 3 * 4B/float = 1200B
	if (BSP_QSPI_Read((uint8_t*)magDataFlashRead, MAG_FLASH_ADDR, sizeof(magDataFlashRead)) != QSPI_OK){
	      Error_Handler();
	}

    // accelData[100][3] = 100 floats * 3 * 4B/float = 1200B
	if (BSP_QSPI_Read((uint8_t*)accelDataFlashRead, ACCEL_FLASH_ADDR, sizeof(accelDataFlashRead)) != QSPI_OK){
	      Error_Handler();
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
  BSP_QSPI_Init();

#if Part == PART_3
  // Verify that QSPI Flash Functions Work
  	  // Access to 64 Mb = 8MB flash
  	  // Address range from 0->max-1 (recall *1024)
  	  	  // 0x000000 -> 0x7FFFFF
  	  	  // uint32_t address size
  	  // Data r/w in min. units of bytes
  	  	  // write is 256 bytes max
  	  	  // Block: 64kB, 0x10000 aligned
  	  	  // Sector: 4kB, 0x1000 aligned
  	  	  // Page: 256B, 0x100 aligned


  // Erase block
  // uint8_t BSP_QSPI_Erase_Block(uint32_t BlockAddress);
  uint32_t blockAddress = 0x000000;

  if (BSP_QSPI_Erase_Block(blockAddress) != QSPI_OK) {
      Error_Handler();
  }

  // Write page at block base address
  	  // uint8_t BSP_QSPI_Write (uint8_t* pData,     uint32_t WriteAddr,     uint32_t Size);
  	  // Maximum size: 256 B = 1 Page to be safe
  uint8_t writeData[256] = "Hello World"; // set size of writeData to be
  if (BSP_QSPI_Write(writeData, blockAddress, sizeof(writeData)) != QSPI_OK) {
      Error_Handler();
  }

  // Read page at block base address
  	  // uint8_t BSP_QSPI_Read (uint8_t* pData,     uint32_t ReadAddr,     uint32_t Size);
  	  // Reading back single page that was written
  uint8_t readData[256] = {0};
  if (BSP_QSPI_Read(readData, blockAddress, sizeof(readData)) != QSPI_OK) {
      Error_Handler();
  }

  // Check if read == write
  for (int i = 0; i < sizeof(readData); i++) {

	  // if fail transmit message and handle
      if (readData[i] != writeData[i]) {
          int len = snprintf(uartBuffer, sizeof(uartBuffer), "uh oh spaghettio");
          HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);
          Error_Handler();
      }
  }

  // if success send data read
  HAL_UART_Transmit(&huart1, (uint8_t *)readData, sizeof(readData), 100);

#endif

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
	  int16_t accelData[3];
	  BSP_ACCELERO_AccGetXYZ(accelData);
	  char output2[64];
	  len = snprintf(output2, sizeof(output2), "Accelero: X: %d, Y: %d, Z: %d\r\n",
			  accelData[0], accelData[1], accelData[2]);
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
// Check data data can be r/w from flash

	  // get data from all the sensors
	    Read_Sensor();

#if Part == PART_3
	    // when arrays are filled (all at the same time, so doesn't matter which sensorState)
		if (sampleCount[sensorState] == (NUM_SAMPLES - 1)) {
			// save to flash
			Sensors_Write_to_Flash();

			// read from flash
			Sensors_Read_from_Flash();

			// print data at a few random or just read in debug
			for (uint8_t i = 0; i < 100; i++) {

				// print UART for each sensor

				// print temperature sensor
				int len = snprintf(uartBuffer, sizeof(uartBuffer), "Temperature: %.2f \r\n", tempDataFlashRead[i]);
				HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);


				// print pressure sensor
				len = snprintf(uartBuffer, sizeof(uartBuffer), "Pressure: %.2f \r\n", pressureDataFlashRead[i]);
				HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

				// print magneto sensor
				len = snprintf(uartBuffer, sizeof(uartBuffer), "Magneto: X: %d, Y: %d, Z: %d\r\n",
				                     magDataFlashRead[i][0], magDataFlashRead[i][1], magDataFlashRead[i][2]);
				HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);


				// print accel sensor
				len = snprintf(uartBuffer, sizeof(uartBuffer), "Accelero: X: %d, Y: %d, Z: %d\r\n",
				accelDataFlashRead[i][0], accelDataFlashRead[i][1], accelDataFlashRead[i][2]);
				HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);

				// add spaces before next transmission
				len = snprintf(uartBuffer, sizeof(uartBuffer), "\r\n\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 100);


			} // for
		} // if
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

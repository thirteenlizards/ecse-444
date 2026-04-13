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

/*
 * Code References:
 * Sleep Mode: https://wiki.st.com/stm32mcu/wiki/Getting_started_with_PWR#Configure_the_sleep_mode
 * Wrench to PWM: https://github.com/mcgill-robotics/AUV-2025/blob/noetic/catkin_ws/src/propulsion/src/thrust_mapper.py
 * All previous labs for ECSE444, all course materials from ECSE 444
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "octospi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#define ARM_MATH_CM4
#include "arm_math.h"

#include "stm32l4s5i_iot01_qspi.h"
#include "stm32l4s5i_iot01_tsensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Struct for wrench vector */
typedef struct {
    union {
        float wrench[6];
        struct {
            float force_x, force_y, force_z;
            float torque_x, torque_y, torque_z;
        };
    };
    bool isValid;
} WrenchPacket;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_RX_TIMEOUT      100
#define UART_TX_TIMEOUT      100
#define UART_RX_BUFFER_SIZE  128
#define UART_TX_BUFFER_SIZE  256
#define WRENCH_SIZE          6
#define NUM_THRUSTERS        8

#define AUV_ALPHA_DEG   45.0f    // angle_thruster (from launch file)
#define AUV_W           0.47f    // distance_thruster_thruster_width
#define AUV_A           0.0925f  // distance_thruster_middle_length

#define PWM_NEUTRAL     1500
#define PWM_LOWER       1228     // thruster_PWM_lower_limit
#define PWM_UPPER       1768     // thruster_PWM_upper_limit

#define NUM_SENSORS 			3			// Temperature, voltage1, voltage 2
#define NUM_SAMPLES 			5  			// Can increment to be a larger number of samples as desired
#define DATA_SIZE 				0x1000
#define FLASH_BLOCK_ADDR 		0x000000 	// Base address of flash memory area used
#define TEMP_FLASH_ADDR			0x000000
#define VOLTAGE1_FLASH_ADDR  	0x001000
#define VOLTAGE2_FLASH_ADDR	    0x002000
#define MAX_TEMP				30		    // Max allowed temperature

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char     uartRxBuffer[UART_RX_BUFFER_SIZE];
char     uartTxBuffer[UART_TX_BUFFER_SIZE];
uint8_t  uartByte;
uint8_t  byte = 0;
WrenchPacket wrenchPacket = {0};

// CMSIS-DSP matrix data buffers
static float32_t T_data[6 * 8];
static float32_t T_t_data[8 * 6];
static float32_t TTt_data[6 * 6];
static float32_t TTt_copy_data[6 * 6];
static float32_t TTt_inv_data[6 * 6];
static float32_t T_inv_data[8 * 6];
static float32_t wrench_vec[6];
static float32_t forces_vec[8];

// CMSIS-DSP matrix instances
static arm_matrix_instance_f32 mat_T, mat_T_t;
static arm_matrix_instance_f32 mat_TTt, mat_TTt_copy, mat_TTt_inv;
static arm_matrix_instance_f32 mat_T_inv;
static arm_matrix_instance_f32 mat_wrench, mat_forces;

static const float32_t mount_dirs[NUM_THRUSTERS] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

// Samples of sensor data
float tempSample, voltage1Sample, voltage2Sample;

// Store samples of sensor data in an array
float tempData[NUM_SAMPLES];
float voltage1Data[NUM_SAMPLES];
float voltage2Data[NUM_SAMPLES];

// Counter for number of samples taken from each sensor (assume all taken at the same time)
uint16_t sampleCount = {0};

// Store sensor data read from flash
float tempDataFlashRead[NUM_SAMPLES];
float voltage1DataFlashRead[NUM_SAMPLES];
float voltage2DataFlashRead[NUM_SAMPLES];

// Check if flash has been written to at least once
bool writeOnce = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Uart_Processing(void);
void ThrustMapper_Init(void);
void Compute_PWM_From_Wrench(WrenchPacket *wp, uint16_t pwm_out[NUM_THRUSTERS]);
float32_t Force_To_PWM(float32_t force);
void Sensors_Write_to_Flash();
void Sensors_Read_from_Flash();
void Sleep_Mode();
void Bad_Input();

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  ThrustMapper_Init();
  BSP_TSENSOR_Init();

  // Start Timer 4, Channel 3 (PWM) and set to 1500
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);  // set thruster output to 0

  if (BSP_QSPI_Init() != QSPI_OK) {
	  Error_Handler();
	}

  // Erase flash memory to be used for sensor data
  for (int i = 0; i < NUM_SENSORS; i++) {
      if (BSP_QSPI_Erase_Block(FLASH_BLOCK_ADDR + i * DATA_SIZE) != QSPI_OK) {
          Error_Handler();
      }
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Main function to process UART input and make decisions
	  Uart_Processing();

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

/*
 * static void ThrustMapper_BuildT(void)
 * Construct thruster allocation matrix that maps individual thruster forces to six-DOF wrench
 */
static void ThrustMapper_BuildT(void) {
    float32_t alpha = AUV_ALPHA_DEG * (float32_t)M_PI / 180.0f;
    float32_t ca = arm_cos_f32(alpha);
    float32_t sa = arm_sin_f32(alpha);
    float32_t w  = AUV_W;
    float32_t a  = AUV_A;
    float32_t gm = a * sa - (w / 2.0f) * ca;

    // Row 0 — SURGE
    T_data[0]=ca;   T_data[1]=0;    T_data[2]=0;    T_data[3]=-ca;
    T_data[4]=-ca;  T_data[5]=0;    T_data[6]=0;    T_data[7]=ca;
    // Row 1 — SWAY
    T_data[8]=-sa;  T_data[9]=0;    T_data[10]=0;   T_data[11]=-sa;
    T_data[12]=sa;  T_data[13]=0;   T_data[14]=0;   T_data[15]=sa;
    // Row 2 — HEAVE
    T_data[16]=0;   T_data[17]=-1;  T_data[18]=-1;  T_data[19]=0;
    T_data[20]=0;   T_data[21]=-1;  T_data[22]=-1;  T_data[23]=0;
    // Row 3 — ROLL
    T_data[24]=0;   T_data[25]=w/2; T_data[26]=w/2; T_data[27]=0;
    T_data[28]=0;   T_data[29]=-w/2;T_data[30]=-w/2;T_data[31]=0;
    // Row 4 — PITCH
    T_data[32]=0;   T_data[33]=-a;  T_data[34]=a;   T_data[35]=0;
    T_data[36]=0;   T_data[37]=a;   T_data[38]=-a;  T_data[39]=0;
    // Row 5 — YAW
    T_data[40]=-gm; T_data[41]=0;   T_data[42]=0;   T_data[43]=gm;
    T_data[44]=-gm; T_data[45]=0;   T_data[46]=0;   T_data[47]=gm;
}

/*
 * void ThrustMapper_Init(void)
 * Calculate the pseudoinverse of the thruster allocation matrix
 */
void ThrustMapper_Init(void) {
    ThrustMapper_BuildT();

    arm_mat_init_f32(&mat_T,        6, 8, T_data);
    arm_mat_init_f32(&mat_T_t,      8, 6, T_t_data);
    arm_mat_init_f32(&mat_TTt,      6, 6, TTt_data);
    arm_mat_init_f32(&mat_TTt_copy, 6, 6, TTt_copy_data);
    arm_mat_init_f32(&mat_TTt_inv,  6, 6, TTt_inv_data);
    arm_mat_init_f32(&mat_T_inv,    8, 6, T_inv_data);
    arm_mat_init_f32(&mat_wrench,   6, 1, wrench_vec);
    arm_mat_init_f32(&mat_forces,   8, 1, forces_vec);

    arm_mat_trans_f32(&mat_T, &mat_T_t);
    arm_mat_mult_f32(&mat_T, &mat_T_t, &mat_TTt);
    memcpy(TTt_copy_data, TTt_data, sizeof(TTt_copy_data));
    arm_mat_inverse_f32(&mat_TTt_copy, &mat_TTt_inv);
    arm_mat_mult_f32(&mat_T_t, &mat_TTt_inv, &mat_T_inv);
}

/*
 * float32_t Force_To_PWM(float32_t force)
 * Convert requested force on a specific thruster to a PWM value using polynomial regression
 */
float32_t Force_To_PWM(float32_t force) {
    static const float32_t c[8] = {
        1500.0f, 30.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    float32_t result = c[0];
    float32_t f_pow  = force;
    for (int i = 1; i <= 7; i++) {
        result += c[i] * f_pow;
        f_pow  *= force;
    }
    return result;
}

/*
 * void Compute_PWM_From_Wrench(WrenchPacket *wp, uint16_t pwm_out[NUM_THRUSTERS])
 * Convert a requested wrench in the frame of the AUV to specific force outputs from specific thrusters
 */
void Compute_PWM_From_Wrench(WrenchPacket *wp, uint16_t pwm_out[NUM_THRUSTERS]) {
    wrench_vec[0] = wp->force_x;  wrench_vec[1] = wp->force_y;
    wrench_vec[2] = wp->force_z;  wrench_vec[3] = wp->torque_x;
    wrench_vec[4] = wp->torque_y; wrench_vec[5] = wp->torque_z;

    arm_mat_mult_f32(&mat_T_inv, &mat_wrench, &mat_forces);

    for (int i = 0; i < NUM_THRUSTERS; i++) {
        float32_t pwm_f = Force_To_PWM(forces_vec[i] * mount_dirs[i]);
        if      (pwm_f > PWM_UPPER) pwm_f = PWM_UPPER;
        else if (pwm_f < PWM_LOWER) pwm_f = PWM_LOWER;
        pwm_out[i] = (uint16_t)pwm_f;
    }
}

/**
 * void Uart_Processing(void)
 * Processes UART byte-by-byte and does actions based on what is received
 * Parses based on assumed formats for RX
 */

void Uart_Processing(void) {
    static char line[128];
    static uint8_t idx = 0;

    // When a byte is received, check what type of data/command we're getting
    if (HAL_UART_Receive(&huart1, &byte, 1, UART_RX_TIMEOUT) == HAL_OK) {

    	// Check if we're at the end of a line (i.e. a single command)
        if (byte == '\r' || byte == '\n') {
            line[idx] = '\0';  // terminate string

            // 1. Voltage command: v, <voltage1, voltage2>
            if (line[0] == 'v') {

                if (sscanf(line, "v, %f, %f", &voltage1Sample, &voltage2Sample) == 2) {

                	// Save the data into buffers
                    voltage1Data[sampleCount] = voltage1Sample;
                    voltage2Data[sampleCount] = voltage2Sample;

                    // Get data from temp sensor
                    tempSample = BSP_TSENSOR_ReadTemp();
                    tempData[sampleCount] = tempSample;

                    // Increment the sample counter
                    sampleCount++;

                    // If received enough samples, save to flash
                    if (sampleCount >= (NUM_SAMPLES)) {
                    	Sensors_Write_to_Flash();
                    	writeOnce = true; // flag that flash has been written to
                    	sampleCount = 0;  // reset sample counter
                    }


                    // UART TX the receieved voltage and the temperature value
                    int len = snprintf(uartTxBuffer, sizeof(uartTxBuffer),
                        "Data -> T: %.2f V1: %.2f V2: %.2f\r\n",
                        tempSample, voltage1Sample, voltage2Sample);

                    HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, len, UART_TX_TIMEOUT);

                    // If voltage is too low, go into sleep mode to save power
                    if ((voltage1Sample < 10) || (voltage2Sample < 10)) {
						int len = snprintf(uartTxBuffer, sizeof(uartTxBuffer),
									"Battery voltage too low. Entering sleep mode now.\n");
								HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, len, UART_TX_TIMEOUT);
								Sleep_Mode();
                    }
                }

                // If input in wrong format, TX a message to the user
                else{
                	Bad_Input();
                }
            }

            // 2. Telemetry command: t
            else if (line[0] == 't') {

            	if (writeOnce) { // make sure no flash read unless data has been written at least once

					// Get tempDataFlashRead, voltage1DataFlashRead, and voltage2DataFlashRead
					Sensors_Read_from_Flash();

					// TX data from flash
					for (int i = 0; i < (NUM_SAMPLES); i++) {

						int len = snprintf(uartTxBuffer, sizeof(uartTxBuffer),
							"Sample %d -> T: %.2f V1: %.2f V2: %.2f\r\n",
							i, tempDataFlashRead[i], voltage1DataFlashRead[i], voltage2DataFlashRead[i]);

						HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, len, UART_TX_TIMEOUT);
					}

            	}

            	// If not enough samples have been taken to display
            	else {
					int len = snprintf(uartTxBuffer, sizeof(uartTxBuffer),
						"Not enough samples taken. You have taken %d samples and you need %d.",
						sampleCount, (uint8_t)NUM_SAMPLES);
					HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, len, UART_TX_TIMEOUT);

            	}


            }

            // 3. Wrench command: w, <force_x, force_y, force_z, torque_x, torque_y, torque_z>
            else if (line[0] == 'w') {
                WrenchPacket wp = {0};

                // Save the data into buffer
                if (sscanf(line, "w, %f, %f, %f, %f, %f, %f",
                    &wp.force_x, &wp.force_y, &wp.force_z,
                    &wp.torque_x, &wp.torque_y, &wp.torque_z) == 6) {

                    wp.isValid = true;

                    // Calculate PWM for each thruster from the input wrench + TX
                    uint16_t pwm[NUM_THRUSTERS];
                    Compute_PWM_From_Wrench(&wp, pwm);

                    int len = snprintf(uartTxBuffer, sizeof(uartTxBuffer), "PWM:[%u,%u,%u,%u,%u,%u,%u,%u]\r\n",
                        pwm[0], pwm[1], pwm[2], pwm[3],
                        pwm[4], pwm[5], pwm[6], pwm[7]);

                    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm[0]);

                    HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, len, UART_TX_TIMEOUT);
                }
                // If input in wrong format, TX a message to the user
                else {
                	Bad_Input();
                }
            }

            // If input in wrong format, TX a message to the user
            else {

            	if (idx > 0 || line[0] != '\0') { // don't trigger on empty line
            		int len = snprintf(uartTxBuffer, sizeof(uartTxBuffer),
            	                        	"Invalid Command. Valid commands: <f, Fx, Fy, Fz, Tx, Ty, Tz> and <t>\r\n");
            		HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, len, UART_TX_TIMEOUT);
            	}
            }

            // reset buffer
            idx = 0;
        }

        // Overflow protection
        else {
            if (idx < sizeof(line) - 1) {
                line[idx++] = byte;
            }
            else {
                idx = 0;
            }
        }
    }
}


/**
 * void Sensors_Write_to_Flash() {
 * Write sensor data buffers to flash (tempData, voltage1Data, voltage2Data)
 */
void Sensors_Write_to_Flash() {

	BSP_QSPI_Erase_Block(FLASH_BLOCK_ADDR);

	// 100 floats * 4B/float = 400B
    BSP_QSPI_Write((uint8_t*)tempData, TEMP_FLASH_ADDR, sizeof(tempData));

    BSP_QSPI_Write((uint8_t*)voltage1Data, VOLTAGE1_FLASH_ADDR, sizeof(voltage1Data));

    BSP_QSPI_Write((uint8_t*)voltage2Data, VOLTAGE2_FLASH_ADDR, sizeof(voltage2Data));


}



/**
 * Sensors_Read_from_Flash( {
 * Read sensor data buffers to flash (tempDataFlashRead, voltage1DataFlashRead, voltage2DataFlashRead)
 */
void Sensors_Read_from_Flash() {

	// 100 floats * 4B/float = 400B
	if (BSP_QSPI_Read((uint8_t*)tempDataFlashRead, TEMP_FLASH_ADDR, sizeof(tempDataFlashRead)) != QSPI_OK){
	      Error_Handler();
	}

	if (BSP_QSPI_Read((uint8_t*)voltage1DataFlashRead, VOLTAGE1_FLASH_ADDR, sizeof(voltage1DataFlashRead)) != QSPI_OK){
	      Error_Handler();
	}

	if (BSP_QSPI_Read((uint8_t*)voltage2DataFlashRead, VOLTAGE2_FLASH_ADDR, sizeof(voltage2DataFlashRead)) != QSPI_OK){
	      Error_Handler();
	}

}


/**
 * void Sleep_Mode() {
 * Enter Sleep Mode
 */
void Sleep_Mode() {

	// Set thruster output to 0
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);

	// Enter sleep mode
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_1) // If blue button pressed
    {
        // Wake up
        HAL_ResumeTick();
    }
}

/**
 * void Bad_Input()
 * Sends TX in response to invalid input from computer
 */
void Bad_Input() {
	int len = snprintf(uartTxBuffer, sizeof(uartTxBuffer),
                        	"Invalid Command. Valid commands: <f, Fx, Fy, Fz, Tx, Ty, Tz> and <t>\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, len, UART_TX_TIMEOUT);
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

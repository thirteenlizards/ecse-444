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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// States of UART processing
typedef enum { Waiting, Receiving } UartState;

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
WrenchPacket UART_Parse_Wrench(uint8_t byte);
void         Test_Uart_Processing(void);
void         ThrustMapper_Init(void);
void         Compute_PWM_From_Wrench(WrenchPacket *wp, uint16_t pwm_out[NUM_THRUSTERS]);
float32_t    Force_To_PWM(float32_t force);

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

  // Start Timer 4, Channel 3 (PWM)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

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

float32_t Force_To_PWM(float32_t force) {
    // !! Replace c[] with real coefficients from thrust_mapper_utils.py !!
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
 * UART Writeback to test UART functionning
 * process next byte.
 * @param byte 		next byte from uart
 * @return 			whatever u want!
 */

void Test_Uart_Processing(void) {
    if (HAL_UART_Receive(&huart1, &byte, 1, UART_RX_TIMEOUT) == HAL_OK) {
        wrenchPacket = UART_Parse_Wrench(byte);
        if (wrenchPacket.isValid) {
            uint16_t pwm[NUM_THRUSTERS];
            Compute_PWM_From_Wrench(&wrenchPacket, pwm);

            uint8_t len = snprintf(uartTxBuffer, sizeof(uartTxBuffer),
                "W:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] PWM:[%u,%u,%u,%u,%u,%u,%u,%u]\r\n",
                wrenchPacket.force_x,  wrenchPacket.force_y,  wrenchPacket.force_z,
                wrenchPacket.torque_x, wrenchPacket.torque_y, wrenchPacket.torque_z,
                pwm[0], pwm[1], pwm[2], pwm[3],
                pwm[4], pwm[5], pwm[6], pwm[7]);

            HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, UART_TX_TIMEOUT);
        }
    }
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

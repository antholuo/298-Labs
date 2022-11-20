/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* servo pulse width */
volatile int pulse_width = 0;

/* not sure if this is needed tbh */
uint8_t rcv_intpt_flag = 0;

/* transmit message buffer */
uint8_t txd_msg_buffer[64] = {0};
uint8_t msg_buffer[64] = {0};

/* cmd to us100 to begin distance sensing */
uint8_t cmd_dist = {0x55};

/* us100 interrupt flag */
volatile uint8_t us100_rx_flag = 0;

/* two bytes to store dist value received from us100 */
volatile uint8_t us100_buffer[2] = {0};

/* 16 bit distance for reporting */
volatile uint16_t distance = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void read_us100_dist();
void send_dist_to_pc();
void step_servo_pulse_width(int amount);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  int TIM2_Ch1_DCVAL = 500;
  int TIM2_Ch2_DCVAL = 2500;
  pulse_width = 1500;

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  TIM2->PSC = 16 - 1;
  TIM2->ARR = 20000 - 1;
  TIM2->CCR1 = TIM2_Ch1_DCVAL;
  TIM2->CCR2 = TIM2_Ch2_DCVAL;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    read_us100_dist();
    send_dist_to_pc();
    HAL_Delay(500);
    //	  while(TIM2_Ch1_DCVAL < 2500) {
    //		  TIM2_Ch1_DCVAL += (20);
    //		  TIM2_Ch2_DCVAL -= (20);
    //		  TIM2->CCR1 = TIM2_Ch1_DCVAL;
    //		  TIM2->CCR2 = TIM2_Ch2_DCVAL;
    //		  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
    //		  read_us100_dist();
    //		  send_dist_to_pc();
    //		  HAL_Delay(500);
    //	  }
    //	  while(TIM2_Ch1_DCVAL > 500) {
    //		  TIM2_Ch1_DCVAL -= (20);
    //		  TIM2_Ch2_DCVAL += (20);
    //		  TIM2->CCR1 = TIM2_Ch1_DCVAL;
    //		  TIM2->CCR2 = TIM2_Ch2_DCVAL;
    //		  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
    //		  read_us100_dist();
    //		  send_dist_to_pc();
    //		  HAL_Delay(500);
    //	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    us100_rx_flag = 01; // this flag is set to show that an receiver interrupt has occurred
  }
}

void read_us100_dist()
{
  HAL_UART_Receive_IT(&huart1, &us100_buffer, 2); // get the UART Receiver ready to receive data and then generate an interrupt
  HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);  // send out the command to the US-100 (TRIG Input)

  while (us100_rx_flag == (00))
  {
  }; // wait for the Flag to be set by the Interrupt Callback routine

  distance = (us100_buffer[0] << 8) + us100_buffer[1]; // convert the two indivual byts into a single 16 bit quantity

  // this section is handled by send_dist_to_pc();
  us100_rx_flag = 00;
  HAL_Delay(200);
  return;
}

void send_dist_to_pc()
{
  sprintf((char *)msg_buffer, "\r\n Distance Sensed (mm)= %d", distance);  // set up the report content
  HAL_Delay(200);                                                          // this is just used to slow down the sequence (user determined)
  HAL_UART_Transmit(&huart6, msg_buffer, strlen((char *)msg_buffer), 500); // send out the report
  return;
}

void step_servo_pulse_width(int amount)
{
  return;
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

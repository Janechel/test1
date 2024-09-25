/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
#include "servo.h"
#include <stdio.h>

uint8_t order_buffer[20];								//Store Generated Instructions
uint8_t order_len;											//Instruction Length
uint8_t receive[20];										//Store the received status packet
uint16_t analysis_data;									//Data parsed from the status packet
uint8_t ret;														//Status Flag

void initTransmitMode(UART_HandleTypeDef *huart);
void initReceiveMode(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//Change the torque switch of servo ID1 to OFF.
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);

		//Change the control mode of servo ID1 to the current control mode.
    servo_set_control_mode(1, 2, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);

		//Change the torque switch of servo ID1 to ON.
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);

		//Change the target PWM of servo ID1 to 100mA.
    servo_set_target_current(1, 100, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_target_current_analysis(receive);

		HAL_Delay(3000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance==USART1)
	{
		HAL_UARTEx_ReceiveToIdle_IT(&huart1,receive,50);
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

#ifdef  USE_FULL_ASSERT
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

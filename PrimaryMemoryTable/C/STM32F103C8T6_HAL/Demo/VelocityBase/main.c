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

uint8_t order_buffer[20];
uint8_t order_len;
uint8_t receive[20];
uint8_t receive_len;										//应答包长度
uint16_t analysis_data;
uint8_t ret; 
uint8_t write_buffer[20] = {0};         //写入内存表数据

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
	
	struct servo_sync_parameter servo;
	
	servo.id_counts = 2;            //同步写两个舵机
  servo.id[0] = 1;                //第一个舵机id为1
  servo.id[1] = 2;                //第二个舵机id为2

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
    //将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    servo_sync_write_torque_switch(servo, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("Sync Write torque witch successfully.\r\n");
		HAL_Delay(1000);

    //将ID1、ID2舵机的控制模式，分别修改为控速模式
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("Sync Write control mode successfully.\r\n");
		HAL_Delay(1000);

	  //设置舵机的控速目标位置
    servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_velocity_base_target_position_analysis(receive);

		HAL_Delay(1000);
		
		//在控速模式下，让ID1以360°/s的控速目标速度运动到300°位置
		write_buffer[0] = 3000 & 0xff;
		write_buffer[1] = (3000 >> 8) & 0xff;
		write_buffer[2] = 3600 & 0xff;
		write_buffer[3] = (3600 >> 8) & 0xff;

		servo_write(1, 0x35, 4, write_buffer, order_buffer, &order_len);
		
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		HAL_Delay(10);
		PRINTF("servo pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
		
		//在控速模式下，让ID1舵机以360°/s的控速目标速度、500°/s²的控速目标加速度、50°/s²的控速目标减速度运动到0°位置
		write_buffer[0] = 0 & 0xff;
		write_buffer[1] = (0 >> 8) & 0xff;
		write_buffer[2] = 3600 & 0xff;
		write_buffer[3] = (3600 >> 8) & 0xff;
		write_buffer[4] = 10 & 0xff;
		write_buffer[5] = 1 & 0xff;

		servo_write(1, 0x35, 6, write_buffer, order_buffer, &order_len);
		
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		HAL_Delay(10);
		PRINTF("servo pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
		
		//在控速模式下，让ID1舵机运动到150中位°，让ID2舵机运动到0°位置

		//id为1，2的舵机运动位置分别设置为1500，0，值和前面的id设置对应
		servo.position[0] = 1500;
		servo.position[1] = 0;

		servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
		
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		
		HAL_Delay(1000);
		
		//在控速模式下，让ID1以360°/s的控速目标速度运动到300°位置，让ID2以720°/s的控速目标速度运动到150°位置

		//id为1，2的舵机速度分别设置为3600，7200，位置分别设置为3000，1500
		servo.velocity[0] = 3600;
		servo.velocity[1] = 7200;
		servo.position[0] = 3000;
		servo.position[1] = 1500;

		servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
	
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		
		HAL_Delay(1000);
			
		//在控速模式下，让ID1舵机以720°/s的控速目标速度、500°/s²的控速目标加速度、50°/s²的控速目标减速度运动到0°位置，让ID2舵机以360°/s的控速目标速度、50°/s²的控速目标加速度、500°/s²的控速目标减速度运动到300°位置

		//id为1，2的舵机速度分别设置为7200，3600，位置分别设置为0，3000,加速度分别设置为10，1，减速度分别设置为1，10
		servo.velocity[0] = 7200;
		servo.velocity[1] = 3600;
		servo.position[0] = 0;
		servo.position[1] = 3000;
		servo.acc_velocity[0] = 10;
		servo.acc_velocity[1] = 1;
		servo.dec_velocity[0] = 1;
		servo.dec_velocity[1] = 10;

		servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);

		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		
		HAL_Delay(1000);
		
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
		//再次 ?启空闲中断接收，不然只会接收 ?次数 ?
		receive_len = Size;
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

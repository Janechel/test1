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

#define PING_TEST 0											//PING指令测试
#define READ_TEST 0				    					//读取舵机数据测试
#define WRITE_TEST 0			    					//写入舵机数据测试
#define SYNC_WRITE_TEST 0		    				//同步写测试
#define FACTORY_RESET_TEST 0	    			//恢复出厂设置测试
#define PARAMETER_RESET_TEST 0	    		//参数重置测试
#define CALIBRATION_TEST 0		    			//校正偏移值测试
#define REBOOT_TEST 0			    					//重启测试
#define MODIFY_ID 0                		  //修改已知舵机ID测试
#define MODIFY_UNKNOWN_ID 0         		//修改未知ID舵机ID测试


uint8_t order_buffer[20];								//存放生成的指令
uint8_t order_len;											//指令长度
uint8_t receive[20];										//存放接收的应答包
uint8_t receive_len;										//应答包长度
uint16_t analysis_data;									//应答包解析出来的数据
uint8_t ret;														//错误检验标志
uint16_t position = 0;                  //当前位置
uint16_t current = 0;                   //当前电流
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
#if FACTORY_RESET_TEST
		//恢复出厂设置
    servo_factory_reset(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		HAL_Delay(10);
		ret = servo_factory_reset_analysis(receive);
		if (ret == SUCCESS)
		{
			PRINTF("servo factory reset successfully!\r\n");
		}
		HAL_Delay(1000);
#endif			
		
#if PARAMETER_RESET_TEST
		//参数重置
    servo_parameter_reset(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		HAL_Delay(10);
		ret = servo_parameter_reset_analysis(receive);
		if (ret == SUCCESS)
		{
				PRINTF("servo parameter reset successfully!\r\n");
		}
		HAL_Delay(1000);
#endif			

#if CALIBRATION_TEST
		//校正偏移值
    servo_calibration(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		HAL_Delay(10);
		ret = servo_calibration_analysis(receive);
		if (ret == SUCCESS)
		{
				PRINTF("servo calibration successfully!\r\n");
		}
		HAL_Delay(1000);
#endif				
		
#if REBOOT_TEST
		//舵机重启
    servo_reboot(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
    PRINTF("The servo reboot");
		HAL_Delay(1000);
#endif	

#if MODIFY_ID
		//修改ID1舵机ID为2
    servo_modify_known_id(1, 2, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
    PRINTF("modify id success");
		HAL_Delay(1000);
#endif

#if MODIFY_ID
		//将未知ID舵机的ID编号修改为1
    servo_modify_unknown_id(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
    PRINTF("modify id success");
		HAL_Delay(1000);
#endif
		
#if PING_TEST
		//向ID为1的舵机发送PING指令
    servo_ping(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_ping_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("Ping succeed!  the model_number is %d\r\n", analysis_data);
		HAL_Delay(1000);
#endif		
		
#if READ_TEST
		//读取ID1舵机的当前位置
    servo_read_present_position(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_present_position_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			  PRINTF("present position is %d",analysis_data);
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//读取ID1舵机的当前电流
    servo_read_present_current(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UART_Receive_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_present_current_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("present current is %d",analysis_data);
		HAL_Delay(1000);
#endif
		
#if READ_TEST
    //读取ID1舵机的当前位置和当前电流
    servo_read_present_position_and_present_current(1, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UART_Receive_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_present_position_and_present_current_analysis(receive, &position, &current);
    if(ret == SUCCESS)
			PRINTF("present position is : % d, present current is : % d\r\n", position, current);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的当前速度
    servo_read_present_velocity(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);
		
    ret = servo_read_present_velocity_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present velocity is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前的规划位置
    servo_read_present_profile_position(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_present_profile_position_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present profile position is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前规划速度
    servo_read_present_profile_velocity(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_present_profile_velocity_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present profile velocity is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前PWM
    servo_read_present_pwm(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_present_pwm_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present pwm analysis is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前温度
    servo_read_present_temperature(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_present_temperature_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present temperature is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前输入电压
    servo_read_present_voltage(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_present_voltage_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present voltage is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控时目标运行时间
    servo_read_time_base_target_moving_time(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_time_base_target_moving_time_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present time base target moving time is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控时目标位置
    servo_read_time_base_target_position(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_time_base_target_position_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present time base target position is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的控时加速度等级
    servo_read_time_base_target_acc(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_time_base_target_acc_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present time base target acc is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
    //读取ID1舵机的控时目标位置和运行时间
    servo_read(1, 0x3C, 4, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);
		
    PRINTF("the time base target position and moving time pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
    //读取ID1舵机的控时目标加速度等级、位置和运行时间
    servo_read(1, 0x3B, 5, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);
		
    PRINTF("the time base target acc, position and moving time pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//读取ID1舵机的控速目标减速度
    servo_read_velocity_base_target_dec(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);
		
    ret = servo_read_velocity_base_target_dec_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present velocity base target dec is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控速目标加速度
    servo_read_velocity_base_target_acc(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_velocity_base_target_acc_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present velocity base target acc is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控速目标速度
    servo_read_velocity_base_target_velocity(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_velocity_base_target_velocity_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present velocity base target velocity is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控速目标位置
    servo_read_velocity_base_target_position(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_velocity_base_target_position_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
      PRINTF("present velocity base target position is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
    //读取ID1舵机的控速目标位置和速度
    servo_read(1, 0x35, 4, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    PRINTF("the velocity base target position and velocity pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//读取ID1舵机的目标电流
    servo_read_target_current(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_target_current_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("target current is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的目标PWM
    servo_read_target_pwm(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_target_pwm_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("target pwm is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的扭矩开关
    servo_read_torque_switch(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_torque_switch_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("torque switch is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的LED开关
    servo_read_led_switch(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_led_switch_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("led switch is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的Flash开关
    servo_read_flash_switch(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_flash_switch_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("flash switch is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电流校正值
    servo_read_current_offset(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_current_offset_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("current offset is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的中位校正值
    servo_read_calibration(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_calibration_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("calibration is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控制模式
    servo_read_control_mode(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_control_mode_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("control mode is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的卸载保护条件
    servo_read_shutdown_condition(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_shutdown_condition_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("shutdown condition is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的LED报警条件
    servo_read_led_condition(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_led_condition_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("led condition is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的位置控制D增益
    servo_read_position_control_d_gain(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_position_control_d_gain_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("position control d gain is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的位置控制I增益
    servo_read_position_control_i_gain(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_position_control_i_gain_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("position control i gain is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的位置控制P增益
    servo_read_position_control_p_gain(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_position_control_p_gain_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("position control p gain is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
    //读取ID1舵机的位置控制PID增益
    servo_read(1, 0x1B, 6, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    PRINTF("position control pid gain pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//读取ID1舵机的PWM叠加值
    servo_read_pwm_punch(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_pwm_punch_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("pwm punch is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的反转死区
    servo_read_ccw_deadband(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_ccw_deadband_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("ccw deadband is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的正转死区
    servo_read_cw_deadband(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_cw_deadband_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("cw deadband is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电流保护时间
    servo_read_current_shutdown_time(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_current_shutdown_time_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("current shutdown time is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电流上限
    servo_read_max_current_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_max_current_limit_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("max current limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的PWM上限
    servo_read_max_pwm_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_max_pwm_limit_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("max pwm limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电压上限
    servo_read_max_voltage_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_max_voltage_limit_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("max voltage limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电压下限
    servo_read_min_voltage_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_min_voltage_limit_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("min voltage limit is %d",analysis_data);
		HAL_Delay(1000);
#endif
		
#if READ_TEST
    //读取ID1舵机的电压限制
    servo_read(1, 0x10, 2, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

		PRINTF("the voltage limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的温度上限
    servo_read_max_temperature_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_max_temperature_limit_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("max temperature limit is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的最大位置限制
    servo_read_max_angle_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);


    ret = servo_read_max_angle_limit_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("max angle limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的最小位置限制
    servo_read_min_angle_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_min_angle_limit_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("min angle limit is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
    //读取ID1舵机的位置限制
    servo_read(1, 0x0B, 4, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    PRINTF("the angle limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//读取ID1舵机的状态返回级别
    servo_read_return_level(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_return_level_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("return level is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的应答延时时间
    servo_read_return_delay_time(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_return_delay_time_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("return delay time is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的波特率
    servo_read_baud_rate(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_baud_rate_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("baud rate is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的出厂编号
    servo_read_model_information(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_model_information_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("model information is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的固件版本号
    servo_read_firmware_version(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_read_firmware_version_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("firmware version is %d",analysis_data);
		HAL_Delay(1000);
#endif
		
#if SYNC_WRITE_TEST
    //将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    servo_sync_write_torque_switch(servo, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("Sync Write torque witch successfully.\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //将ID1、ID2舵机的控制模式，分别修改为控速模式
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("Sync Write control mode successfully.\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标速度
		
		//id为1，2的舵机速度分别设置为3600，1800，值和前面的id设置对应
    servo.velocity[0] = 3600;
    servo.velocity[1] = 1800;
		
		servo_sync_write_velocity_base_target_velocity(servo, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标加速度
		
		//id为1，2的舵机加速度分别设置为150，150，值和前面的id设置对应
    servo.acc_velocity[0] = 150;          
    servo.acc_velocity[1] = 150;    
		
		servo_sync_write_velocity_base_target_acc(servo, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标减速度
		
		//id为1，2的舵机减速度分别设置为150，150，值和前面的id设置对应
    servo.dec_velocity[0] = 150;           
    servo.dec_velocity[1] = 150;    
		
		servo_sync_write_velocity_base_target_dec(servo, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标位置
		
		//id为1，2的舵机运动位置分别设置为0，0，值和前面的id设置对应
    servo.position[0] = 0;
    servo.position[1] = 0;
		
		servo_sync_write_velocity_base_target_position(servo, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标位置和速度
		
		//id为1，2的舵机速度分别设置为1800，3600，位置分别设置为3000，3000
    servo.velocity[0] = 1800;
    servo.velocity[1] = 3600;
    servo.position[0] = 3000;
    servo.position[1] = 3000;
		
		servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的加速度，减速度，速度和位置

    //id为1，2的舵机速度分别设置为3600，3600，位置分别设置为0，0,加速度分别设置为100，100，减速度分别设置为100，100
    servo.velocity[0] = 3600;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 0;
    servo.acc_velocity[0] = 100;
    servo.acc_velocity[1] = 100;
    servo.dec_velocity[0] = 100;
    servo.dec_velocity[1] = 100;
	
		servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    servo_sync_write_torque_switch(servo, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("Sync Write torque witch successfully.\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //将ID1、ID2舵机的控制模式，分别修改为控时模式
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("Sync Write control mode successfully.\r\n");
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控时目标加速度等级
		
		//设置舵机id为1，2的加速度等级分别为0，0
    servo.acc_velocity_grade[0] = 0;
    servo.acc_velocity_grade[1] = 0;
		
    servo_sync_write_time_base_target_acc(servo, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 20);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控时目标位置和运动时间
		
		//设置舵机id为1，2的运动位置为3000，3000，运动时间为500ms，1500ms
    servo.position[0] = 3000;
    servo.position[1] = 3000;
    servo.time[0] = 500;
    servo.time[1] = 1500;
		
    servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 20);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的状态返回级别修改为应答所有指令
    servo_set_return_level(1, 2, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_return_level_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set return level successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的应答延迟时间修改为500us
		servo_set_return_delay_time(1, 250, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_return_delay_time_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set return delay time successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的波特率为1000000
    servo_set_baud_rate(1, 7, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_baud_rate_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set baud rate successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将舵机ID1的最小位置限制修改为0°
    servo_set_min_angle_limit(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_min_angle_limit_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set min angle limit successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将舵机ID1的最大位置限制修改为300°
    servo_set_max_angle_limit(1, 3000, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_max_angle_limit_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set max angle limit successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将舵机ID1的位置限制修改为0°~300°
    write_buffer[0] = 0 & 0xff;;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3000 & 0xff;
    write_buffer[3] = (3000 >> 8) & 0xff;

    servo_write(1, 0x0B, 4, write_buffer, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		PRINTF("servo set angle limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的温度上限修改为65℃
    servo_set_max_temperature_limit(1, 65, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_max_temperature_limit_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set max temperature limit successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电压上限修改为8.4V
    servo_set_max_voltage_limit(1,84, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_max_voltage_limit_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set max voltage limit successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电压下限修改为3.5V
    servo_set_min_voltage_limit(1, 35, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_min_voltage_limit_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set min voltage limit successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电压限制修改为3.5V~8.4V
    write_buffer[0] = 84 & 0xff;
    write_buffer[1] = 35 & 0xff;

    servo_write(1, 0x10, 2, write_buffer, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		PRINTF("the voltage limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的PWM上限修改为90%
    servo_set_max_pwm_limit(1, 900, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_max_pwm_limit_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set max pwm limit successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电流上限修改为900mA
    servo_set_max_current_limit(1, 900, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_max_current_limit_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set max current limit successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电流保护时间修改为500ms
    servo_set_current_shutdown_time(1, 500, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_current_shutdown_time_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set current shutdown time successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的正转死区修改为0.2°
    servo_set_cw_deadband(1, 2, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_cw_deadband_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set cw deadband successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的反转死区修改为0.2°
    servo_set_ccw_deadband(1, 2, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_ccw_deadband_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set ccw deadband successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的正反转死区修改为0.2°
    write_buffer[0] = 2 & 0xff;
    write_buffer[1] = 2 & 0xff;

    servo_write(1, 0x18, 2, write_buffer, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		PRINTF("servo set cw deadband and ccw deadband pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的PWM叠加值修改为1%
    servo_set_pwm_punch(1, 10, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_pwm_punch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set pwm punch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的位置控制P增益修改为5995
    servo_set_position_control_p_gain(1, 5995, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_position_control_p_gain_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set position control p gain successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的位置控制I增益修改为5
    servo_set_position_control_i_gain(1, 5, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_position_control_i_gain_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set position control i gain successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的位置控制D增益修改为145
    servo_set_position_control_d_gain(1, 145, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_position_control_d_gain_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set position control d gain successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的位置控制PID增益，分别修改为5500、100、250
    write_buffer[0] = 5500 & 0xff;
    write_buffer[1] = (5500 >> 8) & 0xff;
    write_buffer[2] = 100 & 0xff;
    write_buffer[3] = (100 >> 8) & 0xff;
    write_buffer[4] = 250 & 0xff;
    write_buffer[5] = (250 >> 8) & 0xff;

    servo_write(1, 0x1B, 6, write_buffer, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);

		PRINTF("servo set position control pid gain pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive[i]);
		}
		PRINTF("\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的LED报警条件修改为开启堵转报错、过热报错和角度报错
    servo_set_led_condition(1, 38, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_led_condition_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set led condition successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的卸载保护条件修改为开启堵转报错、过热报错、电压报错和角度报错
    servo_set_shutdown_conditions(1, 39, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_shutdown_conditions_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set shutdown conditions successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的Flash开关状态修改为打开
    servo_set_flash_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_flash_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set flash switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的Flash开关状态修改为关闭
    servo_set_flash_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_flash_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set flash switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的LED开关状态修改为打开
    servo_set_led_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_led_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set led switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的LED开关状态修改为关闭
    servo_set_led_switch(1, 0, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_led_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set led switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为关闭
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_torque_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控制模式修改为PWM输出控制模式
    servo_set_control_mode(1, 3, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_control_mode_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set control mode successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为开启
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_torque_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的目标PWM修改为-50%
    servo_set_target_pwm(1, -500, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_target_pwm_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set target pwm successfully.\r\n");

		HAL_Delay(3000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为关闭
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_torque_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控制模式修改为电流控制模式
    servo_set_control_mode(1, 2, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_control_mode_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set control mode successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为开启
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_torque_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的目标电流修改为-400mA
    servo_set_target_current(1, -400, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_target_current_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set target current successfully.\r\n");

		HAL_Delay(3000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为关闭
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_torque_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控制模式修改为控速模式
    servo_set_control_mode(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_control_mode_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set control mode successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为开启
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_torque_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控速目标速度修改为360°/s
    servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_velocity_base_target_velocity_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set velocity base target velocity successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控速目标加速度修改为500°/s²
    servo_set_velocity_base_target_acc(1, 10, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_velocity_base_target_acc_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set velocity base target acc successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控速目标减速度修改为50°/s²
    servo_set_velocity_base_target_dec(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_velocity_base_target_dec_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set velocity base target dec successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控速目标位置修改为150°
    servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_velocity_base_target_position_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set velocity base target position successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为关闭
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_torque_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控制模式修改为控时模式
    servo_set_control_mode(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_control_mode_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set control mode successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为开启
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_torque_switch_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控时目标加速度等级修改为5
    servo_set_time_base_target_acc(1, 5, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_time_base_target_acc_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set time base target acc successfully.\r\n");

		HAL_Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控时目标位置和运行时间，分别修改为300°、500ms
    servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_set_time_base_target_position_and_moving_time_analysis(receive);
    if (ret == SUCCESS)
        PRINTF("servo set time base target position and moving time successfully.\r\n");

		HAL_Delay(1000);
#endif
		
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

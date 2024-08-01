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

#define READ_TEST 0				    // 读取舵机数据测试
#define WRITE_TEST 0			    // 写入舵机数据测试
#define SYNC_WRITE_TEST 0		    // 同步写测试
#define FACTORY_RESET_TEST 0	    // 恢复出厂设置测试
#define PARAMETER_RESET_TEST 0	    // 参数重置测试
#define CALIBRATION_TEST 0		    // 校正偏移值测试
#define REBOOT_TEST 0			    // 重启测试
#define MODIFY_ID 0                 // 修改舵机ID测试
#define MODIFY_UNKNOWN_ID 0         // 修改未知ID舵机ID测试


uint8_t order_buffer[20];
uint8_t order_len;
uint8_t receive[20];
uint16_t analysis_data;
uint8_t ret; 

uint16_t sync_write_velocity_base_target_position[5] = {1, 0, 2, 0};                    //同步写多个舵机控速目标位置
uint16_t sync_write_velocity_base_target_velocity[5] = {1, 3600, 2, 3600};              //同步写多个舵机控速目标速度
uint16_t sync_write_velocity_base_target_acc[5] = {1, 150, 2, 150};                     //同步写多个舵机控速目标加速度
uint16_t sync_write_velocity_base_target_dec[5] = {1, 150, 2, 150};                     //同步写多个舵机控速目标减速度
uint16_t sync_write_time_base_target_acc[5] = {1, 0, 2, 0};                             //同步写多个舵机控时目标加速度
uint16_t sync_write_time_base_target_position_and_moving_time[10] = {1, 3000, 500, 2, 3000, 500};           //同步写多个舵机控时目标运动位置和运动时间


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
#if FACTORY_RESET_TEST
		//恢复出厂设置
    servo_factory_reset(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("The servo factory reset");
		HAL_Delay(1000);
#endif			
		
#if PARAMETER_RESET_TEST
		//参数重置
    servo_parameter_reset(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("The servo parameter reset");
		HAL_Delay(1000);
#endif			

#if CALIBRATION_TEST
		//校正偏移 ?
    servo_calibration(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		PRINTF("The servo calibration");
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
		//将ID ?1的舵机修改为2
    servo_modify_known_id(1, 2, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
    PRINTF("modify id success");
		HAL_Delay(1000);
#endif

#if MODIFY_ID
		//将未知ID舵机修改为2
    servo_modify_unknown_id(2, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
    PRINTF("modify id success");
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//向ID为1的舵机发送PING指令
    servo_ping(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    ret = servo_ping_analysis(receive, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("The servo exists");
		HAL_Delay(1000);
#endif		
		
#if READ_TEST
		//读取舵机的当前位置
    servo_read_present_position(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

		HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_present_position_analysis(receive, &analysis_data);
    PRINTF("present position is %d",analysis_data);
		HAL_Delay(1000);
#endif
		
#if READ_TEST
		//读取舵机的当前电流
    servo_read_present_current(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UART_Receive_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_present_current_analysis(receive, &analysis_data);
    PRINTF("present current is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取舵机的当前速度
    servo_read_present_velocity(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);
		
    servo_read_present_velocity_analysis(receive, &analysis_data);
    PRINTF("present velocity is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的当前的规划位置
    servo_read_present_profile_position(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_present_profile_position_analysis(receive, &analysis_data);
    PRINTF("present profile position is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的当前规划速度
    servo_read_present_profile_velocity(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_present_profile_velocity_analysis(receive, &analysis_data);
    PRINTF("present profile velocity is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的当前PWM
    servo_read_present_pwm(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_present_pwm_analysis(receive, &analysis_data);
    PRINTF("present pwm analysis is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的当前温度
    servo_read_present_temperature(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_present_temperature_analysis(receive, &analysis_data);
    PRINTF("present temperature is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的当前输入电压
    servo_read_present_voltage(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_present_voltage_analysis(receive, &analysis_data);
    PRINTF("present voltage is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的控时目标运行时间
    servo_read_time_base_target_moving_time(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_time_base_target_moving_time_analysis(receive, &analysis_data);
    PRINTF("present time base target moving time is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的控时目标位置
    servo_read_time_base_target_position(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_time_base_target_position_analysis(receive, &analysis_data);
    PRINTF("present time base target position is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取舵机的控时加速度等级
    servo_read_time_base_target_acc(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_time_base_target_acc_analysis(receive, &analysis_data);
    PRINTF("present time base target acc is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的控速目标减速度
    servo_read_velocity_base_target_dec(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);
		
    servo_read_velocity_base_target_dec_analysis(receive, &analysis_data);
    PRINTF("present velocity base target dec is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的控速目标加速度
    servo_read_velocity_base_target_acc(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_velocity_base_target_acc_analysis(receive, &analysis_data);
    PRINTF("present velocity base target acc is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的控速目标速度
    servo_read_velocity_base_target_velocity(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_velocity_base_target_velocity_analysis(receive, &analysis_data);
    PRINTF("present velocity base target velocity is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的控速目标位置
    servo_read_velocity_base_target_position(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_velocity_base_target_position_analysis(receive, &analysis_data);
    PRINTF("present velocity base target position is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的目标电流
    servo_read_target_current(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_target_current_analysis(receive, &analysis_data);
    PRINTF("target current is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的目标PWM
    servo_read_target_pwm(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_target_pwm_analysis(receive, &analysis_data);
    PRINTF("target pwm is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的扭矩开关
    servo_read_torque_switch(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_torque_switch_analysis(receive, &analysis_data);
    PRINTF("torque switch is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的LED开关
    servo_read_led_switch(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_led_switch_analysis(receive, &analysis_data);
    PRINTF("led switch is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取舵机的Flash开关
    servo_read_flash_switch(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_flash_switch_analysis(receive, &analysis_data);
    PRINTF("flash switch is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的电流校正值
    servo_read_current_offset(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_current_offset_analysis(receive, &analysis_data);
    PRINTF("current offset is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的中位校正值
    servo_read_calibration(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_calibration_analysis(receive, &analysis_data);
    PRINTF("calibration is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的控制模式
    servo_read_control_mode(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_control_mode_analysis(receive, &analysis_data);
    PRINTF("control mode is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的卸载保护条件
    servo_read_shutdown_condition(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_shutdown_condition_analysis(receive, &analysis_data);
    PRINTF("shutdown condition is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的LED报警条件
    servo_read_led_condition(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_led_condition_analysis(receive, &analysis_data);
    PRINTF("led condition is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的位置控制D增益
    servo_read_position_control_d_gain(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_position_control_d_gain_analysis(receive, &analysis_data);
    PRINTF("position control d gain is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的位置控制I增益
    servo_read_position_control_i_gain(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_position_control_i_gain_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("position control i gain is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的位置控制P增益
    servo_read_position_control_p_gain(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_position_control_p_gain_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("position control p gain is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取舵机的PWM叠加值
    servo_read_pwm_punch(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_pwm_punch_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("pwm punch is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的反转死区
    servo_read_ccw_deadband(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_ccw_deadband_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("ccw deadband is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的正转死区
    servo_read_cw_deadband(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_cw_deadband_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("cw deadband is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的电流保护时间
    servo_read_current_shutdown_time(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_current_shutdown_time_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("current shutdown time is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的电流上限
    servo_read_max_current_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_max_current_limit_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("max current limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的PWM上限
    servo_read_max_pwm_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_max_pwm_limit_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("max pwm limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的电压上限
    servo_read_max_voltage_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_max_voltage_limit_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("max voltage limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的电压下限
    servo_read_min_voltage_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_min_voltage_limit_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("min voltage limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的温度上限
    servo_read_max_temperature_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);


    servo_read_max_temperature_limit_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("max temperature limit is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if READ_TEST
		//读取舵机的最大位置限制
    servo_read_max_angle_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);


    servo_read_max_angle_limit_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("max angle limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的最小位置限制
    servo_read_min_angle_limit(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_min_angle_limit_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("min angle limit is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的状态返回级别
    servo_read_return_level(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_return_level_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("return level is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的应答延时时间
    servo_read_return_delay_time(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_return_delay_time_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("return delay time is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的波特率
    servo_read_baud_rate(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_baud_rate_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("baud rate is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的出厂编号
    servo_read_model_information(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_model_information_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("model information is %d",analysis_data);
		HAL_Delay(1000);
#endif


#if READ_TEST
		//读取舵机的固件版本号
    servo_read_firmware_version(1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_read_firmware_version_analysis(receive, &analysis_data);
		HAL_HalfDuplex_EnableTransmitter(&huart1);

    PRINTF("firmware version is %d",analysis_data);
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的控制模式
    servo_set_control_mode(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(2, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的控制模式
    servo_set_control_mode(2, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(2, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif
		
#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标速度
		servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标加速度
		servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标减速度
		servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标位置
		servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 100);
		
		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(1, 0, order_buffer,&order_len);
		HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的控制模式
    servo_set_control_mode(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(2, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的控制模式
    servo_set_control_mode(2, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(2, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控时目标加速度等级
    servo_sync_write_time_base_target_acc(2, sync_write_time_base_target_acc, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 20);

		HAL_Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控时目标位置和运动时间
    servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 20);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的状态返回级别
    servo_set_return_level(1, 2, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_return_level_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的应答延时时间
		servo_set_return_delay_time(1, 250, order_buffer, &order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_return_delay_time_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的波特率
    servo_set_baud_rate(1, 7, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_baud_rate_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的最小位置限制
    servo_set_min_angle_limit(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_min_angle_limit_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的最大位置限制
    servo_set_max_angle_limit(1, 3000, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_max_angle_limit_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的温度上限
    servo_set_max_temperature_limit(1, 100, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_max_temperature_limit_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的电压上限
    servo_set_max_voltage_limit(1,90, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_max_voltage_limit_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的电压下限
    servo_set_min_voltage_limit(1, 33, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_min_voltage_limit_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的PWM上限
    servo_set_max_pwm_limit(1, 1000, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_max_pwm_limit_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的电流上限
    servo_set_max_current_limit(1, 400, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_max_current_limit_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的电流保护时间
    servo_set_current_shutdown_time(1, 1000, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_current_shutdown_time_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的正转死区
    servo_set_cw_deadband(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_cw_deadband_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的反转死区
    servo_set_ccw_deadband(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_ccw_deadband_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的PWM叠加值
    servo_set_pwm_punch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_pwm_punch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的位置控制P增益
    servo_set_position_control_p_gain(1, 6000, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_position_control_p_gain_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的位置控制I增益
    servo_set_position_control_i_gain(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_position_control_i_gain_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的位置控制D增益
    servo_set_position_control_d_gain(1, 150, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_position_control_d_gain_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的LED报警条件
    servo_set_led_condition(1, 36, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_led_condition_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的卸载保护条件
    servo_set_shutdown_conditions(1, 36, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_shutdown_conditions_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的Flash开关
    servo_set_flash_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_flash_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的LED开关
    servo_set_led_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_led_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控制模式
    servo_set_control_mode(1, 3, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控制模式
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的目标PWM
    servo_set_target_pwm(1, 1000, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_target_pwm_analysis(receive);

		HAL_Delay(3000);
#endif

#if WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控制模式
    servo_set_control_mode(1, 2, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控制模式
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的目标电流
    servo_set_target_current(1, -1000, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_target_current_analysis(receive);

		HAL_Delay(3000);
#endif

#if WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控制模式
    servo_set_control_mode(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控制模式
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控速目标速度
    servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_velocity_base_target_velocity_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控速目标加速度
    servo_set_velocity_base_target_acc(1, 150, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_velocity_base_target_acc_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控速目标减速度
    servo_set_velocity_base_target_dec(1, 150, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_velocity_base_target_dec_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控速目标位置
    servo_set_velocity_base_target_position(1, 0, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_velocity_base_target_position_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的扭矩开关
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控制模式
    servo_set_control_mode(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_control_mode_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控制模式
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_torque_switch_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控时目标加速度等级
    servo_set_time_base_target_acc(1, 0, order_buffer,&order_len);

    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_time_base_target_acc_analysis(receive);

		HAL_Delay(1000);
#endif

#if WRITE_TEST
		//设置舵机的控时目标位置和目标运行时间
    servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_len);
   
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, order_buffer, order_len, 10);

    HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, receive, 50);
		
		HAL_Delay(10);

    servo_set_time_base_target_position_and_moving_time_analysis(receive);

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
		//再次开启空闲中断接收，不然只会接收1次数据
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

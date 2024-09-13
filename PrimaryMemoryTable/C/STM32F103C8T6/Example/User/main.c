#include "stm32f10x.h"
#include "platform_config.h"
#include "servo.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

#define PING_TEST 0   						// PING指令测试
#define READ_TEST 0								// 读取舵机数据测试
#define WRITE_TEST 0							// 写入舵机数据测试
#define SYNC_WRITE_TEST 0					// 同步写测试
#define FACTORY_RESET_TEST 0			// 恢复出厂设置测试
#define PARAMETER_RESET_TEST 0	  // 参数重置测试
#define CALIBRATION_TEST 0				// 校正偏移值测试
#define REBOOT_TEST 0							// 重启测试
#define MODIFY_ID 0               // 修改已知舵机ID测试
#define MODIFY_UNKNOWN_ID 0       // 修改未知ID舵机ID测试

//数据接收
uint8_t receive_data[20];
uint8_t receive_len;
uint8_t ret;
uint16_t position = 0;                         //当前位置
uint16_t current = 0;                          //当前电流
uint8_t write_buffer[20] = {0};                //写入内存表数据

extern __IO uint32_t TimingDelay;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Init(void);
void USART2_Init(void);
void USART1_Send(uint8_t *data, uint8_t data_len);
void SysTick_Configuration(void);
void Delay(__IO uint32_t nTime);

//重定向printf
int fputc(int ch, FILE *f)
{
  USART_SendData(USART2, (uint8_t) ch);

  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}
		
	return ch;
}

int main(void)
{
	//串口发送数据以及长度
	uint8_t order_buffer[20];
	uint8_t order_len;
	
	//解析应答包得到的数据
	uint16_t analysis_data;
	
	//系统配置
	SysTick_Configuration();
	
	//RCC时钟配置
	RCC_Configuration();
	
	//GPIO配置
	GPIO_Configuration();
	
	//与舵机通信串口初始化
	USART1_Init();
	
	//信息打印串口初始化
	USART2_Init();
	
	struct servo_sync_parameter servo;
	
	servo.id_counts = 2;            //同步写两个舵机
  servo.id[0] = 1;                //第一个舵机id为1
  servo.id[1] = 2;                //第二个舵机id为2
	
  while (1)
  {
#if FACTORY_RESET_TEST
		//恢复出厂设置
    servo_factory_reset(1, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_factory_reset_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("FACTORY RESET success");
		Delay(1000);
#endif			
		
#if PARAMETER_RESET_TEST
		//参数重置
    servo_parameter_reset(1, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_parameter_reset_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("PARAMETER RESET SUCCESS");
		Delay(1000);
#endif			

#if CALIBRATION_TEST
		//校正偏移值
    servo_calibration(1, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
    ret = servo_calibration_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("CALIBRATION SUCCESS");
		Delay(1000);
#endif				
		
#if REBOOT_TEST
		//ID1舵机重启
    servo_reboot(1, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("The servo reboot");
		
		Delay(1000);
#endif			

#if MODIFY_ID
		//将ID为1的舵机修改为ID2
    servo_modify_known_id(1, 2, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		PRINTF("modify id success");
		Delay(1000);
#endif

#if MODIFY_UNKNOWN_ID
		//将未知ID舵机的ID编号修改为1
    servo_modify_unknown_id(1, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		PRINTF("modify id success");
		Delay(1000);
#endif

#if PING_TEST
		//生成指令
    servo_ping(1, order_buffer,&order_len);
    
		//串口发送
		USART1_Send(order_buffer, order_len);
		
		//使能中断接收数据
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		
		//接收数据清零
		receive_len = 0x00;
		Delay(10);
		
		//关闭接收中断
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
		//解析接收到的数据
    ret = servo_ping_analysis(receive_data, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("The servo exists");
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的当前电流
    servo_read_present_current(1, order_buffer,&order_len);
    
		USART1_Send(order_buffer, order_len);
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
    ret = servo_read_present_current_analysis(receive_data, &analysis_data);
    if(ret == SUCCESS)
		{
			PRINTF("present current is %d",analysis_data);
		}
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的当前位置
    servo_read_present_position(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_position_analysis(receive_data, &analysis_data);
    if(ret == SUCCESS)
		{
			PRINTF("present position is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //读取ID1舵机的当前位置和当前电流
    servo_read_present_position_and_present_current(1, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_position_and_present_current_analysis(receive_data, &position, &current);
    if(ret == SUCCESS)
		{
			PRINTF("present position is : % d, present current is : % d\r\n", position, current);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前速度
    servo_read_present_velocity(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_velocity_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前的规划位置
    servo_read_present_profile_position(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_profile_position_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present profile position is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前规划速度
    servo_read_present_profile_velocity(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present profile velocity is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前PWM
    servo_read_present_pwm(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_pwm_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present pwm analysis is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前温度
    servo_read_present_temperature(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_temperature_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present temperature is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的当前输入电压
    servo_read_present_voltage(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_voltage_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present voltage is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控时目标运行时间
    servo_read_time_base_target_moving_time(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present time base target moving time is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控时目标位置
    servo_read_time_base_target_position(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present time base target position is %d",analysis_data);
		}
		
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的控时加速度等级
    servo_read_time_base_target_acc(1, order_buffer,&order_len);   
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present time base target acc is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //读取ID1舵机的控时目标位置和运行时间
    servo_read(1, 0x3C, 4, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		PRINTF("the time base target position and moving time pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
    //读取ID1舵机的控时目标加速度等级、位置和运行时间
    servo_read(1, 0x3B, 5, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		PRINTF("the time base target acc, position and moving time pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的控速目标减速度
    servo_read_velocity_base_target_dec(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
    ret = servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity base target dec is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控速目标加速度
    servo_read_velocity_base_target_acc(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity base target acc is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控速目标速度
    servo_read_velocity_base_target_velocity(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity base target velocity is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控速目标位置
    servo_read_velocity_base_target_position(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity base target position is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //读取ID1舵机的控速目标位置和速度
    servo_read(1, 0x35, 4, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the velocity base target position and velocity pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
    //读取ID1舵机的控速目标位置、速度和加减速度
    servo_read(1, 0x35, 6, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the velocity base target position,velocity,acc and dec pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的目标电流
    servo_read_target_current(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_target_current_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("target current is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的目标PWM
    servo_read_target_pwm(1, order_buffer,&order_len);  
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_target_pwm_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("target pwm is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的扭矩开关
    servo_read_torque_switch(1, order_buffer,&order_len);  
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_torque_switch_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("torque switch is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的LED开关
    servo_read_led_switch(1, order_buffer,&order_len);   
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_led_switch_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("led switch is %d",analysis_data);
		}
		
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的Flash开关
    servo_read_flash_switch(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_flash_switch_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("flash switch is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电流校正值
    servo_read_current_offset(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_current_offset_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("current offset is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的中位校正值
    servo_read_calibration(1, order_buffer,&order_len);  
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_calibration_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("calibration is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的控制模式
    servo_read_control_mode(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_control_mode_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("control mode is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的卸载保护条件
    servo_read_shutdown_condition(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("shutdown condition is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的LED报警条件
    servo_read_led_condition(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_led_condition_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("led condition is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的位置控制D增益
    servo_read_position_control_d_gain(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("position control d gain is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的位置控制I增益
    servo_read_position_control_i_gain(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("position control i gain is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的位置控制P增益
    servo_read_position_control_p_gain(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("position control p gain is %d",analysis_data);
		}
		
		Delay(1000);
#endif
			
#if READ_TEST
    //读取ID1舵机的位置控制PID增益
    servo_read(1, 0x1B, 6, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("position control pid gain pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的PWM叠加值
    servo_read_pwm_punch(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_pwm_punch_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("pwm punch is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的反转死区
    servo_read_ccw_deadband(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("ccw deadband is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的正转死区
    servo_read_cw_deadband(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_cw_deadband_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("cw deadband is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电流保护时间
    servo_read_current_shutdown_time(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("current shutdown time is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电流上限
    servo_read_max_current_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_max_current_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max current limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的PWM上限
    servo_read_max_pwm_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max pwm limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电压上限
    servo_read_max_voltage_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max voltage limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电压下限
    servo_read_min_voltage_limit(1, order_buffer,&order_len);   
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("min voltage limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //读取ID1舵机的电压限制
    servo_read(1, 0x10, 2, order_buffer, &order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the voltage limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的温度上限
    servo_read_max_temperature_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
    ret = servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max temperature limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的最大位置限制
    servo_read_max_angle_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max angle limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的最小位置限制
    servo_read_min_angle_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("min angle limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //读取ID1舵机的位置限制
    servo_read(1, 0x0B, 4, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the angle limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
		//读取ID1舵机的状态返回级别
    servo_read_return_level(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_return_level_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("return level is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的应答延时时间
    servo_read_return_delay_time(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_return_delay_time_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("return delay time is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的波特率
    servo_read_baud_rate(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_baud_rate_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("baud rate is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的出厂编号
    servo_read_model_information(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_model_information_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("model information is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的固件版本号
    servo_read_firmware_version(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_firmware_version_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("firmware version is %d",analysis_data);
		}
		
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
		servo.torque_switch[0] = 0;
		servo.torque_switch[1] = 0;
		servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write torque switch!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //将ID1、ID2舵机的控制模式，分别修改为控速模式
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write control mode!");
		Delay(1000);
#endif

	
#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标速度
		
		//id为1，2的舵机速度分别设置为3600，1800，值和前面的id设置对应
    servo.velocity[0] = 3600;
    servo.velocity[1] = 1800;
		
		servo_sync_write_velocity_base_target_velocity(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target velocity!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标加速度
		
		//id为1，2的舵机加速度分别设置为150，150，值和前面的id设置对应
    servo.acc_velocity[0] = 150;          
    servo.acc_velocity[1] = 150;    
		
		servo_sync_write_velocity_base_target_acc(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target acc!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标减速度
		
		//id为1，2的舵机减速度分别设置为150，150，值和前面的id设置对应
    servo.dec_velocity[0] = 150;           
    servo.dec_velocity[1] = 150;   
		
		servo_sync_write_velocity_base_target_dec(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target dec!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标位置
		
		//id为1，2的舵机运动位置分别设置为0，0，值和前面的id设置对应
    servo.position[0] = 0;
    servo.position[1] = 0;
		
		servo_sync_write_velocity_base_target_position(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target position!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标位置和速度
		
		//id为1，2的舵机速度分别设置为1800，3600，位置分别设置为3000，3000
    servo.velocity[0] = 1800;
    servo.velocity[1] = 3600;
    servo.position[0] = 3000;
    servo.position[1] = 3000;
		
		servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target position and velocity!");
		Delay(1000);
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
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target acc dec velocity and position!");
		Delay(1000);

#endif

#if SYNC_WRITE_TEST
    //将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
		servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write torque switch!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //将ID1、ID2舵机的控制模式，分别修改为控时模式
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write control mode!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控时目标加速度等级
		
		//设置舵机id为1，2的加速度等级分别为0，0
    servo.acc_velocity_grade[0] = 0;
    servo.acc_velocity_grade[1] = 0;
		
    servo_sync_write_time_base_target_acc(servo, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);
		PRINTF("sync write time base target acc!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控时目标位置和运动时间
		
		//设置舵机id为1，2的运动位置为3000，3000，运动时间为500ms，1500ms
    servo.position[0] = 3000;
    servo.position[1] = 3000;
    servo.time[0] = 500;
    servo.time[1] = 1500;
		
		servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write time base target position and moving time!");
		Delay(1000);
#endif

#if WRITE_TEST
		//将ID1舵机的状态返回级别修改为应答所有指令
    servo_set_return_level(1, 2, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_return_level_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set return level success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的应答延迟时间修改为500us
		servo_set_return_delay_time(1, 250,order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_return_delay_time_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set return delay time success");
		Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的波特率为1000000
    servo_set_baud_rate(1, 7, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_baud_rate_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set baud rate success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将舵机ID1的最小位置限制修改为0°
    servo_set_min_angle_limit(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_min_angle_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set min angle limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将舵机ID1的最大位置限制修改为300°
    servo_set_max_angle_limit(1, 3000, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_angle_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max angle limit success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //将舵机ID1的位置限制修改为0°~300°
    write_buffer[0] = 0 & 0xff;;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3000 & 0xff;
    write_buffer[3] = (3000 >> 8) & 0xff;
		servo_write(1, 0x0B, 4, write_buffer, order_buffer, &order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("servo set angle limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的温度上限修改为65℃
    servo_set_max_temperature_limit(1, 65, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_temperature_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max temperature limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电压上限修改为8.4V
    servo_set_max_voltage_limit(1,84, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_voltage_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max voltage limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电压下限修改为3.5V
    servo_set_min_voltage_limit(1, 35, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_min_voltage_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set min voltage limit success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //将ID1舵机的电压限制修改为3.5V~8.4V
    write_buffer[0] = 84 & 0xff;
    write_buffer[1] = 35 & 0xff;
    servo_write(1, 0x10, 2, write_buffer, order_buffer, &order_len);

		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the voltage limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的PWM上限修改为90%
    servo_set_max_pwm_limit(1, 900, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_pwm_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max pwm limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电流上限修改为900mA
    servo_set_max_current_limit(1, 900, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_current_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max current limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的电流保护时间修改为500ms
    servo_set_current_shutdown_time(1, 500, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_current_shutdown_time_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set current shutdown time success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的正转死区修改为0.2°
    servo_set_cw_deadband(1, 2, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_cw_deadband_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set cw deadband success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的正反转死区修改为0.2°
    write_buffer[0] = 2 & 0xff;
    write_buffer[1] = 2 & 0xff;
    servo_write(1, 0x18, 2, write_buffer, order_buffer, &order_len);
		
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("servo set cw deadband and ccw deadband pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的PWM叠加值修改为1%
    servo_set_pwm_punch(1, 10, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_pwm_punch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set pwm punch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的位置控制P增益修改为5995
    servo_set_position_control_p_gain(1, 5995, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_position_control_p_gain_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set position control p gain success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的位置控制I增益修改为5
    servo_set_position_control_i_gain(1, 5, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_position_control_i_gain_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set position control i gain success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的位置控制D增益修改为145
    servo_set_position_control_d_gain(1, 145, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_position_control_d_gain_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set position control d gain success");
		Delay(1000);
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
		
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("servo set position control pid gain pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的LED报警条件修改为开启堵转报错、过热报错和角度报错
    servo_set_led_condition(1, 38, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_led_condition_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set led condition success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的卸载保护条件修改为开启堵转报错、过热报错、电压报错和角度报错
    servo_set_shutdown_conditions(1, 39, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_shutdown_conditions_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set shutdown conditions success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的Flash开关状态修改为打开
    servo_set_flash_switch(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_flash_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set flash switch success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //将ID1舵机的Flash开关状态修改为关闭
    servo_set_flash_switch(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_flash_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set flash switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的LED开关状态修改为打开
    servo_set_led_switch(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_led_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set led switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的LED开关状态修改为关闭
    servo_set_led_switch(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_led_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set led switch success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //设置ID1舵机的扭矩开关为关闭
    servo_set_torque_switch(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_torque_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set torque switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控制模式修改为PWM输出控制模式
    servo_set_control_mode(1, 3, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_control_mode_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set control mode success");
		Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为开启
    servo_set_torque_switch(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_torque_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set torque switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的目标PWM修改为-50%
    servo_set_target_pwm(1, -500, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_target_pwm_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set target pwm success");
		Delay(3000);
#endif
		
		#if WRITE_TEST
    //设置ID1舵机的扭矩开关为关闭
    servo_set_torque_switch(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_torque_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set torque switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控制模式修改为电流控制模式
    servo_set_control_mode(1, 2, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_control_mode_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set control mode success");
		Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为开启
    servo_set_torque_switch(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_torque_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set torque switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的目标电流修改为-400mA
    servo_set_target_current(1, -400, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_target_current_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set target current success");
		Delay(3000);
#endif
		
#if WRITE_TEST
    //设置ID1舵机的扭矩开关为关闭
    servo_set_torque_switch(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_torque_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set torque switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控制模式修改为控速模式
    servo_set_control_mode(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_control_mode_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set control mode success");
		Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为开启
    servo_set_torque_switch(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_torque_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set torque switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控速目标速度修改为360°/s
    servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_velocity_base_target_velocity_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set velocity base target velocity success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控速目标加速度修改为500°/s²
    servo_set_velocity_base_target_acc(1, 10, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_velocity_base_target_acc_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set velocity base target acc success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控速目标减速度修改为50°/s²
    servo_set_velocity_base_target_dec(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_velocity_base_target_dec_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set velocity base target dec success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控速目标位置修改为150°
    servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_velocity_base_target_position_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set velocity base target position success");
		Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为关闭
    servo_set_torque_switch(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_torque_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set torque switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控制模式修改为控时模式
    servo_set_control_mode(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_control_mode_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set control mode success");
		Delay(1000);
#endif

#if WRITE_TEST
    //设置ID1舵机的扭矩开关为开启
    servo_set_torque_switch(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_torque_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set torque switch success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //将ID1舵机的控时目标加速度等级修改为5
    servo_set_time_base_target_acc(1, 5, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_time_base_target_acc_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set time base target acc success");
		Delay(1000);
#endif

#if WRITE_TEST
    //将ID1舵机的控时目标位置和运行时间，分别修改为300°、500ms
    servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_time_base_target_position_and_moving_time_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set time base target position and moving time success");
		Delay(1000);
#endif
	}
}

void RCC_Configuration(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

	//USART1   PA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
  //USART2_TX   PA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2
 
	//USART2_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA3
}

void USART1_Init()
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  USART_InitStructure.USART_BaudRate = 1000000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
	
	// 配置中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 启动中断向量表
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	// 使能USART1的半双工模式
  USART_HalfDuplexCmd(USART1, ENABLE);

  // 使能USART1
  USART_Cmd(USART1, ENABLE);
}

void USART2_Init()
{
	USART_InitTypeDef USART_InitStructure;
	
	USART_DeInit(USART2);  //复位串口2
 
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = 1000000;//默认设置为1000000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
 
	USART_Init(USART2, &USART_InitStructure); //初始化串口
	
	USART_Cmd(USART2, ENABLE);                    //使能串口 
}

void USART1_Send(uint8_t *data, uint8_t data_len)
{
  for(uint8_t i = 0; i < data_len; i++)
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

		// 发送数据
		USART_SendData(USART1, data[i]);
	}
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
  /* Set SysTick Priority to 3 */
  NVIC_SetPriority(SysTick_IRQn, 0x0C);
}

void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

#include "stm32f10x.h"
#include "platform_config.h"
#include "servo.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

//调试变量定义
#define PING_TEST 0   			//PING指令测试
#define READ_TEST 0					// 读取舵机数据测试
#define WRITE_TEST 0				// 写入舵机数据测试
#define SYNC_WRITE_TEST 0			// 同步写测试
#define FACTORY_RESET_TEST 0		// 恢复出厂设置测试
#define PARAMETER_RESET_TEST 0	    // 参数重置测试
#define CALIBRATION_TEST 0			// 校正偏移值测试
#define REBOOT_TEST 0				// 重启测试
#define MODIFY_ID 0                 // 修改舵机ID测试
#define MODIFY_UNKNOWN_ID 0         // 修改未知ID舵机ID测试

//数据接收
uint8_t receive_data[20];
uint8_t receive_len;
uint8_t ret;

extern __IO uint32_t TimingDelay;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Init(void);
void USART2_Init(void);
void USART1_Send(uint8_t *data, uint8_t data_len);
void SysTick_Configuration(void);
void Delay(__IO uint32_t nTime);

uint16_t sync_write_velocity_base_target_position[5] = {1, 0, 2, 0};                    //同步写多个舵机控速目标位置
uint16_t sync_write_velocity_base_target_velocity[5] = {1, 3600, 2, 3600};              //同步写多个舵机控速目标速度
uint16_t sync_write_velocity_base_target_acc[5] = {1, 150, 2, 150};                     //同步写多个舵机控速目标加速度
uint16_t sync_write_velocity_base_target_dec[5] = {1, 150, 2, 150};                     //同步写多个舵机控速目标减速度
uint16_t sync_write_time_base_target_acc[5] = {1, 0, 2, 0};                             //同步写多个舵机控时目标加速度
uint16_t sync_write_time_base_target_position_and_moving_time[10] = {1, 1500, 500, 2, 1500, 500};           //同步写多个舵机控时目标运动位置和运动时间

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
		//将未知舵机的ID修改为2
    servo_modify_unknown_id(2, order_buffer,&order_len);

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
		
    servo_read_present_current_analysis(receive_data, &analysis_data);
    PRINTF("present current is %d",analysis_data);
		
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

    servo_read_present_position_analysis(receive_data, &analysis_data);
    PRINTF("present position is %d",analysis_data);
		
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

    servo_read_present_velocity_analysis(receive_data, &analysis_data);
    PRINTF("present velocity is %d",analysis_data);
		
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

    servo_read_present_profile_position_analysis(receive_data, &analysis_data);
    PRINTF("present profile position is %d",analysis_data);
		
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

    servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
    PRINTF("present profile velocity is %d",analysis_data);
		
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

    servo_read_present_pwm_analysis(receive_data, &analysis_data);
    PRINTF("present pwm analysis is %d",analysis_data);
		
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

    servo_read_present_temperature_analysis(receive_data, &analysis_data);
    PRINTF("present temperature is %d",analysis_data);
		
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

    servo_read_present_voltage_analysis(receive_data, &analysis_data);
    PRINTF("present voltage is %d",analysis_data);
		
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

    servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
    PRINTF("present time base target moving time is %d",analysis_data);
		
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

    servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
    PRINTF("present time base target position is %d",analysis_data);
		
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

    servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
    PRINTF("present time base target acc is %d",analysis_data);
		
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
		
    servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
    PRINTF("present velocity base target dec is %d",analysis_data);
		
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

    servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
    PRINTF("present velocity base target acc is %d",analysis_data);
		
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

    servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
    PRINTF("present velocity base target velocity is %d",analysis_data);
		
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

    servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
    PRINTF("present velocity base target position is %d",analysis_data);
		
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

    servo_read_target_current_analysis(receive_data, &analysis_data);
    PRINTF("target current is %d",analysis_data);
		
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

    servo_read_target_pwm_analysis(receive_data, &analysis_data);
    PRINTF("target pwm is %d",analysis_data);
		
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

    servo_read_torque_switch_analysis(receive_data, &analysis_data);
    PRINTF("torque switch is %d",analysis_data);
		
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

    servo_read_led_switch_analysis(receive_data, &analysis_data);
    PRINTF("led switch is %d",analysis_data);
		
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

    servo_read_flash_switch_analysis(receive_data, &analysis_data);
    PRINTF("flash switch is %d",analysis_data);
		
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

    servo_read_current_offset_analysis(receive_data, &analysis_data);
    PRINTF("current offset is %d",analysis_data);
		
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

    servo_read_calibration_analysis(receive_data, &analysis_data);
    PRINTF("calibration is %d",analysis_data);
		
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

    servo_read_control_mode_analysis(receive_data, &analysis_data);
    PRINTF("control mode is %d",analysis_data);
		
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

    servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
    PRINTF("shutdown condition is %d",analysis_data);
		
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

    servo_read_led_condition_analysis(receive_data, &analysis_data);
    PRINTF("led condition is %d",analysis_data);
		
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

    servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
    PRINTF("position control d gain is %d",analysis_data);
		
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

    servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
    PRINTF("position control i gain is %d",analysis_data);
		
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

    servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
    PRINTF("position control p gain is %d",analysis_data);
		
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

    servo_read_pwm_punch_analysis(receive_data, &analysis_data);
    PRINTF("pwm punch is %d",analysis_data);
		
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

    servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
    PRINTF("ccw deadband is %d",analysis_data);
		
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

    servo_read_cw_deadband_analysis(receive_data, &analysis_data);
    PRINTF("cw deadband is %d",analysis_data);
		
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

    servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
    PRINTF("current shutdown time is %d",analysis_data);
		
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

    servo_read_max_current_limit_analysis(receive_data, &analysis_data);
    PRINTF("max current limit is %d",analysis_data);
		
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

    servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
    PRINTF("max pwm limit is %d",analysis_data);
		
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

    servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
    PRINTF("max voltage limit is %d",analysis_data);
		
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

    servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
    PRINTF("min voltage limit is %d",analysis_data);
		
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
		
    servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
    PRINTF("max temperature limit is %d",analysis_data);
		
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

    servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
    PRINTF("max angle limit is %d",analysis_data);
		
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

    servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
    PRINTF("min angle limit is %d",analysis_data);
		
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

    servo_read_return_level_analysis(receive_data, &analysis_data);
    PRINTF("return level is %d",analysis_data);
		
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

    servo_read_return_delay_time_analysis(receive_data, &analysis_data);
    PRINTF("return delay time is %d",analysis_data);
		
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

    servo_read_baud_rate_analysis(receive_data, &analysis_data);
    PRINTF("baud rate is %d",analysis_data);
		
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

    servo_read_model_information_analysis(receive_data, &analysis_data);
    PRINTF("model information is %d",analysis_data);
		
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

    servo_read_firmware_version_analysis(receive_data, &analysis_data);
    PRINTF("firmware version is %d",analysis_data);
		
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置ID1舵机的扭矩开关
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

#if SYNC_WRITE_TEST
		//设置ID1舵机的控制模式
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

#if SYNC_WRITE_TEST
		//设置ID1舵机的扭矩开关
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

#if SYNC_WRITE_TEST
		//设置ID2舵机的扭矩开关
    servo_set_torque_switch(2, 0, order_buffer,&order_len);
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

#if SYNC_WRITE_TEST
		//设置ID2舵机的控制模式
    servo_set_control_mode(2, 1, order_buffer,&order_len);
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

#if SYNC_WRITE_TEST
		//设置ID2舵机的扭矩开关
    servo_set_torque_switch(2, 1, order_buffer,&order_len);
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
	
#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标速度
		servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标加速度
		servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标减速度
		servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控速目标位置
		servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置ID1舵机的扭矩开关
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

#if SYNC_WRITE_TEST
		//设置ID1舵机的控制模式
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

#if SYNC_WRITE_TEST
		//设置ID1舵机的扭矩开关
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

#if SYNC_WRITE_TEST
		//设置ID2舵机的扭矩开关
    servo_set_torque_switch(2, 0, order_buffer,&order_len);
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

#if SYNC_WRITE_TEST
		//设置ID2舵机的控制模式
    servo_set_control_mode(2, 0, order_buffer,&order_len);
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

#if SYNC_WRITE_TEST
		//设置ID2舵机的扭矩开关
    servo_set_torque_switch(2, 1, order_buffer,&order_len);
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

#if SYNC_WRITE_TEST
		//设置多个舵机的控时目标加速度等级
    servo_sync_write_time_base_target_acc(2, sync_write_time_base_target_acc, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//设置多个舵机的控时目标位置和运动时间
		servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		Delay(1000);
#endif

#if WRITE_TEST
		//设置ID1舵机的状态返回级别
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
		//设置ID1舵机的应答延时时间
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
		//设置ID1舵机的波特率
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
		//设置ID1舵机的最小位置限制
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
		//设置ID1舵机的最大位置限制
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
		//设置ID1舵机的温度上限
    servo_set_max_temperature_limit(1, 100, order_buffer,&order_len);
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
		//设置ID1舵机的电压上限
    servo_set_max_voltage_limit(1,90, order_buffer,&order_len);
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
		//设置ID1舵机的电压下限
    servo_set_min_voltage_limit(1, 33, order_buffer,&order_len);
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
		//设置ID1舵机的PWM上限
    servo_set_max_pwm_limit(1, 1000, order_buffer,&order_len);
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
		//设置ID1舵机的电流上限
    servo_set_max_current_limit(1, 400, order_buffer,&order_len);
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
		//设置ID1舵机的电流保护时间
    servo_set_current_shutdown_time(1, 1000, order_buffer,&order_len);
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
		//设置ID1舵机的正转死区
    servo_set_cw_deadband(1, 1, order_buffer,&order_len);
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
		//设置ID1舵机的反转死区
    servo_set_ccw_deadband(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_ccw_deadband_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set ccw deadband success");
		Delay(1000);
#endif

#if WRITE_TEST
		//设置ID1舵机的PWM叠加值
    servo_set_pwm_punch(1, 1, order_buffer,&order_len);
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
		//设置ID1舵机的位置控制P增益
    servo_set_position_control_p_gain(1, 6000, order_buffer,&order_len);
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
		//设置ID1舵机的位置控制I增益
    servo_set_position_control_i_gain(1, 1, order_buffer,&order_len);
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
		//设置ID1舵机的位置控制D增益
    servo_set_position_control_d_gain(1, 151, order_buffer,&order_len);
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
		//设置ID1舵机的LED报警条件
    servo_set_led_condition(1, 36, order_buffer,&order_len);
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
		//设置ID1舵机的卸载保护条件
    servo_set_shutdown_conditions(1, 36, order_buffer,&order_len);
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
		//设置ID1舵机的Flash开关
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
		//设置ID1舵机的LED开关
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
		//设置ID1舵机的扭矩开关
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
		//设置ID1舵机的控制模式
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
		//设置ID1舵机的扭矩开关
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
		//设置ID1舵机的目标PWM
    servo_set_target_pwm(1, 1000, order_buffer,&order_len);
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
		//设置ID1舵机的扭矩开关
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
		//设置ID1舵机的控制模式
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
		//设置ID1舵机的扭矩开关
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
		//设置ID1舵机的目标电流
    servo_set_target_current(1, -1000, order_buffer,&order_len);
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
		//设置ID1舵机的扭矩开关
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
		//设置ID1舵机的控制模式
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
		//设置ID1舵机的扭矩开关
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
		//设置ID1舵机的控速目标速度
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
		//设置ID1舵机的控速目标加速度
    servo_set_velocity_base_target_acc(1, 150, order_buffer,&order_len);
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
		//设置ID1舵机的控速目标减速度
    servo_set_velocity_base_target_dec(1, 150, order_buffer,&order_len);
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
		//设置ID1舵机的控时目标加速度等级
    servo_set_time_base_target_acc(1, 0, order_buffer,&order_len);
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
		//设置ID1舵机的控速目标位置
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
		//设置ID1舵机的扭矩开关
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
		//设置ID1舵机的控制模式
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
		//设置ID1舵机的扭矩开关
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
		//设置ID1舵机的控时目标位置和目标运行时间
    servo_set_time_base_target_position_and_moving_time(1, 0, 500, order_buffer,&order_len);
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

#include "reg52.h"
#include "servo.c"

#define READ_TEST 0                 // 读取舵机数据测试
#define WRITE_TEST 0                // 写入舵机数据测试
#define SYNC_WRITE_TEST 0           // 同步写测试
#define PING_TEST 0                 // PING命令测试
#define FACTORY_RESET_TEST 0        // 恢复出厂设置测试
#define PARAMETER_RESET_TEST 0      // 参数重置测试
#define REBOOT_TEST 0               // 重启测试
#define CALIBRATION_TEST 0          // 校正偏移值测试
#define MODIFY_ID_TEST 0            // 修改舵机ID测试
#define MODIFY_UNKNOWN_ID_TEST 0    // 修改未知ID舵机ID测试

uint16_t ms_count;

xdata uint8_t receive_data[20] = {0};
xdata uint8_t receive_len = 0;          // 接收应答包长度

// time0初始化做延时
void timer0_init()
{
    TMOD |= 0x01;
    TH0 = 0xFC;
    TL0 = 0x67;
    ET0 = 1;
    EA = 1;
    TR0 = 1;
}

// 延时函数
void delay_ms(uint16_t ms)
{
    ms_count =  2 * ms;   // 因为开了双倍速6T，所以这里延时时间需要乘以2
    while (ms_count);
}

// 定时器中断函数
void timer0_isr() interrupt 1 using 1
{
    TH0 = 0xFC;
    TL0 = 0x67;
    if (ms_count)
        ms_count--;
}

// 串口初始化
void uart_init()
{
	TMOD|=0X20;	   // 8位自动重装载定时器
	SCON=0X40;	   // 8位UART，波特率可变
	PCON=0X80;	   // 波特率加倍
	TH1=0xff;	   // 设置波特率为115200
	TL1=0xff;
    ES=0;		   // 关闭接收中断
    EA=1;		   // CPU总中断
    TR1=1;		   // 开启定时器T1开始计数
}

void uart_init_recv()
{
    TMOD |= 0x20;  // 8位自动重装载定时器
    SCON = 0x50;   // 8位UART，波特率可变，并开启串行接收
    PCON = 0x80;   // 波特率加倍
    TH1 = 0xff;    // 设置波特率为115200
    TL1 = 0xff;
    ES = 1;        // 开启接收中断
    EA = 1;        // CPU总中断
    TR1 = 1;       // 开启定时器T1开始计数
}

// 串口发送函数
void uart_send(uint8_t order_data)
{
    SBUF = order_data;      // 将数据写入串口缓冲寄存器开始传输
    while(!TI);    			// 等待传输完成
    TI = 0;      			// 清除传输完成标志
}


void uart_send_buffer(uint8_t *buffer, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++) {
        uart_send(buffer[i]);
    }
}


void main()
{
	xdata uint8_t order_buffer[20];												                    // 存放生成的指令
	xdata uint8_t order_buffer_len = 0;										                        // 指令长度
	xdata uint16_t analysis_data = 0;											                    // 应答包解析出来的数据
	xdata uint16_t sync_write_velocity_base_target_position[4] = {1, 0, 2, 0};                       // 同步写多个舵机控速目标位置
	xdata uint16_t sync_write_velocity_base_target_velocity[5] = {1, 3600, 2, 3600};                 // 同步写多个舵机控速目标速度
	xdata uint16_t sync_write_velocity_base_target_acc[5] = {1, 150, 2, 150};                        // 同步写多个舵机控速目标加速度
	xdata uint16_t sync_write_velocity_base_target_dec[5] = {1, 150, 2, 150};                        // 同步写多个舵机控速目标减速度
	xdata uint16_t sync_write_time_base_target_acc[5] = {1, 0, 2, 0};                                // 同步写多个舵机控时目标加速度
	xdata uint16_t sync_write_time_base_target_position_and_moving_time[10] = {1, 3000, 500, 2, 3000, 500};           // 同步写多个舵机控时目标运动位置和运动时间
	xdata uint16_t sync_write_velocity_base_target_position_and_velocity[10] = { 1, 1500, 1800, 2, 1500, 1800};              // 同步写多个舵机控速目标位置和速度
	
	timer0_init();

	while(1)
	{
#if FACTORY_RESET_TEST
		// 恢复出厂设置
		uart_init();
		servo_factory_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		
		uart_init_recv();
		delay_ms(10);
		servo_factory_reset_analysis(receive_data);
		delay_ms(1000);
#endif			
		
#if PARAMETER_RESET_TEST
        // 参数重置
		uart_init();
		servo_parameter_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_parameter_reset_analysis(receive_data);
		delay_ms(1000);
#endif			

#if CALIBRATION_TEST
        // 校正偏移值
		uart_init();
		servo_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_calibration_analysis(receive_data);
		delay_ms(1000);
#endif				
		
#if REBOOT_TEST
        // 舵机重启
		uart_init();
		servo_reboot(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		delay_ms(1000);
#endif		

#if PING_TEST
		// 向ID为1的舵机发送PING指令
		uart_init();
		servo_ping(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_ping_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif	

#if MODIFY_ID_TEST
        // 修改ID1舵机为ID2
		uart_init();
		servo_modify_known_id(1, 2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if MODIFY_UNKNOWN_ID_TEST
        // 将未知ID舵机ID修改为2
		uart_init();
		servo_modify_unknown_id(2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if READ_TEST
        // 读取ID1舵机的当前电流
		uart_init();
		servo_read_present_current(1, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的当前位置
		uart_init();
		servo_read_present_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的当前速度
		uart_init();
		servo_read_present_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		// 读取ID1舵机的当前的规划位置
		uart_init();
		servo_read_present_profile_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的当前规划速度
		uart_init();
		servo_read_present_profile_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的当前PWM
		uart_init();
		servo_read_present_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的当前温度
		uart_init();
		servo_read_present_temperature(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_temperature_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的当前输入电压
		uart_init();
		servo_read_present_voltage(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_voltage_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的控时目标运行时间
		uart_init();
		servo_read_time_base_target_moving_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的控时目标位置
		uart_init();
		servo_read_time_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的控时加速度等级
		uart_init();
		servo_read_time_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的控速目标减速度
		uart_init();
		servo_read_velocity_base_target_dec(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的控速目标加速度
		uart_init();
		servo_read_velocity_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的控速目标速度
		uart_init();
		servo_read_velocity_base_target_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的控速目标位置
		uart_init();
		servo_read_velocity_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的目标电流
		uart_init();
		servo_read_target_current(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的目标PWM
		uart_init();
		servo_read_target_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的扭矩开关
		uart_init();
		servo_read_torque_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_torque_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的LED开关
		uart_init();
		servo_read_led_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的Flash开关
		uart_init();
		servo_read_flash_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_flash_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的电流校正值
		uart_init();
		servo_read_current_offset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_offset_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的中位校正值
		uart_init();
		servo_read_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_calibration_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的控制模式
		uart_init();
		servo_read_control_mode(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_control_mode_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的卸载保护条件
		uart_init();
		servo_read_shutdown_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的LED报警条件
		uart_init();
		servo_read_led_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的位置控制D增益
		uart_init();
		servo_read_position_control_d_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的位置控制I增益
		uart_init();
		servo_read_position_control_i_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的位置控制P增益
		uart_init();
		servo_read_position_control_p_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的PWM叠加值
		uart_init();
		servo_read_pwm_punch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_pwm_punch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的反转死区
		uart_init();
		servo_read_ccw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的正转死区
		uart_init();
		servo_read_cw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_cw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的电流保护时间
		uart_init();
		servo_read_current_shutdown_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的电流上限
		uart_init();
		servo_read_max_current_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_current_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的PWM上限
		uart_init();
		servo_read_max_pwm_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的电压上限
		uart_init();
		servo_read_max_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的电压下限
		uart_init();
		servo_read_min_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的温度上限
		uart_init();
		servo_read_max_temperature_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的最大位置限制
		uart_init();
		servo_read_max_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的最小位置限制
		uart_init();
		servo_read_min_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的状态返回级别
		uart_init();
		servo_read_return_level(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_level_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的应答延时时间
		uart_init();
		servo_read_return_delay_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_delay_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的波特率
		uart_init();
		servo_read_baud_rate(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_baud_rate_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的出厂编号
		uart_init();
		servo_read_model_information(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_model_information_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// 读取ID1舵机的固件版本号
		uart_init();
		servo_read_firmware_version(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_firmware_version_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if WRITE_TEST
        // 设置ID1舵机的Flash开关
		uart_init();
		servo_set_flash_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_flash_switch_analysis(receive_data);
		delay_ms(1000);

		// 设置ID1舵机的应答延时时间
		uart_init();
		servo_set_return_delay_time(1, 250,order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_delay_time_analysis(receive_data);	 
		delay_ms(1000);

		// 设置ID1舵机的状态返回级别
		uart_init();
		servo_set_return_level(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_level_analysis(receive_data);	   
		delay_ms(1000);

		// 设置ID1舵机的波特率
		uart_init();
		servo_set_baud_rate(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_baud_rate_analysis(receive_data);		 
		delay_ms(1000);

		// 设置ID1舵机的最小位置限制
		uart_init();
		servo_set_min_angle_limit(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// 设置ID1舵机的最大位置限制
		uart_init();
		servo_set_max_angle_limit(1, 3000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// 设置ID1舵机的温度上限
		uart_init();
		servo_set_max_temperature_limit(1, 100, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_temperature_limit_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的电压上限
		uart_init();
		servo_set_max_voltage_limit(1,90, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的电压下限
		uart_init();
		servo_set_min_voltage_limit(1, 33, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的PWM上限
		uart_init();
		servo_set_max_pwm_limit(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_pwm_limit_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的电流上限
		uart_init();
		servo_set_max_current_limit(1, 400, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_current_limit_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的电流保护时间
		uart_init();
		servo_set_current_shutdown_time(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_current_shutdown_time_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的正转死区
		uart_init();
		servo_set_cw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_cw_deadband_analysis(receive_data);		
		delay_ms(1000);

		// 设置ID1舵机的反转死区
		uart_init();
		servo_set_ccw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_ccw_deadband_analysis(receive_data); 
		delay_ms(1000);

		// 设置ID1舵机的PWM叠加值
		uart_init();
		servo_set_pwm_punch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_pwm_punch_analysis(receive_data);		 
		delay_ms(1000);

		// 设置ID1舵机的位置控制P增益
		uart_init();
		servo_set_position_control_p_gain(1, 6000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_p_gain_analysis(receive_data); 
		delay_ms(1000);

		// 设置ID1舵机的位置控制I增益
		uart_init();
		servo_set_position_control_i_gain(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_i_gain_analysis(receive_data);  
		delay_ms(1000);

		// 设置ID1舵机的位置控制D增益
		uart_init();
		servo_set_position_control_d_gain(1, 151, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_d_gain_analysis(receive_data); 
		delay_ms(1000);

		// 设置ID1舵机的LED报警条件
		uart_init();
		servo_set_led_condition(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_condition_analysis(receive_data);		   
		delay_ms(1000);

		// 设置ID1舵机的卸载保护条件
		uart_init();
		servo_set_shutdown_conditions(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_shutdown_conditions_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的LED开关
		uart_init();
		servo_set_led_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_switch_analysis(receive_data);	 
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的控制模式
		uart_init();
		servo_set_control_mode(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的目标PWM
		uart_init();
		servo_set_target_pwm(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_pwm_analysis(receive_data);  
		delay_ms(3000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的控制模式
		uart_init();
		servo_set_control_mode(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的目标电流
		uart_init();
		servo_set_target_current(1, -1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_current_analysis(receive_data);   
		delay_ms(3000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的控制模式
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的控速目标速度
		uart_init();
		servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_velocity_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的控速目标加速度
		uart_init();
		servo_set_velocity_base_target_acc(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_acc_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的控速目标减速度
		uart_init();
		servo_set_velocity_base_target_dec(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_dec_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的控速目标位置
		uart_init();
		servo_set_velocity_base_target_position(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_velocity_base_target_position_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// 设置ID1舵机的控制模式
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		 
		delay_ms(1000);

		// 设置ID1舵机的控时目标加速度等级
		uart_init();
		servo_set_time_base_target_acc(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_acc_analysis(receive_data);	 
		delay_ms(1000);

		// 设置ID1舵机的控时目标位置和目标运行时间
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);
#endif

#if SYNC_WRITE_TEST
        // 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的控制模式
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// 设置ID2舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// 设置ID2舵机的控制模式
		uart_init();
		servo_set_control_mode(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	   
		delay_ms(1000);

		// 设置ID2舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// 设置多个舵机的控速目标加速度
		uart_init();
		servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// 设置多个舵机的控速目标减速度
		uart_init();
		servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// 设置多个舵机的控速目标速度
		uart_init();
		servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// 设置多个舵机的控速目标位置
		uart_init();
		servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		// 设置多个舵机的控速目标位置和速度
		uart_init();
		servo_sync_write_velocity_base_target_position_and_velocity(2, sync_write_velocity_base_target_position_and_velocity, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// 设置ID1舵机的控制模式
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID1舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置ID2舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// 设置ID2舵机的控制模式
		uart_init();
		servo_set_control_mode(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// 设置ID2舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	
		delay_ms(1000);

		// 设置多个舵机的控时目标加速度等级
		uart_init();
		servo_sync_write_time_base_target_acc(1,sync_write_time_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// 设置多个舵机的控时目标位置和运动时间
		uart_init();
		servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
#endif


	}		
}

void uart() interrupt 4
{
	if(RI)                                          // 检查接收中断标志
	{
		RI = 0;                                     // 清除接收中断标志
		receive_data[receive_len++] = SBUF;         // 将接收到的数据存储到缓冲区
	}				
}


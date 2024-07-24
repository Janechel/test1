#include "reg52.h"
#include "servo.c"
#include <stdlib.h>

u16 ms_count;

xdata u8 receive_data[20] = {0};
xdata u8 receive_len = 0;          // 接收应答包长度

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
void delay_ms(u16 ms)
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
void uart_send(u8 order_data)
{
    SBUF = order_data;      // 将数据写入串口缓冲寄存器开始传输
    while(!TI);    			// 等待传输完成
    TI = 0;      			// 清除传输完成标志
}


void uart_send_buffer(u8 *buffer, u16 length)
{
    u16 i;
    for (i = 0; i < length; i++) {
        uart_send(buffer[i]);
    }
}


void main()
{
	xdata u8 order_buffer[20];												                    // 存放生成的指令
	xdata u8 order_buffer_len = 0;										                        // 指令长度
	xdata u16 analysis_data = 0;											                    // 应答包解析出来的数据

	timer0_init();

	while(1)
	{
		// 设置舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置舵机的控制模式
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// 设置舵机的扭矩开关
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// 设置舵机的控时目标位置和目标运行时间
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);

		// 设置舵机的控时目标加速度等级
		uart_init();
		servo_set_time_base_target_acc(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_acc_analysis(receive_data);	 
		delay_ms(1000);

		// 设置舵机的控时目标位置和目标运行时间
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 0, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);
		
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


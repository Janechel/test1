#include "reg52.h"
#include "servo.c"

uint16_t ms_count;

xdata uint8_t receive_data[20] = {0};
xdata uint8_t receive_len = 0;          // 接收应答包长度
xdata uint8_t write_buffer[10] = {0};     //写入内存表数据

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

struct servo_sync_parameter servo;

void main()
{
	xdata uint8_t order_buffer[20];												                    // 存放生成的指令
	xdata uint8_t order_buffer_len = 0;										                        // 指令长度
	xdata uint16_t analysis_data = 0;											                    // 应答包解析出来的数据

	timer0_init();

	while(1)
	{
    servo.id_counts = 2;            //同步写两个舵机
    servo.id[0] = 1;                //第一个舵机id为1
    servo.id[1] = 2;                //第二个舵机id为2
		
		// 将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    servo_sync_write_torque_switch(servo, order_buffer, &order_buffer_len);
		
		uart_init();
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		// 将ID1、ID2舵机的控制模式，分别修改为控速模式
		servo.control_mode[0] = 1;
		servo.control_mode[1] = 1;
		servo_sync_write_control_mode(servo, order_buffer, &order_buffer_len);

		uart_init();
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		
		// 设置舵机的控速目标位置
		uart_init();
		servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_velocity_base_target_position_analysis(receive_data);	  
		delay_ms(1000);
		
		// 在控速模式下，让ID1以360°/s的控速目标速度运动到300°位置
		write_buffer[0] = 3000 & 0xff;
		write_buffer[1] = (3000 >> 8) & 0xff;
		write_buffer[2] = 3600 & 0xff;
		write_buffer[3] = (3600 >> 8) & 0xff;

		servo_write(1, 0x35, 4, write_buffer, order_buffer, &order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);
		uart_init_recv();
		delay_ms(1000);
		
		
		//在控速模式下，让ID1舵机以360°/s的控速目标速度、500°/s2的控速目标加速度、50°/s2的控速目标减速度运动到0°位置
		write_buffer[0] = 0 & 0xff;
		write_buffer[1] = (0 >> 8) & 0xff;
		write_buffer[2] = 3600 & 0xff;
		write_buffer[3] = (3600 >> 8) & 0xff;
		write_buffer[4] = 10 & 0xff;
		write_buffer[5] = 1 & 0xff;

		servo_write(1, 0x35, 6, write_buffer, order_buffer, &order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);
		uart_init_recv();
		delay_ms(1000);
		
			
		//设置多个舵机的控速目标位置

		//id为1，2的舵机运动位置分别设置为0，0，值和前面的id设置对应
		servo.position[0] = 0;
		servo.position[1] = 0;

		servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_buffer_len);
		delay_ms(1000);
		
		 //在控速模式下，让ID1以360°/s的控速目标速度运动到300°位置，让ID2以720°/s的控速目标速度运动到150°位置

	 //id为1，2的舵机速度分别设置为3600，7200，位置分别设置为3000，1500
		servo.velocity[0] = 3600;
		servo.velocity[1] = 7200;
		servo.position[0] = 3000;
		servo.position[1] = 1500;

		servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_buffer_len);
		delay_ms(1000);
		
		
		//在控速模式下，让ID1舵机以720°/s的控速目标速度、500°/s2的控速目标加速度、50°/s2的控速目标减速度运动到0°位置，让ID2舵机以360°/s的控速目标速度、50°/s2的控速目标加速度、500°/s2的控速目标减速度运动到300°位置

		//id为1，2的舵机速度分别设置为3600，3600，位置分别设置为0，0,加速度分别设置为100，100，减速度分别设置为100，100
		servo.velocity[0] = 3600;
		servo.velocity[1] = 3600;
		servo.position[0] = 0;
		servo.position[1] = 0;
		servo.acc_velocity[0] = 100;
		servo.acc_velocity[1] = 100;
		servo.dec_velocity[0] = 100;
		servo.dec_velocity[1] = 100;

		servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_buffer_len);
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


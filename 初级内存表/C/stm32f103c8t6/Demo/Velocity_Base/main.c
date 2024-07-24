#include "stm32f10x.h"
#include "platform_config.h"
#include "servo.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

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

	//设置舵机的扭矩开关
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

	//设置舵机的控制模式
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

	//设置舵机的扭矩开关
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
	
	while(1)
	{
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
		
		//设置舵机的控速目标位置
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

		
		//设置舵机的控速目标速度
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

		//设置舵机的控速目标加速度
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

		//设置舵机的控速目标减速度
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

		//设置舵机的控速目标位置
    servo_set_velocity_base_target_position(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_velocity_base_target_position_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set velocity base target position success");
		Delay(1000);
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

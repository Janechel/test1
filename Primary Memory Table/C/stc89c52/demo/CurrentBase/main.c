#include "reg52.h"
#include "servo.c"

uint16_t ms_count;

xdata uint8_t receive_data[20] = {0};
xdata uint8_t receive_len = 0;          // ����Ӧ�������

// time0��ʼ������ʱ
void timer0_init()
{
    TMOD |= 0x01;
    TH0 = 0xFC;
    TL0 = 0x67;
    ET0 = 1;
    EA = 1;
    TR0 = 1;
}

// ��ʱ����
void delay_ms(uint16_t ms)
{
    ms_count =  2 * ms;   // ��Ϊ����˫����6T������������ʱʱ����Ҫ����2
    while (ms_count);
}

// ��ʱ���жϺ���
void timer0_isr() interrupt 1 using 1
{
    TH0 = 0xFC;
    TL0 = 0x67;
    if (ms_count)
        ms_count--;
}

// ���ڳ�ʼ��
void uart_init()
{
	TMOD|=0X20;	   // 8λ�Զ���װ�ض�ʱ��
	SCON=0X40;	   // 8λUART�������ʿɱ�
	PCON=0X80;	   // �����ʼӱ�
	TH1=0xff;	   // ���ò�����Ϊ115200
	TL1=0xff;
	ES=0;		   // �رս����ж�
	EA=1;		   // CPU���ж�
	TR1=1;		   // ������ʱ��T1��ʼ����
}

void uart_init_recv()
{
	TMOD |= 0x20;  // 8λ�Զ���װ�ض�ʱ��
	SCON = 0x50;   // 8λUART�������ʿɱ䣬���������н���
	PCON = 0x80;   // �����ʼӱ�
	TH1 = 0xff;    // ���ò�����Ϊ115200
	TL1 = 0xff;
	ES = 1;        // ���������ж�
	EA = 1;        // CPU���ж�
	TR1 = 1;       // ������ʱ��T1��ʼ����
}

// ���ڷ��ͺ���
void uart_send(uint8_t order_data)
{
	SBUF = order_data;      // ������д�봮�ڻ���Ĵ�����ʼ����
	while(!TI);    			// �ȴ��������
	TI = 0;      			// ���������ɱ�־
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
	xdata uint8_t order_buffer[20];												                    // ������ɵ�ָ��
	xdata uint8_t order_buffer_len = 0;										                        // ָ���
	xdata uint16_t analysis_data = 0;											                    // Ӧ�����������������

	timer0_init();

	while(1)
	{
		// ���ö����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ���ö���Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ���ö����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ?????????
		uart_init();
		servo_set_target_current(1, 100, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_current_analysis(receive_data);   
		delay_ms(3000);
	}		
}

void uart() interrupt 4
{
	if(RI)                                          // �������жϱ�־
	{
		RI = 0;                                     // ��������жϱ�־
		receive_data[receive_len++] = SBUF;         // �����յ������ݴ洢��������
	}				
}


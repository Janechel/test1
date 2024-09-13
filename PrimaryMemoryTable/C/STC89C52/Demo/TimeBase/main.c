#include "reg52.h"
#include "servo.c"

uint16_t ms_count;

xdata uint8_t receive_data[20] = {0};
xdata uint8_t receive_len = 0;          // ����Ӧ�������
xdata uint8_t write_buffer[10] = {0};     //д���ڴ������

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

struct servo_sync_parameter servo;

void main()
{
	xdata uint8_t order_buffer[20];												                    // ������ɵ�ָ��
	xdata uint8_t order_buffer_len = 0;										                        // ָ���
	xdata uint16_t analysis_data = 0;											                    // Ӧ�����������������

	timer0_init();

	while(1)
	{
    servo.id_counts = 2;            //ͬ��д�������
    servo.id[0] = 1;                //��һ�����idΪ1
    servo.id[1] = 2;                //�ڶ������idΪ2
		
		// ��ID1��ID2�����Ť�ؿ���״̬���ֱ��޸�Ϊ�ر�
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    servo_sync_write_torque_switch(servo, order_buffer, &order_buffer_len);
		
		uart_init();
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		// ��ID1��ID2����Ŀ���ģʽ���ֱ��޸�Ϊ��ʱ��ģʽ
		servo.control_mode[0] = 0;
		servo.control_mode[1] = 0;
		servo_sync_write_control_mode(servo, order_buffer, &order_buffer_len);

		uart_init();
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		
		// �ڿ�ʱģʽ�£���ID1�����500ms�˶���300��λ��
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);
	
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);

		// �ڿ�ʱģʽ�£���ID1�����1s�����˶���0��λ��
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 0, 1000, order_buffer,&order_buffer_len);
	
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);
		
		// �ڿ�ʱģʽ�£���ID1�����500ms�˶���150��λ�ã���ID2�����1s�����˶���0��λ��
		servo.position[0] = 1500;
		servo.position[1] = 0;
		servo.time[0] = 500;
		servo.time[1] = 1000;
		
		uart_init();
		servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
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


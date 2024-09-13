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
		
		// ��ID1��ID2����Ŀ���ģʽ���ֱ��޸�Ϊ����ģʽ
		servo.control_mode[0] = 1;
		servo.control_mode[1] = 1;
		servo_sync_write_control_mode(servo, order_buffer, &order_buffer_len);

		uart_init();
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		
		// ���ö���Ŀ���Ŀ��λ��
		uart_init();
		servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_velocity_base_target_position_analysis(receive_data);	  
		delay_ms(1000);
		
		// �ڿ���ģʽ�£���ID1��360��/s�Ŀ���Ŀ���ٶ��˶���300��λ��
		write_buffer[0] = 3000 & 0xff;
		write_buffer[1] = (3000 >> 8) & 0xff;
		write_buffer[2] = 3600 & 0xff;
		write_buffer[3] = (3600 >> 8) & 0xff;

		servo_write(1, 0x35, 4, write_buffer, order_buffer, &order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);
		uart_init_recv();
		delay_ms(1000);
		
		
		//�ڿ���ģʽ�£���ID1�����360��/s�Ŀ���Ŀ���ٶȡ�500��/s2�Ŀ���Ŀ����ٶȡ�50��/s2�Ŀ���Ŀ����ٶ��˶���0��λ��
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
		
			
		//���ö������Ŀ���Ŀ��λ��

		//idΪ1��2�Ķ���˶�λ�÷ֱ�����Ϊ0��0��ֵ��ǰ���id���ö�Ӧ
		servo.position[0] = 0;
		servo.position[1] = 0;

		servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_buffer_len);
		delay_ms(1000);
		
		 //�ڿ���ģʽ�£���ID1��360��/s�Ŀ���Ŀ���ٶ��˶���300��λ�ã���ID2��720��/s�Ŀ���Ŀ���ٶ��˶���150��λ��

	 //idΪ1��2�Ķ���ٶȷֱ�����Ϊ3600��7200��λ�÷ֱ�����Ϊ3000��1500
		servo.velocity[0] = 3600;
		servo.velocity[1] = 7200;
		servo.position[0] = 3000;
		servo.position[1] = 1500;

		servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_buffer_len);
		delay_ms(1000);
		
		
		//�ڿ���ģʽ�£���ID1�����720��/s�Ŀ���Ŀ���ٶȡ�500��/s2�Ŀ���Ŀ����ٶȡ�50��/s2�Ŀ���Ŀ����ٶ��˶���0��λ�ã���ID2�����360��/s�Ŀ���Ŀ���ٶȡ�50��/s2�Ŀ���Ŀ����ٶȡ�500��/s2�Ŀ���Ŀ����ٶ��˶���300��λ��

		//idΪ1��2�Ķ���ٶȷֱ�����Ϊ3600��3600��λ�÷ֱ�����Ϊ0��0,���ٶȷֱ�����Ϊ100��100�����ٶȷֱ�����Ϊ100��100
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
	if(RI)                                          // �������жϱ�־
	{
		RI = 0;                                     // ��������жϱ�־
		receive_data[receive_len++] = SBUF;         // �����յ������ݴ洢��������
	}				
}


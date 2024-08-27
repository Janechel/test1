#include "reg52.h"
#include "servo.c"

#define READ_TEST 0                 // ��ȡ������ݲ���
#define WRITE_TEST 0                // д�������ݲ���
#define SYNC_WRITE_TEST 0           // ͬ��д����
#define PING_TEST 0                 // PING�������
#define FACTORY_RESET_TEST 0        // �ָ��������ò���
#define PARAMETER_RESET_TEST 0      // �������ò���
#define REBOOT_TEST 0               // ��������
#define CALIBRATION_TEST 0          // У��ƫ��ֵ����
#define MODIFY_ID_TEST 0            // �޸Ķ��ID����
#define MODIFY_UNKNOWN_ID_TEST 0    // �޸�δ֪ID���ID����

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
	xdata uint16_t sync_write_velocity_base_target_position[4] = {1, 0, 2, 0};                       // ͬ��д����������Ŀ��λ��
	xdata uint16_t sync_write_velocity_base_target_velocity[5] = {1, 3600, 2, 3600};                 // ͬ��д����������Ŀ���ٶ�
	xdata uint16_t sync_write_velocity_base_target_acc[5] = {1, 150, 2, 150};                        // ͬ��д����������Ŀ����ٶ�
	xdata uint16_t sync_write_velocity_base_target_dec[5] = {1, 150, 2, 150};                        // ͬ��д����������Ŀ����ٶ�
	xdata uint16_t sync_write_time_base_target_acc[5] = {1, 0, 2, 0};                                // ͬ��д��������ʱĿ����ٶ�
	xdata uint16_t sync_write_time_base_target_position_and_moving_time[10] = {1, 3000, 500, 2, 3000, 500};           // ͬ��д��������ʱĿ���˶�λ�ú��˶�ʱ��
	xdata uint16_t sync_write_velocity_base_target_position_and_velocity[10] = { 1, 1500, 1800, 2, 1500, 1800};              // ͬ��д����������Ŀ��λ�ú��ٶ�
	
	timer0_init();

	while(1)
	{
#if FACTORY_RESET_TEST
		// �ָ���������
		uart_init();
		servo_factory_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		
		uart_init_recv();
		delay_ms(10);
		servo_factory_reset_analysis(receive_data);
		delay_ms(1000);
#endif			
		
#if PARAMETER_RESET_TEST
        // ��������
		uart_init();
		servo_parameter_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_parameter_reset_analysis(receive_data);
		delay_ms(1000);
#endif			

#if CALIBRATION_TEST
        // У��ƫ��ֵ
		uart_init();
		servo_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_calibration_analysis(receive_data);
		delay_ms(1000);
#endif				
		
#if REBOOT_TEST
        // �������
		uart_init();
		servo_reboot(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		delay_ms(1000);
#endif		

#if PING_TEST
		// ��IDΪ1�Ķ������PINGָ��
		uart_init();
		servo_ping(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_ping_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif	

#if MODIFY_ID_TEST
        // �޸�ID1���ΪID2
		uart_init();
		servo_modify_known_id(1, 2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if MODIFY_UNKNOWN_ID_TEST
        // ��δ֪ID���ID�޸�Ϊ2
		uart_init();
		servo_modify_unknown_id(2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if READ_TEST
        // ��ȡID1����ĵ�ǰ����
		uart_init();
		servo_read_present_current(1, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�ǰλ��
		uart_init();
		servo_read_present_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�ǰ�ٶ�
		uart_init();
		servo_read_present_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		// ��ȡID1����ĵ�ǰ�Ĺ滮λ��
		uart_init();
		servo_read_present_profile_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�ǰ�滮�ٶ�
		uart_init();
		servo_read_present_profile_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�ǰPWM
		uart_init();
		servo_read_present_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�ǰ�¶�
		uart_init();
		servo_read_present_temperature(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_temperature_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�ǰ�����ѹ
		uart_init();
		servo_read_present_voltage(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_voltage_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ŀ�ʱĿ������ʱ��
		uart_init();
		servo_read_time_base_target_moving_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ŀ�ʱĿ��λ��
		uart_init();
		servo_read_time_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ŀ�ʱ���ٶȵȼ�
		uart_init();
		servo_read_time_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ŀ���Ŀ����ٶ�
		uart_init();
		servo_read_velocity_base_target_dec(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ŀ���Ŀ����ٶ�
		uart_init();
		servo_read_velocity_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ŀ���Ŀ���ٶ�
		uart_init();
		servo_read_velocity_base_target_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ŀ���Ŀ��λ��
		uart_init();
		servo_read_velocity_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����Ŀ�����
		uart_init();
		servo_read_target_current(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����Ŀ��PWM
		uart_init();
		servo_read_target_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����Ť�ؿ���
		uart_init();
		servo_read_torque_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_torque_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����LED����
		uart_init();
		servo_read_led_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����Flash����
		uart_init();
		servo_read_flash_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_flash_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ���У��ֵ
		uart_init();
		servo_read_current_offset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_offset_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�������λУ��ֵ
		uart_init();
		servo_read_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_calibration_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ŀ���ģʽ
		uart_init();
		servo_read_control_mode(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_control_mode_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����ж�ر�������
		uart_init();
		servo_read_shutdown_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����LED��������
		uart_init();
		servo_read_led_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����λ�ÿ���D����
		uart_init();
		servo_read_position_control_d_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����λ�ÿ���I����
		uart_init();
		servo_read_position_control_i_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����λ�ÿ���P����
		uart_init();
		servo_read_position_control_p_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����PWM����ֵ
		uart_init();
		servo_read_pwm_punch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_pwm_punch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ķ�ת����
		uart_init();
		servo_read_ccw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�������ת����
		uart_init();
		servo_read_cw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_cw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�������ʱ��
		uart_init();
		servo_read_current_shutdown_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�������
		uart_init();
		servo_read_max_current_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_current_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����PWM����
		uart_init();
		servo_read_max_pwm_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�ѹ����
		uart_init();
		servo_read_max_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĵ�ѹ����
		uart_init();
		servo_read_min_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1������¶�����
		uart_init();
		servo_read_max_temperature_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1��������λ������
		uart_init();
		servo_read_max_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�������Сλ������
		uart_init();
		servo_read_min_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����״̬���ؼ���
		uart_init();
		servo_read_return_level(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_level_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1�����Ӧ����ʱʱ��
		uart_init();
		servo_read_return_delay_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_delay_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ĳ�����
		uart_init();
		servo_read_baud_rate(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_baud_rate_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����ĳ������
		uart_init();
		servo_read_model_information(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_model_information_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ��ȡID1����Ĺ̼��汾��
		uart_init();
		servo_read_firmware_version(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_firmware_version_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if WRITE_TEST
        // ����ID1�����Flash����
		uart_init();
		servo_set_flash_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_flash_switch_analysis(receive_data);
		delay_ms(1000);

		// ����ID1�����Ӧ����ʱʱ��
		uart_init();
		servo_set_return_delay_time(1, 250,order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_delay_time_analysis(receive_data);	 
		delay_ms(1000);

		// ����ID1�����״̬���ؼ���
		uart_init();
		servo_set_return_level(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_level_analysis(receive_data);	   
		delay_ms(1000);

		// ����ID1����Ĳ�����
		uart_init();
		servo_set_baud_rate(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_baud_rate_analysis(receive_data);		 
		delay_ms(1000);

		// ����ID1�������Сλ������
		uart_init();
		servo_set_min_angle_limit(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// ����ID1��������λ������
		uart_init();
		servo_set_max_angle_limit(1, 3000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// ����ID1������¶�����
		uart_init();
		servo_set_max_temperature_limit(1, 100, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_temperature_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����ĵ�ѹ����
		uart_init();
		servo_set_max_voltage_limit(1,90, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����ĵ�ѹ����
		uart_init();
		servo_set_min_voltage_limit(1, 33, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1�����PWM����
		uart_init();
		servo_set_max_pwm_limit(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_pwm_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����ĵ�������
		uart_init();
		servo_set_max_current_limit(1, 400, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_current_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����ĵ�������ʱ��
		uart_init();
		servo_set_current_shutdown_time(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_current_shutdown_time_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1�������ת����
		uart_init();
		servo_set_cw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_cw_deadband_analysis(receive_data);		
		delay_ms(1000);

		// ����ID1����ķ�ת����
		uart_init();
		servo_set_ccw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_ccw_deadband_analysis(receive_data); 
		delay_ms(1000);

		// ����ID1�����PWM����ֵ
		uart_init();
		servo_set_pwm_punch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_pwm_punch_analysis(receive_data);		 
		delay_ms(1000);

		// ����ID1�����λ�ÿ���P����
		uart_init();
		servo_set_position_control_p_gain(1, 6000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_p_gain_analysis(receive_data); 
		delay_ms(1000);

		// ����ID1�����λ�ÿ���I����
		uart_init();
		servo_set_position_control_i_gain(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_i_gain_analysis(receive_data);  
		delay_ms(1000);

		// ����ID1�����λ�ÿ���D����
		uart_init();
		servo_set_position_control_d_gain(1, 151, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_d_gain_analysis(receive_data); 
		delay_ms(1000);

		// ����ID1�����LED��������
		uart_init();
		servo_set_led_condition(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_condition_analysis(receive_data);		   
		delay_ms(1000);

		// ����ID1�����ж�ر�������
		uart_init();
		servo_set_shutdown_conditions(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_shutdown_conditions_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1�����LED����
		uart_init();
		servo_set_led_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_switch_analysis(receive_data);	 
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1�����Ŀ��PWM
		uart_init();
		servo_set_target_pwm(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_pwm_analysis(receive_data);  
		delay_ms(3000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1�����Ŀ�����
		uart_init();
		servo_set_target_current(1, -1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_current_analysis(receive_data);   
		delay_ms(3000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����Ŀ���Ŀ���ٶ�
		uart_init();
		servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_velocity_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1����Ŀ���Ŀ����ٶ�
		uart_init();
		servo_set_velocity_base_target_acc(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_acc_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1����Ŀ���Ŀ����ٶ�
		uart_init();
		servo_set_velocity_base_target_dec(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_dec_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1����Ŀ���Ŀ��λ��
		uart_init();
		servo_set_velocity_base_target_position(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_velocity_base_target_position_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ����ID1����Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		 
		delay_ms(1000);

		// ����ID1����Ŀ�ʱĿ����ٶȵȼ�
		uart_init();
		servo_set_time_base_target_acc(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_acc_analysis(receive_data);	 
		delay_ms(1000);

		// ����ID1����Ŀ�ʱĿ��λ�ú�Ŀ������ʱ��
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);
#endif

#if SYNC_WRITE_TEST
        // ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1����Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ����ID2�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ����ID2����Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	   
		delay_ms(1000);

		// ����ID2�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ���ö������Ŀ���Ŀ����ٶ�
		uart_init();
		servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ���ö������Ŀ���Ŀ����ٶ�
		uart_init();
		servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ���ö������Ŀ���Ŀ���ٶ�
		uart_init();
		servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ���ö������Ŀ���Ŀ��λ��
		uart_init();
		servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
		
		// ���ö������Ŀ���Ŀ��λ�ú��ٶ�
		uart_init();
		servo_sync_write_velocity_base_target_position_and_velocity(2, sync_write_velocity_base_target_position_and_velocity, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ����ID1����Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID1�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����ID2�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ����ID2����Ŀ���ģʽ
		uart_init();
		servo_set_control_mode(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����ID2�����Ť�ؿ���
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	
		delay_ms(1000);

		// ���ö������Ŀ�ʱĿ����ٶȵȼ�
		uart_init();
		servo_sync_write_time_base_target_acc(1,sync_write_time_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ���ö������Ŀ�ʱĿ��λ�ú��˶�ʱ��
		uart_init();
		servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
#endif


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


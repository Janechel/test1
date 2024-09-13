#include "reg52.h"
#include "servo.c"

#define READ_TEST 0                 // ?����????����y?Y2a��?
#define WRITE_TEST 1                // D�䨨????����y?Y2a��?
#define SYNC_WRITE_TEST 0           // ��?2?D��2a��?
#define PING_TEST 0                 // PING?����?2a��?
#define FACTORY_RESET_TEST 0        // ???��3?3�쨦��??2a��?
#define PARAMETER_RESET_TEST 0      // 2?��y????2a��?
#define REBOOT_TEST 0               // ????2a��?
#define CALIBRATION_TEST 0          // D��?y??��??��2a��?
#define MODIFY_ID_TEST 0            // DT?????��ID2a��?
#define MODIFY_UNKNOWN_ID_TEST 0    // DT???��?aID???��ID2a��?

uint16_t ms_count;

xdata uint8_t receive_data[20] = {0};
xdata uint8_t receive_len = 0;          // ?����?��|��e�㨹3��?��

// time03?��??����??������
void timer0_init()
{
    TMOD |= 0x01;
    TH0 = 0xFC;
    TL0 = 0x67;
    ET0 = 1;
    EA = 1;
    TR0 = 1;
}

// ?������o����y
void delay_ms(uint16_t ms)
{
    ms_count =  2 * ms;   // ����?a?a��???��??��6T��??����??a��??����������??D����a3?��?2
    while (ms_count);
}

// ?������?��?D??o����y
void timer0_isr() interrupt 1 using 1
{
    TH0 = 0xFC;
    TL0 = 0x67;
    if (ms_count)
        ms_count--;
}

// ��??��3?��??��
void uart_init()
{
	TMOD|=0X20;	   // 8??��??��??����???������?��
	SCON=0X40;	   // 8??UART��?2����??��?����?
	PCON=0X80;	   // 2����??��?����?
	TH1=0xff;	   // ����??2����??��?a115200
	TL1=0xff;
    ES=0;		   // 1?��??����??D??
    EA=1;		   // CPU����?D??
    TR1=1;		   // ?a???������?��T1?a��???��y
}

void uart_init_recv()
{
    TMOD |= 0x20;  // 8??��??��??����???������?��
    SCON = 0x50;   // 8??UART��?2����??��?����?��?2��?a??��?DD?����?
    PCON = 0x80;   // 2����??��?����?
    TH1 = 0xff;    // ����??2����??��?a115200
    TL1 = 0xff;
    ES = 1;        // ?a???����??D??
    EA = 1;        // CPU����?D??
    TR1 = 1;       // ?a???������?��T1?a��???��y
}

// ��??������?��o����y
void uart_send(uint8_t order_data)
{
    SBUF = order_data;      // ??��y?YD�䨨?��??��?o3???��??��?a��?��?��?
    while(!TI);    			// �̨���y��?��?����3��
    TI = 0;      			// ??3y��?��?����3������??
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
	xdata uint8_t order_buffer[20];												                    // ��?��?����3����???��?
	xdata uint8_t order_buffer_len = 0;										                        // ??��?3��?��
	xdata uint16_t analysis_data = 0;											                    // ��|��e�㨹?a??3?�����?��y?Y
	xdata uint16_t sync_write_velocity_base_target_position[4] = {1, 0, 2, 0};                       // ��?2?D��?��?????��???��??����????
	xdata uint16_t sync_write_velocity_base_target_velocity[5] = {1, 3600, 2, 3600};                 // ��?2?D��?��?????��???��??����?��?��
	xdata uint16_t sync_write_velocity_base_target_acc[5] = {1, 150, 2, 150};                        // ��?2?D��?��?????��???��??����?��?��?��
	xdata uint16_t sync_write_velocity_base_target_dec[5] = {1, 150, 2, 150};                        // ��?2?D��?��?????��???��??����???��?��
	xdata uint16_t sync_write_time_base_target_acc[5] = {1, 0, 2, 0};                                // ��?2?D��?��?????��??����??����?��?��?��
	xdata uint16_t sync_write_time_base_target_position_and_moving_time[10] = {1, 3000, 500, 2, 3000, 500};           // ��?2?D��?��?????��??����??����???��????o��???������??

	timer0_init();

	while(1)
	{
#if FACTORY_RESET_TEST
		// ???��3?3�쨦��??
		uart_init();
		servo_factory_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		
		uart_init_recv();
		delay_ms(10);
		servo_factory_reset_analysis(receive_data);
		delay_ms(1000);
#endif			
		
#if PARAMETER_RESET_TEST
        // 2?��y????
		uart_init();
		servo_parameter_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_parameter_reset_analysis(receive_data);
		delay_ms(1000);
#endif			

#if CALIBRATION_TEST
        // D��?y??��??��
		uart_init();
		servo_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_calibration_analysis(receive_data);
		delay_ms(1000);
#endif				
		
#if REBOOT_TEST
        // ???��????
		uart_init();
		servo_reboot(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		delay_ms(1000);
#endif		

#if PING_TEST
		// ?��ID?a1��????������?��PING??��?
		uart_init();
		servo_ping(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_ping_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif	

#if MODIFY_ID_TEST
        // DT?????��ID2a��?
		uart_init();
		servo_modify_known_id(1, 2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if MODIFY_UNKNOWN_ID_TEST
        // DT???��?aID???��ID2a��?
		uart_init();
		servo_modify_unknown_id(2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if READ_TEST
        // ?����????����?�̡�?���?����
		uart_init();
		servo_read_present_current(1, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?�̡�?��????
		uart_init();
		servo_read_present_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?�̡�?��?��?��
		uart_init();
		servo_read_present_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		// ?����????����?�̡�?���?1???????
		uart_init();
		servo_read_present_profile_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?�̡�?��1????��?��
		uart_init();
		servo_read_present_profile_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?�̡�?��PWM
		uart_init();
		servo_read_present_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?�̡�?��???��
		uart_init();
		servo_read_present_temperature(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_temperature_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?�̡�?�㨺?��?��??1
		uart_init();
		servo_read_present_voltage(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_voltage_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����???����??����??DD����??
		uart_init();
		servo_read_time_base_target_moving_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����???����??����????
		uart_init();
		servo_read_time_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����???����?��?��?���̨�??
		uart_init();
		servo_read_time_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����????��??����???��?��
		uart_init();
		servo_read_velocity_base_target_dec(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����????��??����?��?��?��
		uart_init();
		servo_read_velocity_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����????��??����?��?��
		uart_init();
		servo_read_velocity_base_target_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����????��??����????
		uart_init();
		servo_read_velocity_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����???������?����
		uart_init();
		servo_read_target_current(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����???����PWM
		uart_init();
		servo_read_target_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����??��???a1?
		uart_init();
		servo_read_torque_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_torque_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?LED?a1?
		uart_init();
		servo_read_led_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?Flash?a1?
		uart_init();
		servo_read_flash_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_flash_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?��?����D��?y?��
		uart_init();
		servo_read_current_offset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_offset_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����??D??D��?y?��
		uart_init();
		servo_read_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_calibration_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����??????�꨺?
		uart_init();
		servo_read_control_mode(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_control_mode_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?D???����?�訬??t
		uart_init();
		servo_read_shutdown_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?LED����?����??t
		uart_init();
		servo_read_led_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?????????D??��?
		uart_init();
		servo_read_position_control_d_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?????????I??��?
		uart_init();
		servo_read_position_control_i_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?????????P??��?
		uart_init();
		servo_read_position_control_p_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?PWM��t?��?��
		uart_init();
		servo_read_pwm_punch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_pwm_punch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?�����a?��??
		uart_init();
		servo_read_ccw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����??y��a?��??
		uart_init();
		servo_read_cw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_cw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?��?���¡���?�診��??
		uart_init();
		servo_read_current_shutdown_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?��?���¨�??T
		uart_init();
		servo_read_max_current_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_current_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?PWM��??T
		uart_init();
		servo_read_max_pwm_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?��??1��??T
		uart_init();
		servo_read_max_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?��??1???T
		uart_init();
		servo_read_min_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����????����??T
		uart_init();
		servo_read_max_temperature_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?��?�䨮?????T??
		uart_init();
		servo_read_max_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?��?D??????T??
		uart_init();
		servo_read_min_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?���䨬?����????��e
		uart_init();
		servo_read_return_level(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_level_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?��|��e?����������??
		uart_init();
		servo_read_return_delay_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_delay_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?2����??��
		uart_init();
		servo_read_baud_rate(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_baud_rate_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?3?3�����o?
		uart_init();
		servo_read_model_information(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_model_information_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?����????����?1��?t��?��?o?
		uart_init();
		servo_read_firmware_version(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_firmware_version_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if WRITE_TEST
        // ����?????����?Flash?a1?
		uart_init();
		servo_set_flash_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_flash_switch_analysis(receive_data);
		delay_ms(1000);

		// ����?????����?��|��e?����������??
		uart_init();
		servo_set_return_delay_time(1, 250,order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_delay_time_analysis(receive_data);	 
		delay_ms(1000);

		// ����?????����?���䨬?����????��e
		uart_init();
		servo_set_return_level(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_level_analysis(receive_data);	   
		delay_ms(1000);

		// ����?????����?2����??��
		uart_init();
		servo_set_baud_rate(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_baud_rate_analysis(receive_data);		 
		delay_ms(1000);

		// ����?????����?��?D??????T??
		uart_init();
		servo_set_min_angle_limit(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// ����?????����?��?�䨮?????T??
		uart_init();
		servo_set_max_angle_limit(1, 3000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// ����?????����????����??T
		uart_init();
		servo_set_max_temperature_limit(1, 100, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_temperature_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����?��??1��??T
		uart_init();
		servo_set_max_voltage_limit(1,90, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����?��??1???T
		uart_init();
		servo_set_min_voltage_limit(1, 33, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����?PWM��??T
		uart_init();
		servo_set_max_pwm_limit(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_pwm_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����?��?���¨�??T
		uart_init();
		servo_set_max_current_limit(1, 400, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_current_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����?��?���¡���?�診��??
		uart_init();
		servo_set_current_shutdown_time(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_current_shutdown_time_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����??y��a?��??
		uart_init();
		servo_set_cw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_cw_deadband_analysis(receive_data);		
		delay_ms(1000);

		// ����?????����?�����a?��??
		uart_init();
		servo_set_ccw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_ccw_deadband_analysis(receive_data); 
		delay_ms(1000);

		// ����?????����?PWM��t?��?��
		uart_init();
		servo_set_pwm_punch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_pwm_punch_analysis(receive_data);		 
		delay_ms(1000);

		// ����?????����?????????P??��?
		uart_init();
		servo_set_position_control_p_gain(1, 6000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_p_gain_analysis(receive_data); 
		delay_ms(1000);

		// ����?????����?????????I??��?
		uart_init();
		servo_set_position_control_i_gain(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_i_gain_analysis(receive_data);  
		delay_ms(1000);

		// ����?????����?????????D??��?
		uart_init();
		servo_set_position_control_d_gain(1, 151, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_d_gain_analysis(receive_data); 
		delay_ms(1000);

		// ����?????����?LED����?����??t
		uart_init();
		servo_set_led_condition(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_condition_analysis(receive_data);		   
		delay_ms(1000);

		// ����?????����?D???����?�訬??t
		uart_init();
		servo_set_shutdown_conditions(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_shutdown_conditions_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����?LED?a1?
		uart_init();
		servo_set_led_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_switch_analysis(receive_data);	 
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����??????�꨺?
		uart_init();
		servo_set_control_mode(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����???����PWM
		uart_init();
		servo_set_target_pwm(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_pwm_analysis(receive_data);  
		delay_ms(3000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����??????�꨺?
		uart_init();
		servo_set_control_mode(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����???������?����
		uart_init();
		servo_set_target_current(1, -1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_current_analysis(receive_data);   
		delay_ms(3000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����??????�꨺?
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����????��??����?��?��
		uart_init();
		servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_velocity_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����????��??����?��?��?��
		uart_init();
		servo_set_velocity_base_target_acc(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_acc_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����????��??����???��?��
		uart_init();
		servo_set_velocity_base_target_dec(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_dec_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����????��??����????
		uart_init();
		servo_set_velocity_base_target_position(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_velocity_base_target_position_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ����?????����??????�꨺?
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		 
		delay_ms(1000);

		// ����?????����???����??����?��?��?���̨�??
		uart_init();
		servo_set_time_base_target_acc(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_acc_analysis(receive_data);	 
		delay_ms(1000);

		// ����?????����???����??����????o��??����??DD����??
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);
#endif

#if SYNC_WRITE_TEST
        // ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����??????�꨺?
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ����?????����??????�꨺?
		uart_init();
		servo_set_control_mode(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	   
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ����???��?????����????��??����?��?��?��
		uart_init();
		servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ����???��?????����????��??����???��?��
		uart_init();
		servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ����???��?????����????��??����?��?��
		uart_init();
		servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ����???��?????����????��??����????
		uart_init();
		servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ����?????����??????�꨺?
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ����?????����??????�꨺?
		uart_init();
		servo_set_control_mode(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ����?????����??��???a1?
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	
		delay_ms(1000);

		// ����???��?????����???����??����?��?��?���̨�??
		uart_init();
		servo_sync_write_time_base_target_acc(1,sync_write_time_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ����???��?????����???����??����????o��???������??
		uart_init();
		servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
#endif


	}		
}

void uart() interrupt 4
{
	if(RI)                                          // ?��2��?����??D??����??
	{
		RI = 0;                                     // ??3y?����??D??����??
		receive_data[receive_len++] = SBUF;         // ???����?��?��?��y?Y��?����??o3???
	}				
}


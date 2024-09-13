#include "reg52.h"
#include "servo.c"

#define READ_TEST 0                 // ?¨¢¨¨????¨²¨ºy?Y2a¨º?
#define WRITE_TEST 1                // D¡ä¨¨????¨²¨ºy?Y2a¨º?
#define SYNC_WRITE_TEST 0           // ¨ª?2?D¡ä2a¨º?
#define PING_TEST 0                 // PING?¨¹¨¢?2a¨º?
#define FACTORY_RESET_TEST 0        // ???¡ä3?3¡ì¨¦¨¨??2a¨º?
#define PARAMETER_RESET_TEST 0      // 2?¨ºy????2a¨º?
#define REBOOT_TEST 0               // ????2a¨º?
#define CALIBRATION_TEST 0          // D¡ê?y??¨°??¦Ì2a¨º?
#define MODIFY_ID_TEST 0            // DT?????¨²ID2a¨º?
#define MODIFY_UNKNOWN_ID_TEST 0    // DT???¡ä?aID???¨²ID2a¨º?

uint16_t ms_count;

xdata uint8_t receive_data[20] = {0};
xdata uint8_t receive_len = 0;          // ?¨®¨º?¨®|¡äe¡ã¨¹3¡è?¨¨

// time03?¨º??¡¥¡Á??¨®¨º¡À
void timer0_init()
{
    TMOD |= 0x01;
    TH0 = 0xFC;
    TL0 = 0x67;
    ET0 = 1;
    EA = 1;
    TR0 = 1;
}

// ?¨®¨º¡Ào¡¥¨ºy
void delay_ms(uint16_t ms)
{
    ms_count =  2 * ms;   // ¨°¨°?a?a¨¢???¡À??¨´6T¡ê??¨´¨°??a¨¤??¨®¨º¡À¨º¡À??D¨¨¨°a3?¨°?2
    while (ms_count);
}

// ?¡§¨º¡À?¡Â?D??o¡¥¨ºy
void timer0_isr() interrupt 1 using 1
{
    TH0 = 0xFC;
    TL0 = 0x67;
    if (ms_count)
        ms_count--;
}

// ¡ä??¨²3?¨º??¡¥
void uart_init()
{
	TMOD|=0X20;	   // 8??¡Á??¡¥??¡Á¡ã???¡§¨º¡À?¡Â
	SCON=0X40;	   // 8??UART¡ê?2¡§¨¬??¨º?¨¦¡À?
	PCON=0X80;	   // 2¡§¨¬??¨º?¨®¡À?
	TH1=0xff;	   // ¨¦¨¨??2¡§¨¬??¨º?a115200
	TL1=0xff;
    ES=0;		   // 1?¡À??¨®¨º??D??
    EA=1;		   // CPU¡Á¨¹?D??
    TR1=1;		   // ?a???¡§¨º¡À?¡ÂT1?a¨º???¨ºy
}

void uart_init_recv()
{
    TMOD |= 0x20;  // 8??¡Á??¡¥??¡Á¡ã???¡§¨º¡À?¡Â
    SCON = 0x50;   // 8??UART¡ê?2¡§¨¬??¨º?¨¦¡À?¡ê?2¡é?a??¡ä?DD?¨®¨º?
    PCON = 0x80;   // 2¡§¨¬??¨º?¨®¡À?
    TH1 = 0xff;    // ¨¦¨¨??2¡§¨¬??¨º?a115200
    TL1 = 0xff;
    ES = 1;        // ?a???¨®¨º??D??
    EA = 1;        // CPU¡Á¨¹?D??
    TR1 = 1;       // ?a???¡§¨º¡À?¡ÂT1?a¨º???¨ºy
}

// ¡ä??¨²¡¤¡é?¨ªo¡¥¨ºy
void uart_send(uint8_t order_data)
{
    SBUF = order_data;      // ??¨ºy?YD¡ä¨¨?¡ä??¨²?o3???¡ä??¡Â?a¨º?¡ä?¨º?
    while(!TI);    			// ¦Ì¨¨¡äy¡ä?¨º?¨ª¨º3¨¦
    TI = 0;      			// ??3y¡ä?¨º?¨ª¨º3¨¦¡À¨º??
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
	xdata uint8_t order_buffer[20];												                    // ¡ä?¡¤?¨¦¨²3¨¦¦Ì???¨¢?
	xdata uint8_t order_buffer_len = 0;										                        // ??¨¢?3¡è?¨¨
	xdata uint16_t analysis_data = 0;											                    // ¨®|¡äe¡ã¨¹?a??3?¨¤¡ä¦Ì?¨ºy?Y
	xdata uint16_t sync_write_velocity_base_target_position[4] = {1, 0, 2, 0};                       // ¨ª?2?D¡ä?¨¤?????¨²???¨´??¡À¨º????
	xdata uint16_t sync_write_velocity_base_target_velocity[5] = {1, 3600, 2, 3600};                 // ¨ª?2?D¡ä?¨¤?????¨²???¨´??¡À¨º?¨´?¨¨
	xdata uint16_t sync_write_velocity_base_target_acc[5] = {1, 150, 2, 150};                        // ¨ª?2?D¡ä?¨¤?????¨²???¨´??¡À¨º?¨®?¨´?¨¨
	xdata uint16_t sync_write_velocity_base_target_dec[5] = {1, 150, 2, 150};                        // ¨ª?2?D¡ä?¨¤?????¨²???¨´??¡À¨º???¨´?¨¨
	xdata uint16_t sync_write_time_base_target_acc[5] = {1, 0, 2, 0};                                // ¨ª?2?D¡ä?¨¤?????¨²??¨º¡À??¡À¨º?¨®?¨´?¨¨
	xdata uint16_t sync_write_time_base_target_position_and_moving_time[10] = {1, 3000, 500, 2, 3000, 500};           // ¨ª?2?D¡ä?¨¤?????¨²??¨º¡À??¡À¨º???¡¥????o¨ª???¡¥¨º¡À??

	timer0_init();

	while(1)
	{
#if FACTORY_RESET_TEST
		// ???¡ä3?3¡ì¨¦¨¨??
		uart_init();
		servo_factory_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		
		uart_init_recv();
		delay_ms(10);
		servo_factory_reset_analysis(receive_data);
		delay_ms(1000);
#endif			
		
#if PARAMETER_RESET_TEST
        // 2?¨ºy????
		uart_init();
		servo_parameter_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_parameter_reset_analysis(receive_data);
		delay_ms(1000);
#endif			

#if CALIBRATION_TEST
        // D¡ê?y??¨°??¦Ì
		uart_init();
		servo_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_calibration_analysis(receive_data);
		delay_ms(1000);
#endif				
		
#if REBOOT_TEST
        // ???¨²????
		uart_init();
		servo_reboot(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		delay_ms(1000);
#endif		

#if PING_TEST
		// ?¨°ID?a1¦Ì????¨²¡¤¡é?¨ªPING??¨¢?
		uart_init();
		servo_ping(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_ping_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif	

#if MODIFY_ID_TEST
        // DT?????¨²ID2a¨º?
		uart_init();
		servo_modify_known_id(1, 2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if MODIFY_UNKNOWN_ID_TEST
        // DT???¡ä?aID???¨²ID2a¨º?
		uart_init();
		servo_modify_unknown_id(2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if READ_TEST
        // ?¨¢¨¨????¨²¦Ì?¦Ì¡À?¡ã¦Ì?¨¢¡Â
		uart_init();
		servo_read_present_current(1, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì¡À?¡ã????
		uart_init();
		servo_read_present_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì¡À?¡ã?¨´?¨¨
		uart_init();
		servo_read_present_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		// ?¨¢¨¨????¨²¦Ì?¦Ì¡À?¡ã¦Ì?1???????
		uart_init();
		servo_read_present_profile_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì¡À?¡ã1????¨´?¨¨
		uart_init();
		servo_read_present_profile_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì¡À?¡ãPWM
		uart_init();
		servo_read_present_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì¡À?¡ã???¨¨
		uart_init();
		servo_read_present_temperature(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_temperature_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì¡À?¡ã¨º?¨¨?¦Ì??1
		uart_init();
		servo_read_present_voltage(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_voltage_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì???¨º¡À??¡À¨º??DD¨º¡À??
		uart_init();
		servo_read_time_base_target_moving_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì???¨º¡À??¡À¨º????
		uart_init();
		servo_read_time_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì???¨º¡À?¨®?¨´?¨¨¦Ì¨¨??
		uart_init();
		servo_read_time_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì????¨´??¡À¨º???¨´?¨¨
		uart_init();
		servo_read_velocity_base_target_dec(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì????¨´??¡À¨º?¨®?¨´?¨¨
		uart_init();
		servo_read_velocity_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì????¨´??¡À¨º?¨´?¨¨
		uart_init();
		servo_read_velocity_base_target_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì????¨´??¡À¨º????
		uart_init();
		servo_read_velocity_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì???¡À¨º¦Ì?¨¢¡Â
		uart_init();
		servo_read_target_current(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì???¡À¨ºPWM
		uart_init();
		servo_read_target_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì??¡è???a1?
		uart_init();
		servo_read_torque_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_torque_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?LED?a1?
		uart_init();
		servo_read_led_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?Flash?a1?
		uart_init();
		servo_read_flash_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_flash_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì?¨¢¡ÂD¡ê?y?¦Ì
		uart_init();
		servo_read_current_offset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_offset_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì??D??D¡ê?y?¦Ì
		uart_init();
		servo_read_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_calibration_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_read_control_mode(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_control_mode_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?D???¡À¡ê?¡è¨¬??t
		uart_init();
		servo_read_shutdown_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?LED¡À¡§?¡¥¨¬??t
		uart_init();
		servo_read_led_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?????????D??¨°?
		uart_init();
		servo_read_position_control_d_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?????????I??¨°?
		uart_init();
		servo_read_position_control_i_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?????????P??¨°?
		uart_init();
		servo_read_position_control_p_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?PWM¦Ìt?¨®?¦Ì
		uart_init();
		servo_read_pwm_punch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_pwm_punch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¡¤¡ä¡Áa?¨¤??
		uart_init();
		servo_read_ccw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì??y¡Áa?¨¤??
		uart_init();
		servo_read_cw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_cw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì?¨¢¡Â¡À¡ê?¡è¨º¡À??
		uart_init();
		servo_read_current_shutdown_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì?¨¢¡Â¨¦??T
		uart_init();
		servo_read_max_current_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_current_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?PWM¨¦??T
		uart_init();
		servo_read_max_pwm_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì??1¨¦??T
		uart_init();
		servo_read_max_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¦Ì??1???T
		uart_init();
		servo_read_min_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì????¨¨¨¦??T
		uart_init();
		servo_read_max_temperature_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¡Á?¡ä¨®?????T??
		uart_init();
		servo_read_max_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¡Á?D??????T??
		uart_init();
		servo_read_min_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¡Á¡ä¨¬?¡¤¦Ì????¡Àe
		uart_init();
		servo_read_return_level(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_level_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?¨®|¡äe?¨®¨º¡À¨º¡À??
		uart_init();
		servo_read_return_delay_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_delay_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?2¡§¨¬??¨º
		uart_init();
		servo_read_baud_rate(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_baud_rate_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?3?3¡ì¡À¨¤o?
		uart_init();
		servo_read_model_information(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_model_information_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ?¨¢¨¨????¨²¦Ì?1¨¬?t¡ã?¡À?o?
		uart_init();
		servo_read_firmware_version(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_firmware_version_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if WRITE_TEST
        // ¨¦¨¨?????¨²¦Ì?Flash?a1?
		uart_init();
		servo_set_flash_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_flash_switch_analysis(receive_data);
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¨®|¡äe?¨®¨º¡À¨º¡À??
		uart_init();
		servo_set_return_delay_time(1, 250,order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_delay_time_analysis(receive_data);	 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¡Á¡ä¨¬?¡¤¦Ì????¡Àe
		uart_init();
		servo_set_return_level(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_level_analysis(receive_data);	   
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?2¡§¨¬??¨º
		uart_init();
		servo_set_baud_rate(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_baud_rate_analysis(receive_data);		 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¡Á?D??????T??
		uart_init();
		servo_set_min_angle_limit(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¡Á?¡ä¨®?????T??
		uart_init();
		servo_set_max_angle_limit(1, 3000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì????¨¨¨¦??T
		uart_init();
		servo_set_max_temperature_limit(1, 100, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_temperature_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¦Ì??1¨¦??T
		uart_init();
		servo_set_max_voltage_limit(1,90, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¦Ì??1???T
		uart_init();
		servo_set_min_voltage_limit(1, 33, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?PWM¨¦??T
		uart_init();
		servo_set_max_pwm_limit(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_pwm_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¦Ì?¨¢¡Â¨¦??T
		uart_init();
		servo_set_max_current_limit(1, 400, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_current_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¦Ì?¨¢¡Â¡À¡ê?¡è¨º¡À??
		uart_init();
		servo_set_current_shutdown_time(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_current_shutdown_time_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??y¡Áa?¨¤??
		uart_init();
		servo_set_cw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_cw_deadband_analysis(receive_data);		
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?¡¤¡ä¡Áa?¨¤??
		uart_init();
		servo_set_ccw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_ccw_deadband_analysis(receive_data); 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?PWM¦Ìt?¨®?¦Ì
		uart_init();
		servo_set_pwm_punch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_pwm_punch_analysis(receive_data);		 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?????????P??¨°?
		uart_init();
		servo_set_position_control_p_gain(1, 6000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_p_gain_analysis(receive_data); 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?????????I??¨°?
		uart_init();
		servo_set_position_control_i_gain(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_i_gain_analysis(receive_data);  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?????????D??¨°?
		uart_init();
		servo_set_position_control_d_gain(1, 151, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_d_gain_analysis(receive_data); 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?LED¡À¡§?¡¥¨¬??t
		uart_init();
		servo_set_led_condition(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_condition_analysis(receive_data);		   
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?D???¡À¡ê?¡è¨¬??t
		uart_init();
		servo_set_shutdown_conditions(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_shutdown_conditions_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì?LED?a1?
		uart_init();
		servo_set_led_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_switch_analysis(receive_data);	 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_set_control_mode(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì???¡À¨ºPWM
		uart_init();
		servo_set_target_pwm(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_pwm_analysis(receive_data);  
		delay_ms(3000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_set_control_mode(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì???¡À¨º¦Ì?¨¢¡Â
		uart_init();
		servo_set_target_current(1, -1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_current_analysis(receive_data);   
		delay_ms(3000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì????¨´??¡À¨º?¨´?¨¨
		uart_init();
		servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_velocity_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì????¨´??¡À¨º?¨®?¨´?¨¨
		uart_init();
		servo_set_velocity_base_target_acc(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_acc_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì????¨´??¡À¨º???¨´?¨¨
		uart_init();
		servo_set_velocity_base_target_dec(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_dec_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì????¨´??¡À¨º????
		uart_init();
		servo_set_velocity_base_target_position(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_velocity_base_target_position_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì???¨º¡À??¡À¨º?¨®?¨´?¨¨¦Ì¨¨??
		uart_init();
		servo_set_time_base_target_acc(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_acc_analysis(receive_data);	 
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì???¨º¡À??¡À¨º????o¨ª??¡À¨º??DD¨º¡À??
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);
#endif

#if SYNC_WRITE_TEST
        // ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_set_control_mode(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	   
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ¨¦¨¨???¨¤?????¨²¦Ì????¨´??¡À¨º?¨®?¨´?¨¨
		uart_init();
		servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ¨¦¨¨???¨¤?????¨²¦Ì????¨´??¡À¨º???¨´?¨¨
		uart_init();
		servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ¨¦¨¨???¨¤?????¨²¦Ì????¨´??¡À¨º?¨´?¨¨
		uart_init();
		servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ¨¦¨¨???¨¤?????¨²¦Ì????¨´??¡À¨º????
		uart_init();
		servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??????¡ê¨º?
		uart_init();
		servo_set_control_mode(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨?????¨²¦Ì??¡è???a1?
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	
		delay_ms(1000);

		// ¨¦¨¨???¨¤?????¨²¦Ì???¨º¡À??¡À¨º?¨®?¨´?¨¨¦Ì¨¨??
		uart_init();
		servo_sync_write_time_base_target_acc(1,sync_write_time_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ¨¦¨¨???¨¤?????¨²¦Ì???¨º¡À??¡À¨º????o¨ª???¡¥¨º¡À??
		uart_init();
		servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
#endif


	}		
}

void uart() interrupt 4
{
	if(RI)                                          // ?¨¬2¨¦?¨®¨º??D??¡À¨º??
	{
		RI = 0;                                     // ??3y?¨®¨º??D??¡À¨º??
		receive_data[receive_len++] = SBUF;         // ???¨®¨º?¦Ì?¦Ì?¨ºy?Y¡ä?¡ä¡é¦Ì??o3???
	}				
}


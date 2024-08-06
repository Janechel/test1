#include "reg52.h"
#include "servo.c"

#define READ_TEST 0                 // ¶ÁÈ¡¶æ»úÊý¾Ý²âÊÔ
#define WRITE_TEST 1                // Ð´Èë¶æ»úÊý¾Ý²âÊÔ
#define SYNC_WRITE_TEST 0           // Í¬²½Ð´²âÊÔ
#define PING_TEST 0                 // PINGÃüÁî²âÊÔ
#define FACTORY_RESET_TEST 0        // »Ö¸´³ö³§ÉèÖÃ²âÊÔ
#define PARAMETER_RESET_TEST 0      // ²ÎÊýÖØÖÃ²âÊÔ
#define REBOOT_TEST 0               // ÖØÆô²âÊÔ
#define CALIBRATION_TEST 0          // Ð£ÕýÆ«ÒÆÖµ²âÊÔ
#define MODIFY_ID_TEST 0            // ÐÞ¸Ä¶æ»úID²âÊÔ
#define MODIFY_UNKNOWN_ID_TEST 0    // ÐÞ¸ÄÎ´ÖªID¶æ»úID²âÊÔ

uint16_t ms_count;

xdata uint8_t receive_data[20] = {0};
xdata uint8_t receive_len = 0;          // ½ÓÊÕÓ¦´ð°ü³¤¶È

// time0³õÊ¼»¯×öÑÓÊ±
void timer0_init()
{
    TMOD |= 0x01;
    TH0 = 0xFC;
    TL0 = 0x67;
    ET0 = 1;
    EA = 1;
    TR0 = 1;
}

// ÑÓÊ±º¯Êý
void delay_ms(uint16_t ms)
{
    ms_count =  2 * ms;   // ÒòÎª¿ªÁËË«±¶ËÙ6T£¬ËùÒÔÕâÀïÑÓÊ±Ê±¼äÐèÒª³ËÒÔ2
    while (ms_count);
}

// ¶¨Ê±Æ÷ÖÐ¶Ïº¯Êý
void timer0_isr() interrupt 1 using 1
{
    TH0 = 0xFC;
    TL0 = 0x67;
    if (ms_count)
        ms_count--;
}

// ´®¿Ú³õÊ¼»¯
void uart_init()
{
	TMOD|=0X20;	   // 8Î»×Ô¶¯ÖØ×°ÔØ¶¨Ê±Æ÷
	SCON=0X40;	   // 8Î»UART£¬²¨ÌØÂÊ¿É±ä
	PCON=0X80;	   // ²¨ÌØÂÊ¼Ó±¶
	TH1=0xff;	   // ÉèÖÃ²¨ÌØÂÊÎª115200
	TL1=0xff;
    ES=0;		   // ¹Ø±Õ½ÓÊÕÖÐ¶Ï
    EA=1;		   // CPU×ÜÖÐ¶Ï
    TR1=1;		   // ¿ªÆô¶¨Ê±Æ÷T1¿ªÊ¼¼ÆÊý
}

void uart_init_recv()
{
    TMOD |= 0x20;  // 8Î»×Ô¶¯ÖØ×°ÔØ¶¨Ê±Æ÷
    SCON = 0x50;   // 8Î»UART£¬²¨ÌØÂÊ¿É±ä£¬²¢¿ªÆô´®ÐÐ½ÓÊÕ
    PCON = 0x80;   // ²¨ÌØÂÊ¼Ó±¶
    TH1 = 0xff;    // ÉèÖÃ²¨ÌØÂÊÎª115200
    TL1 = 0xff;
    ES = 1;        // ¿ªÆô½ÓÊÕÖÐ¶Ï
    EA = 1;        // CPU×ÜÖÐ¶Ï
    TR1 = 1;       // ¿ªÆô¶¨Ê±Æ÷T1¿ªÊ¼¼ÆÊý
}

// ´®¿Ú·¢ËÍº¯Êý
void uart_send(uint8_t order_data)
{
    SBUF = order_data;      // ½«Êý¾ÝÐ´Èë´®¿Ú»º³å¼Ä´æÆ÷¿ªÊ¼´«Êä
    while(!TI);    			// µÈ´ý´«ÊäÍê³É
    TI = 0;      			// Çå³ý´«ÊäÍê³É±êÖ¾
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
	xdata uint8_t order_buffer[20];												                    // ´æ·ÅÉú³ÉµÄÖ¸Áî
	xdata uint8_t order_buffer_len = 0;										                        // Ö¸Áî³¤¶È
	xdata uint16_t analysis_data = 0;											                    // Ó¦´ð°ü½âÎö³öÀ´µÄÊý¾Ý
	xdata uint16_t sync_write_velocity_base_target_position[4] = {1, 0, 2, 0};                       // Í¬²½Ð´¶à¸ö¶æ»ú¿ØËÙÄ¿±êÎ»ÖÃ
	xdata uint16_t sync_write_velocity_base_target_velocity[5] = {1, 3600, 2, 3600};                 // Í¬²½Ð´¶à¸ö¶æ»ú¿ØËÙÄ¿±êËÙ¶È
	xdata uint16_t sync_write_velocity_base_target_acc[5] = {1, 150, 2, 150};                        // Í¬²½Ð´¶à¸ö¶æ»ú¿ØËÙÄ¿±ê¼ÓËÙ¶È
	xdata uint16_t sync_write_velocity_base_target_dec[5] = {1, 150, 2, 150};                        // Í¬²½Ð´¶à¸ö¶æ»ú¿ØËÙÄ¿±ê¼õËÙ¶È
	xdata uint16_t sync_write_time_base_target_acc[5] = {1, 0, 2, 0};                                // Í¬²½Ð´¶à¸ö¶æ»ú¿ØÊ±Ä¿±ê¼ÓËÙ¶È
	xdata uint16_t sync_write_time_base_target_position_and_moving_time[10] = {1, 3000, 500, 2, 3000, 500};           // Í¬²½Ð´¶à¸ö¶æ»ú¿ØÊ±Ä¿±êÔË¶¯Î»ÖÃºÍÔË¶¯Ê±¼ä

	timer0_init();

	while(1)
	{
#if FACTORY_RESET_TEST
		// »Ö¸´³ö³§ÉèÖÃ
		uart_init();
		servo_factory_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		
		uart_init_recv();
		delay_ms(10);
		servo_factory_reset_analysis(receive_data);
		delay_ms(1000);
#endif			
		
#if PARAMETER_RESET_TEST
        // ²ÎÊýÖØÖÃ
		uart_init();
		servo_parameter_reset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_parameter_reset_analysis(receive_data);
		delay_ms(1000);
#endif			

#if CALIBRATION_TEST
        // Ð£ÕýÆ«ÒÆÖµ
		uart_init();
		servo_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_calibration_analysis(receive_data);
		delay_ms(1000);
#endif				
		
#if REBOOT_TEST
        // ¶æ»úÖØÆô
		uart_init();
		servo_reboot(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		delay_ms(1000);
#endif		

#if PING_TEST
		// ÏòIDÎª1µÄ¶æ»ú·¢ËÍPINGÖ¸Áî
		uart_init();
		servo_ping(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_ping_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif	

#if MODIFY_ID_TEST
        // ÐÞ¸Ä¶æ»úID²âÊÔ
		uart_init();
		servo_modify_known_id(1, 2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if MODIFY_UNKNOWN_ID_TEST
        // ÐÞ¸ÄÎ´ÖªID¶æ»úID²âÊÔ
		uart_init();
		servo_modify_unknown_id(2, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if READ_TEST
        // ¶ÁÈ¡¶æ»úµÄµ±Ç°µçÁ÷
		uart_init();
		servo_read_present_current(1, order_buffer, &order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµ±Ç°Î»ÖÃ
		uart_init();
		servo_read_present_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµ±Ç°ËÙ¶È
		uart_init();
		servo_read_present_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);
		
		// ¶ÁÈ¡¶æ»úµÄµ±Ç°µÄ¹æ»®Î»ÖÃ
		uart_init();
		servo_read_present_profile_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµ±Ç°¹æ»®ËÙ¶È
		uart_init();
		servo_read_present_profile_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		uart_init_recv();
		delay_ms(10);

		servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµ±Ç°PWM
		uart_init();
		servo_read_present_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµ±Ç°ÎÂ¶È
		uart_init();
		servo_read_present_temperature(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_temperature_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµ±Ç°ÊäÈëµçÑ¹
		uart_init();
		servo_read_present_voltage(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_present_voltage_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¿ØÊ±Ä¿±êÔËÐÐÊ±¼ä
		uart_init();
		servo_read_time_base_target_moving_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¿ØÊ±Ä¿±êÎ»ÖÃ
		uart_init();
		servo_read_time_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¿ØÊ±¼ÓËÙ¶ÈµÈ¼¶
		uart_init();
		servo_read_time_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¿ØËÙÄ¿±ê¼õËÙ¶È
		uart_init();
		servo_read_velocity_base_target_dec(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¿ØËÙÄ¿±ê¼ÓËÙ¶È
		uart_init();
		servo_read_velocity_base_target_acc(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¿ØËÙÄ¿±êËÙ¶È
		uart_init();
		servo_read_velocity_base_target_velocity(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¿ØËÙÄ¿±êÎ»ÖÃ
		uart_init();
		servo_read_velocity_base_target_position(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÄ¿±êµçÁ÷
		uart_init();
		servo_read_target_current(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_current_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÄ¿±êPWM
		uart_init();
		servo_read_target_pwm(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_target_pwm_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_read_torque_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_torque_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄLED¿ª¹Ø
		uart_init();
		servo_read_led_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄFlash¿ª¹Ø
		uart_init();
		servo_read_flash_switch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_flash_switch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµçÁ÷Ð£ÕýÖµ
		uart_init();
		servo_read_current_offset(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_offset_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÖÐÎ»Ð£ÕýÖµ
		uart_init();
		servo_read_calibration(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_calibration_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_read_control_mode(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_control_mode_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÐ¶ÔØ±£»¤Ìõ¼þ
		uart_init();
		servo_read_shutdown_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄLED±¨¾¯Ìõ¼þ
		uart_init();
		servo_read_led_condition(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_led_condition_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÎ»ÖÃ¿ØÖÆDÔöÒæ
		uart_init();
		servo_read_position_control_d_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÎ»ÖÃ¿ØÖÆIÔöÒæ
		uart_init();
		servo_read_position_control_i_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÎ»ÖÃ¿ØÖÆPÔöÒæ
		uart_init();
		servo_read_position_control_p_gain(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄPWMµþ¼ÓÖµ
		uart_init();
		servo_read_pwm_punch(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_pwm_punch_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ·´×ªËÀÇø
		uart_init();
		servo_read_ccw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÕý×ªËÀÇø
		uart_init();
		servo_read_cw_deadband(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_cw_deadband_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµçÁ÷±£»¤Ê±¼ä
		uart_init();
		servo_read_current_shutdown_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµçÁ÷ÉÏÏÞ
		uart_init();
		servo_read_max_current_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_current_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄPWMÉÏÏÞ
		uart_init();
		servo_read_max_pwm_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµçÑ¹ÉÏÏÞ
		uart_init();
		servo_read_max_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄµçÑ¹ÏÂÏÞ
		uart_init();
		servo_read_min_voltage_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÎÂ¶ÈÉÏÏÞ
		uart_init();
		servo_read_max_temperature_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ×î´óÎ»ÖÃÏÞÖÆ
		uart_init();
		servo_read_max_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ×îÐ¡Î»ÖÃÏÞÖÆ
		uart_init();
		servo_read_min_angle_limit(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ×´Ì¬·µ»Ø¼¶±ð
		uart_init();
		servo_read_return_level(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_level_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄÓ¦´ðÑÓÊ±Ê±¼ä
		uart_init();
		servo_read_return_delay_time(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_return_delay_time_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ²¨ÌØÂÊ
		uart_init();
		servo_read_baud_rate(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_baud_rate_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ³ö³§±àºÅ
		uart_init();
		servo_read_model_information(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_model_information_analysis(receive_data, &analysis_data);
		delay_ms(1000);

		// ¶ÁÈ¡¶æ»úµÄ¹Ì¼þ°æ±¾ºÅ
		uart_init();
		servo_read_firmware_version(1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_read_firmware_version_analysis(receive_data, &analysis_data);
		delay_ms(1000);
#endif

#if WRITE_TEST
        // ÉèÖÃ¶æ»úµÄFlash¿ª¹Ø
		uart_init();
		servo_set_flash_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_flash_switch_analysis(receive_data);
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÓ¦´ðÑÓÊ±Ê±¼ä
		uart_init();
		servo_set_return_delay_time(1, 250,order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_delay_time_analysis(receive_data);	 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ×´Ì¬·µ»Ø¼¶±ð
		uart_init();
		servo_set_return_level(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_return_level_analysis(receive_data);	   
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ²¨ÌØÂÊ
		uart_init();
		servo_set_baud_rate(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_baud_rate_analysis(receive_data);		 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ×îÐ¡Î»ÖÃÏÞÖÆ
		uart_init();
		servo_set_min_angle_limit(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ×î´óÎ»ÖÃÏÞÖÆ
		uart_init();
		servo_set_max_angle_limit(1, 3000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_angle_limit_analysis(receive_data);	 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÎÂ¶ÈÉÏÏÞ
		uart_init();
		servo_set_max_temperature_limit(1, 100, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_temperature_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄµçÑ¹ÉÏÏÞ
		uart_init();
		servo_set_max_voltage_limit(1,90, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄµçÑ¹ÏÂÏÞ
		uart_init();
		servo_set_min_voltage_limit(1, 33, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_min_voltage_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄPWMÉÏÏÞ
		uart_init();
		servo_set_max_pwm_limit(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_pwm_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄµçÁ÷ÉÏÏÞ
		uart_init();
		servo_set_max_current_limit(1, 400, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_max_current_limit_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄµçÁ÷±£»¤Ê±¼ä
		uart_init();
		servo_set_current_shutdown_time(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_current_shutdown_time_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÕý×ªËÀÇø
		uart_init();
		servo_set_cw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_cw_deadband_analysis(receive_data);		
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ·´×ªËÀÇø
		uart_init();
		servo_set_ccw_deadband(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_ccw_deadband_analysis(receive_data); 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄPWMµþ¼ÓÖµ
		uart_init();
		servo_set_pwm_punch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_pwm_punch_analysis(receive_data);		 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÎ»ÖÃ¿ØÖÆPÔöÒæ
		uart_init();
		servo_set_position_control_p_gain(1, 6000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_p_gain_analysis(receive_data); 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÎ»ÖÃ¿ØÖÆIÔöÒæ
		uart_init();
		servo_set_position_control_i_gain(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_i_gain_analysis(receive_data);  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÎ»ÖÃ¿ØÖÆDÔöÒæ
		uart_init();
		servo_set_position_control_d_gain(1, 151, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_position_control_d_gain_analysis(receive_data); 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄLED±¨¾¯Ìõ¼þ
		uart_init();
		servo_set_led_condition(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_condition_analysis(receive_data);		   
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÐ¶ÔØ±£»¤Ìõ¼þ
		uart_init();
		servo_set_shutdown_conditions(1, 36, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_shutdown_conditions_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄLED¿ª¹Ø
		uart_init();
		servo_set_led_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_led_switch_analysis(receive_data);	 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_set_control_mode(1, 3, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÄ¿±êPWM
		uart_init();
		servo_set_target_pwm(1, 1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_pwm_analysis(receive_data);  
		delay_ms(3000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_set_control_mode(1, 2, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÄ¿±êµçÁ÷
		uart_init();
		servo_set_target_current(1, -1000, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_target_current_analysis(receive_data);   
		delay_ms(3000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØËÙÄ¿±êËÙ¶È
		uart_init();
		servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_velocity_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØËÙÄ¿±ê¼ÓËÙ¶È
		uart_init();
		servo_set_velocity_base_target_acc(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_acc_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØËÙÄ¿±ê¼õËÙ¶È
		uart_init();
		servo_set_velocity_base_target_dec(1, 150, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_velocity_base_target_dec_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØËÙÄ¿±êÎ»ÖÃ
		uart_init();
		servo_set_velocity_base_target_position(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_velocity_base_target_position_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÊ±Ä¿±ê¼ÓËÙ¶ÈµÈ¼¶
		uart_init();
		servo_set_time_base_target_acc(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_acc_analysis(receive_data);	 
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÊ±Ä¿±êÎ»ÖÃºÍÄ¿±êÔËÐÐÊ±¼ä
		uart_init();
		servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_buffer_len);

		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);

		servo_set_time_base_target_position_and_moving_time_analysis(receive_data);		 
		delay_ms(1000);
#endif

#if SYNC_WRITE_TEST
        // ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_set_control_mode(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_set_control_mode(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	   
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);		
		delay_ms(1000);

		// ÉèÖÃ¶à¸ö¶æ»úµÄ¿ØËÙÄ¿±ê¼ÓËÙ¶È
		uart_init();
		servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ÉèÖÃ¶à¸ö¶æ»úµÄ¿ØËÙÄ¿±ê¼õËÙ¶È
		uart_init();
		servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ÉèÖÃ¶à¸ö¶æ»úµÄ¿ØËÙÄ¿±êËÙ¶È
		uart_init();
		servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ÉèÖÃ¶à¸ö¶æ»úµÄ¿ØËÙÄ¿±êÎ»ÖÃ
		uart_init();
		servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_set_control_mode(1, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(1, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	  
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	   
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄ¿ØÖÆÄ£Ê½
		uart_init();
		servo_set_control_mode(2, 0, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_control_mode_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶æ»úµÄÅ¤¾Ø¿ª¹Ø
		uart_init();
		servo_set_torque_switch(2, 1, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);

		uart_init_recv();
		delay_ms(10);
		servo_set_torque_switch_analysis(receive_data);	
		delay_ms(1000);

		// ÉèÖÃ¶à¸ö¶æ»úµÄ¿ØÊ±Ä¿±ê¼ÓËÙ¶ÈµÈ¼¶
		uart_init();
		servo_sync_write_time_base_target_acc(1,sync_write_time_base_target_acc, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);

		// ÉèÖÃ¶à¸ö¶æ»úµÄ¿ØÊ±Ä¿±êÎ»ÖÃºÍÔË¶¯Ê±¼ä
		uart_init();
		servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer,&order_buffer_len);
		uart_send_buffer(order_buffer, order_buffer_len);
		delay_ms(1000);
#endif


	}		
}

void uart() interrupt 4
{
	if(RI)                                          // ¼ì²é½ÓÊÕÖÐ¶Ï±êÖ¾
	{
		RI = 0;                                     // Çå³ý½ÓÊÕÖÐ¶Ï±êÖ¾
		receive_data[receive_len++] = SBUF;         // ½«½ÓÊÕµ½µÄÊý¾Ý´æ´¢µ½»º³åÇø
	}				
}


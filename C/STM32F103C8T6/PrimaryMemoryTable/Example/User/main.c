#include "stm32f10x.h"
#include "platform_config.h"
#include "PrimaryServo.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

#define PING_TEST 0   					//PING Instruction Test
#define READ_TEST 0						//Read Servo Data Test
#define WRITE_TEST 0					//Write Servo Data Test
#define SYNC_WRITE_TEST 0				//Sync Write Test
#define FACTORY_RESET_TEST 0			//Factory Reset Test
#define PARAMETER_RESET_TEST 0	        //Parameter Reset Test
#define CALIBRATION_TEST 0				//Calibration Test
#define REBOOT_TEST 0					//Reboot Test
#define MODIFY_ID 0                     //Change Known Servo ID Test
#define MODIFY_UNKNOWN_ID 0             //Change Unknown Servo ID Test

uint8_t receive_data[20];                      //Store the received status packet
uint8_t receive_len;                           //received Length
uint8_t ret;                                   //Status Flag
uint16_t position = 0;                         //present position
uint16_t current = 0;                          //present current
uint8_t write_buffer[20] = {0};                //Write data to the memory table

extern __IO uint32_t TimingDelay;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Init(void);
void USART2_Init(void);
void USART1_Send(uint8_t *data, uint8_t data_len);
void SysTick_Configuration(void);
void Delay(__IO uint32_t nTime);

//Redirect printf
int fputc(int ch, FILE *f)
{
  USART_SendData(USART2, (uint8_t) ch);

  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}
		
	return ch;
}

int main(void)
{
	uint8_t order_buffer[20];           //Store Generated Instructions
	uint8_t order_len;                  //Instruction Length

	uint16_t analysis_data;             //Data parsed from the status packet

	SysTick_Configuration();
	RCC_Configuration();
	GPIO_Configuration();

	USART1_Init();
	
	//Information print serial port initialization
	USART2_Init();
	
	struct servo_sync_parameter servo;
	
	servo.id_counts = 2;            //Sync write two servos
    servo.id[0] = 1;                //Set the ID of the first servo to 1
    servo.id[1] = 2;                //Set the ID of the second servo to 2
	
  while (1)
  {
#if FACTORY_RESET_TEST
		//Reset the servo to the factory default values.
		servo_factory_reset(1, order_buffer,&order_len);

		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_factory_reset_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("FACTORY RESET success");
		Delay(1000);
#endif			
		
#if PARAMETER_RESET_TEST
		//Reset the parameter settings of the servo.
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
#endif			

#if CALIBRATION_TEST
		//Calibrate the midpoint of the servo.
    servo_calibration(1, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
    ret = servo_calibration_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("CALIBRATION SUCCESS");
		Delay(1000);
#endif				
		
#if REBOOT_TEST
		//Reboot the servo ID1.
    servo_reboot(1, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("The servo reboot");
		
		Delay(1000);
#endif			

#if MODIFY_ID
		//Change the servo ID of servo ID1 to 2.
    servo_modify_known_id(1, 2, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		PRINTF("modify id success");
		Delay(1000);
#endif

#if MODIFY_UNKNOWN_ID
		//Change the servo ID of the servo with an unknown ID to 1.
    servo_modify_unknown_id(1, order_buffer,&order_len);

    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		PRINTF("modify id success");
		Delay(1000);
#endif

#if PING_TEST
		//Query the model number of servo ID1.
    servo_ping(1, order_buffer,&order_len);
    
		USART1_Send(order_buffer, order_len);
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    ret = servo_ping_analysis(receive_data, &analysis_data);
    if(ret == SUCCESS)
			PRINTF("The servo exists");
		Delay(1000);
#endif

#if READ_TEST
		//Read the present current of servo ID1.
    servo_read_present_current(1, order_buffer,&order_len);
    
		USART1_Send(order_buffer, order_len);
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
    ret = servo_read_present_current_analysis(receive_data, &analysis_data);
    if(ret == SUCCESS)
		{
			PRINTF("present current is %d",analysis_data);
		}
		Delay(1000);
#endif

#if READ_TEST
		//Read the present position of servo ID1.
    servo_read_present_position(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_position_analysis(receive_data, &analysis_data);
    if(ret == SUCCESS)
		{
			PRINTF("present position is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //Read the present position and present current of servo ID1.
    servo_read_present_position_and_present_current(1, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_position_and_present_current_analysis(receive_data, &position, &current);
    if(ret == SUCCESS)
		{
			PRINTF("present position is : % d, present current is : % d\r\n", position, current);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the present velocity of servo ID1.
    servo_read_present_velocity(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_velocity_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the present profile position of servo ID1.
    servo_read_present_profile_position(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_profile_position_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present profile position is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the present profile velocity of servo ID1.
    servo_read_present_profile_velocity(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_profile_velocity_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present profile velocity is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the present PWM of servo ID1.
    servo_read_present_pwm(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_pwm_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present pwm analysis is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the present temperature of servo ID1.
    servo_read_present_temperature(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_temperature_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present temperature is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the present voltage of servo ID1.
    servo_read_present_voltage(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_present_voltage_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present voltage is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the time base target moving time of servo ID1.
    servo_read_time_base_target_moving_time(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_time_base_target_moving_time_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present time base target moving time is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the time base target position of servo ID1.
    servo_read_time_base_target_position(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_time_base_target_position_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present time base target position is %d",analysis_data);
		}
		
		Delay(1000);
#endif

#if READ_TEST
		//Read the time base target ACC of servo ID1.
    servo_read_time_base_target_acc(1, order_buffer,&order_len);   
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_time_base_target_acc_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present time base target acc is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //Read the time base target position and moving time of servo ID1.
    servo_read(1, 0x3C, 4, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		PRINTF("the time base target position and moving time pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
    //Read the time base target ACC, position and moving time of servo ID1.
    servo_read(1, 0x3B, 5, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		PRINTF("the time base target acc, position and moving time pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
		//Read the velocity base target DEC of servo ID1.
    servo_read_velocity_base_target_dec(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
    ret = servo_read_velocity_base_target_dec_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity base target dec is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the velocity base target ACC of servo ID1.
    servo_read_velocity_base_target_acc(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_velocity_base_target_acc_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity base target acc is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the velocity base target velocity of servo ID1.
    servo_read_velocity_base_target_velocity(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_velocity_base_target_velocity_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity base target velocity is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the velocity base target position of servo ID1.
    servo_read_velocity_base_target_position(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_velocity_base_target_position_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("present velocity base target position is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //Read the velocity base target position and velocity of servo ID1.
    servo_read(1, 0x35, 4, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the velocity base target position and velocity pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
    //Read the velocity base target position, velocity, ACC, and DEC of servo ID1.
    servo_read(1, 0x35, 6, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the velocity base target position,velocity,acc and dec pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
		//Read the target current of servo ID1.
    servo_read_target_current(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_target_current_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("target current is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the target PWM of servo ID1.
    servo_read_target_pwm(1, order_buffer,&order_len);  
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_target_pwm_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("target pwm is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the torque switch of servo ID1.
    servo_read_torque_switch(1, order_buffer,&order_len);  
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_torque_switch_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("torque switch is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the LED switch of servo ID1.
    servo_read_led_switch(1, order_buffer,&order_len);   
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_led_switch_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("led switch is %d",analysis_data);
		}
		
		Delay(1000);
#endif

#if READ_TEST
		//Read the Flash switch of servo ID1.
    servo_read_flash_switch(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_flash_switch_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("flash switch is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the current offset of servo ID1.
    servo_read_current_offset(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_current_offset_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("current offset is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the calibration of servo ID1.
    servo_read_calibration(1, order_buffer,&order_len);  
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_calibration_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("calibration is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the control mode of servo ID1.
    servo_read_control_mode(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_control_mode_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("control mode is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the shutdown condition of servo ID1.
    servo_read_shutdown_condition(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_shutdown_condition_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("shutdown condition is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the LED condition of servo ID1.
    servo_read_led_condition(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_led_condition_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("led condition is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the position control D gain of servo ID1.
    servo_read_position_control_d_gain(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_position_control_d_gain_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("position control d gain is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the position control I gain of servo ID1.
    servo_read_position_control_i_gain(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_position_control_i_gain_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("position control i gain is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the position control P gain of servo ID1.
    servo_read_position_control_p_gain(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_position_control_p_gain_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("position control p gain is %d",analysis_data);
		}
		
		Delay(1000);
#endif
			
#if READ_TEST
    //Read the position control PID gain of servo ID1.
    servo_read(1, 0x1B, 6, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("position control pid gain pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
		//Read the PWM punch of servo ID1.
    servo_read_pwm_punch(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_pwm_punch_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("pwm punch is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the ccw deadband of servo ID1.
    servo_read_ccw_deadband(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_ccw_deadband_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("ccw deadband is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the cw deadband of servo ID1.
    servo_read_cw_deadband(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_cw_deadband_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("cw deadband is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的电流保护时间
    servo_read_current_shutdown_time(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_current_shutdown_time_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("current shutdown time is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the max current limit of servo ID1.
    servo_read_max_current_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_max_current_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max current limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the max PWM limit of servo ID1.
    servo_read_max_pwm_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_max_pwm_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max pwm limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the max voltage limit of servo ID1.
    servo_read_max_voltage_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_max_voltage_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max voltage limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the min voltage limit of servo ID1.
    servo_read_min_voltage_limit(1, order_buffer,&order_len);   
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_min_voltage_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("min voltage limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //Read the voltage limit of servo ID1.
    servo_read(1, 0x10, 2, order_buffer, &order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the voltage limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif


#if READ_TEST
		//读取ID1舵机的温度上限
    servo_read_max_temperature_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		
    ret = servo_read_max_temperature_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max temperature limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif

#if READ_TEST
		//Read the max angle limit of servo ID1.
    servo_read_max_angle_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_max_angle_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("max angle limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the min angle limit of servo ID1.
    servo_read_min_angle_limit(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_min_angle_limit_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("min angle limit is %d",analysis_data);
		}
		
		Delay(1000);
#endif
		
#if READ_TEST
    //Read the angle limit of servo ID1.
    servo_read(1, 0x0B, 4, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the angle limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		
		Delay(1000);
#endif

#if READ_TEST
		//Read the return level of servo ID1.
    servo_read_return_level(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_return_level_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("return level is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the return delay time of servo ID1.
    servo_read_return_delay_time(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_return_delay_time_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("return delay time is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the baud rate of servo ID1.
    servo_read_baud_rate(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_baud_rate_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("baud rate is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the model information of servo ID1.
    servo_read_model_information(1, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_model_information_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("model information is %d",analysis_data);
		}
		
		Delay(1000);
#endif


#if READ_TEST
		//Read the firmware version of servo ID1.
    servo_read_firmware_version(1, order_buffer,&order_len); 
    USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_read_firmware_version_analysis(receive_data, &analysis_data);
		if(ret == SUCCESS)
		{
			PRINTF("firmware version is %d",analysis_data);
		}
		
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the torque switch of the servo ID1, ID2 to OFF respectively.
		servo.torque_switch[0] = 0;
		servo.torque_switch[1] = 0;
		servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write torque switch!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write control mode!");
		Delay(1000);
#endif

	
#if SYNC_WRITE_TEST
		//Change the velocity base target velocity of the servo ID1, ID2 to 360°/s² and 720°/s², respectively.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;
		
		servo_sync_write_velocity_base_target_velocity(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target velocity!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target ACC of servo ID1, ID2 to 500°/s² and 50°/s², respectively.
    servo.acc_velocity[0] = 10;          
    servo.acc_velocity[1] = 1;    
		
		servo_sync_write_velocity_base_target_acc(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target acc!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target DEC of servo ID1, ID2 to 50°/s² and 500°/s², respectively.
    servo.dec_velocity[0] = 1;           
    servo.dec_velocity[1] = 10;   
		
		servo_sync_write_velocity_base_target_dec(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target dec!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target velocity of the servo ID1, ID2 to 0° midpoint and 0° position, respectively.
    servo.position[0] = 0;
    servo.position[1] = 0;
		
		servo_sync_write_velocity_base_target_position(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target position!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the velocity base target velocity of servo ID1 ,ID2 to 1800 and 3600, and the position to 3000 and 3000, respectively
    servo.velocity[0] = 1800;
    servo.velocity[1] = 3600;
    servo.position[0] = 3000;
    servo.position[1] = 3000;
		
		servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target position and velocity!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //SChange the velocity base target velocity of servo ID1 ,ID2 to 3600 and 3600, position to 0,0, acceleration to 100, 100, deceleration to 100, 100, respectively
    servo.velocity[0] = 3600;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 0;
    servo.acc_velocity[0] = 100;
    servo.acc_velocity[1] = 100;
    servo.dec_velocity[0] = 100;
    servo.dec_velocity[1] = 100;

		servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write velocity base target acc dec velocity and position!");
		Delay(1000);

#endif

#if SYNC_WRITE_TEST
    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
		servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write torque switch!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
    //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write control mode!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the time base target ACC of servo ID1 to 1 and 5 respectively.
    servo.acc_velocity_grade[0] = 1;
    servo.acc_velocity_grade[1] = 5;
		
    servo_sync_write_time_base_target_acc(servo, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);
		PRINTF("sync write time base target acc!");
		Delay(1000);
#endif

#if SYNC_WRITE_TEST
		//Change the time base target position and moving time of servo ID1 to 150° midpoint and 1s, 0° and 500ms respectively.
    servo.position[0] = 1500;
    servo.position[1] = 0;
    servo.time[0] = 1000;
    servo.time[1] = 500;
		
		servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer,&order_len);
    USART1_Send(order_buffer, order_len);
		PRINTF("sync write time base target position and moving time!");
		Delay(1000);
#endif

#if WRITE_TEST
		//Change the return level of servo ID1 to respond to all instruction.
    servo_set_return_level(1, 2, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_return_level_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set return level success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the return delay time of servo ID1 to 500us.
		servo_set_return_delay_time(1, 250,order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_return_delay_time_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set return delay time success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the baud rate of servo ID1 to 1000000.
    servo_set_baud_rate(1, 7, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_baud_rate_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set baud rate success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the min angle limit of servo ID1 to 0°.
    servo_set_min_angle_limit(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_min_angle_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set min angle limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the max angle limit of servo ID1 to 300°.
    servo_set_max_angle_limit(1, 3000, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_angle_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max angle limit success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //Change the angle limit of servo ID1 to 0°~300°.
    write_buffer[0] = 0 & 0xff;;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3000 & 0xff;
    write_buffer[3] = (3000 >> 8) & 0xff;
		servo_write(1, 0x0B, 4, write_buffer, order_buffer, &order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("servo set angle limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the max temperature limit of servo ID1 to 65℃.
    servo_set_max_temperature_limit(1, 65, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_temperature_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max temperature limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the max voltage limit of servo ID1 to 8.4V.
    servo_set_max_voltage_limit(1,84, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_voltage_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max voltage limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the min voltage limit of servo ID1 to 3.5V.
    servo_set_min_voltage_limit(1, 35, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_min_voltage_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set min voltage limit success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //Change the voltage limit of servo ID1 to 3.5~8.4V.
    write_buffer[0] = 84 & 0xff;
    write_buffer[1] = 35 & 0xff;
    servo_write(1, 0x10, 2, write_buffer, order_buffer, &order_len);

		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("the voltage limit pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the max PWM limit of servo ID1 to 90%.
    servo_set_max_pwm_limit(1, 900, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_pwm_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max pwm limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the max current limit of servo ID1 to 900mA.
    servo_set_max_current_limit(1, 900, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_max_current_limit_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set max current limit success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the current shutdown time of servo ID1 to 500ms.
    servo_set_current_shutdown_time(1, 500, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_current_shutdown_time_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set current shutdown time success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the CW deadband of servo ID1 to 0.2°.
    servo_set_cw_deadband(1, 2, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_cw_deadband_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set cw deadband success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the CCW deadband of servo ID1 to 0.2°.
    servo_set_ccw_deadband(1, 2, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_ccw_deadband_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set ccw deadband success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the CW and CCW deadband of servo ID1 to 0.2°.
    write_buffer[0] = 2 & 0xff;
    write_buffer[1] = 2 & 0xff;
    servo_write(1, 0x18, 2, write_buffer, order_buffer, &order_len);
		
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("servo set cw deadband and ccw deadband pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the PWM punch of servo ID1 to 1%.
    servo_set_pwm_punch(1, 10, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_pwm_punch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set pwm punch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the position control P gain of servo ID1 to 5995.
    servo_set_position_control_p_gain(1, 5995, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_position_control_p_gain_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set position control p gain success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the position control D gain of servo ID1 to 5.
    servo_set_position_control_i_gain(1, 5, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_position_control_i_gain_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set position control i gain success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the position control D gain of servo ID1 to 145.
    servo_set_position_control_d_gain(1, 145, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_position_control_d_gain_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set position control d gain success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //Change the position control PID gain of servo ID1 to 5995, 5, and 145 respectively.
    write_buffer[0] = 5995 & 0xff;
    write_buffer[1] = (5995 >> 8) & 0xff;
    write_buffer[2] = 5 & 0xff;
    write_buffer[3] = (5 >> 8) & 0xff;
    write_buffer[4] = 145 & 0xff;
    write_buffer[5] = (145 >> 8) & 0xff;
    servo_write(1, 0x1B, 6, write_buffer, order_buffer, &order_len);
		
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    PRINTF("servo set position control pid gain pack is: ");
		for (uint8_t i = 0; i < receive_len; i++)
		{
				PRINTF("0x%x ", receive_data[i]);
		}
		PRINTF("\r\n");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the LED condition of servo ID1 to turn on stall error, overheating error, and angle error.
    servo_set_led_condition(1, 38, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_led_condition_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set led condition success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the shutdown condition of servo ID1 to turn on stall error, overheating error, voltage error, and angle error.
    servo_set_shutdown_conditions(1, 39, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_shutdown_conditions_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set shutdown conditions success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the Flash switch of servo ID1 to ON.
    servo_set_flash_switch(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_flash_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set flash switch success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //Change the Flash switch of servo ID1 to OFF.
    servo_set_flash_switch(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_flash_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set flash switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the LED switch of servo ID1 to ON.
    servo_set_led_switch(1, 1, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_led_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set led switch success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the LED switch of servo ID1 to OFF.
    servo_set_led_switch(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_led_switch_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set led switch success");
		Delay(1000);
#endif
		
#if WRITE_TEST
    //Change the torque switch of servo ID1 to OFF.
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
#endif

#if WRITE_TEST
    //Change the control mode of servo ID1 to the PWM control mode.
    servo_set_control_mode(1, 3, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_control_mode_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set control mode success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to ON.
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
#endif

#if WRITE_TEST
    //Change the target PWM of servo ID1 to -50%.
    servo_set_target_pwm(1, -500, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_target_pwm_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set target pwm success");
		Delay(3000);
#endif
		
		#if WRITE_TEST
    //Change the torque switch of servo ID1 to OFF.
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
#endif

#if WRITE_TEST
    //Change the control mode of servo ID1 to the current control mode.
    servo_set_control_mode(1, 2, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_control_mode_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set control mode success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to ON.
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
#endif

#if WRITE_TEST
    //Change the target current of servo ID1 to -400mA.
    servo_set_target_current(1, -400, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_target_current_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set target current success");
		Delay(3000);
#endif
		
#if WRITE_TEST
    //Change the torque switch of servo ID1 to OFF.
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
#endif

#if WRITE_TEST
    //Change the control mode of servo ID1 to the velocity base position control mode.
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
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to ON.
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
#endif

#if WRITE_TEST
    //Change the velocity base target velocity of servo ID1 to 360°/s.
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
#endif

#if WRITE_TEST
    //Change the velocity base target ACC of servo ID1 to 500°/s².
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
#endif

#if WRITE_TEST
    //Change the velocity base target DEC of servo ID1 to 50°/s².
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
#endif

#if WRITE_TEST
    //Change the velocity base target position of servo ID1 to 150°.
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
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to OFF.
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
#endif

#if WRITE_TEST
    //Change the control mode of servo ID1 to the time base position control mode.
    servo_set_control_mode(1, 0, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_control_mode_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set control mode success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the torque switch of servo ID1 to ON.
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
#endif
		
#if WRITE_TEST
    //Change the time base target ACC of servo ID1 to 5.
    servo_set_time_base_target_acc(1, 5, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_time_base_target_acc_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set time base target acc success");
		Delay(1000);
#endif

#if WRITE_TEST
    //Change the time base target position and moving time of servo ID1 to 300°, 500ms respectively.
    servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_len);
		USART1_Send(order_buffer, order_len);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    receive_len = 0x00;
		Delay(10);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    ret = servo_set_time_base_target_position_and_moving_time_analysis(receive_data);
		if(ret == SUCCESS)
			PRINTF("servo set time base target position and moving time success");
		Delay(1000);
#endif
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	//USART2_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
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
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	//Enable USART1 half-duplex mode
  USART_HalfDuplexCmd(USART1, ENABLE);

  //Enable USART1
  USART_Cmd(USART1, ENABLE);
}

void USART2_Init()
{
	USART_InitTypeDef USART_InitStructure;
	
	USART_DeInit(USART2);
 
	USART_InitStructure.USART_BaudRate = 1000000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE);         
}

void USART1_Send(uint8_t *data, uint8_t data_len)
{
  for(uint8_t i = 0; i < data_len; i++)
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
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

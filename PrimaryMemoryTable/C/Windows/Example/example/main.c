#include "servo.h"
#include <windows.h>
#include <stdio.h>

#define READ_TEST 0                 //Read Servo Data Test
#define WRITE_TEST 0                //Write Servo Data Test
#define SYNC_WRITE_TEST 0           //Sync Write Test
#define PING_TEST 0                 //PING Instruction Test
#define FACTORY_RESET_TEST 0        //Factory Reset Test
#define PARAMETER_RESET_TEST 0      //Parameter Reset Test
#define REBOOT_TEST 0               //Reboot Test
#define CALIBRATION_TEST 0          //Calibration Test
#define MODIFY_ID 0                 //Change Known Servo ID Test
#define MODIFY_UNKNOWN_ID 0         //Change Unknown Servo ID Test

struct servo_sync_parameter servo;

//uart init
uint8_t uart_init(HANDLE hSerial)
{
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        PRINTF("Failed to open serial port\n");
        return FALSE;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        PRINTF("Failed to get serial port parameters\n");
        CloseHandle(hSerial);
        return FALSE;
    }

    //Set serial port protocol parameters
    dcbSerialParams.BaudRate = 1000000;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        PRINTF("Failed to set serial port parameters\n");
        CloseHandle(hSerial);
        return FALSE;
    }

    return TRUE;

}

//uaer send
uint8_t order_send(HANDLE hSerial, uint8_t* order_buffer, uint8_t order_len)
{
    uint8_t ret;                    //Status Flag
    DWORD bytesWritten;             //Actual Data Length Written

    ret = WriteFile(hSerial, order_buffer, order_len, &bytesWritten, NULL);

    if (ret != 0)
    {
        return TRUE;
    }
    else
    {
        PRINTF("send error!\r\n");
        return FALSE;
    }
}

//uart receiver
uint8_t order_receive(HANDLE hSerial, uint8_t pack[])
{
    uint8_t ret;                //Status Flag
    DWORD bytesRead;            
    DWORD errors;               
    DWORD read_len;             
    COMSTAT comstat;            //Describes the status information of serial port communication

    if (!ClearCommError(hSerial, &errors, &comstat)) {
        return FALSE;
    }

    //Gets the number of bytes available in the receive buffer
    read_len = comstat.cbInQue;

    ret = ReadFile(hSerial, pack, read_len, &bytesRead, NULL);

    if (ret != 0)
    {
        if (bytesRead > 0)
        {
            return bytesRead;
        }
        else
        {
            PRINTF("\r\nNo response packet data!\r\n");
            return TRUE;
        }
    }
    else
    {
        PRINTF("read error!\r\n");
        return FALSE;
    }
}

int main() {
    uint8_t ret = 0;                                                                                        //Status Flag
    uint8_t order_buffer[50] = { 0 };                                                                       //Store Generated Instructions
    uint8_t order_len = 0;                                                                                  //Instruction Length
    uint8_t pack[20] = { 0 };                                                                               //Store the received status packet
    uint16_t analysis_data = 0;                                                                             //Data parsed from the status packet
    uint16_t position = 0;                                                                                  //present position
    uint16_t current = 0;                                                                                   //present current
    uint8_t write_buffer[20] = { 0 };                                                                         //Write data to the memory table

    //Open the serial port
    HANDLE hSerial = CreateFile("\\\\.\\COM18", GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    ret = uart_init(hSerial);
    if (ret == FALSE)
    {
        return FALSE;
    }

#if PING_TEST
    //Query the model number of servo ID1.
    servo_ping(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_ping_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("Ping succeed!  the model_number is %d\r\n", analysis_data);
    }
#endif

#if CALIBRATION_TEST
    //Calibrate the midpoint of the servo.
    servo_calibration(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_calibration_analysis(pack);
    if (ret == SUCCESS)
    {
        PRINTF("servo calibration successfully!\r\n");
    }
#endif

#if FACTORY_RESET_TEST
    //Reset the servo to the factory default values.
    servo_factory_reset(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_factory_reset_analysis(pack);
    if (ret == SUCCESS)
    {
        PRINTF("servo factory reset successfully!\r\n");
    }
#endif

#if PARAMETER_RESET_TEST
    //Reset the parameter settings of the servo.
    servo_parameter_reset(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_parameter_reset_analysis(pack);
    if (ret == SUCCESS)
    {
        PRINTF("servo parameter reset successfully!\r\n");
    }
#endif

#if REBOOT_TEST
    //Reboot the servo.
    servo_reboot(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    servo_reboot_analysis(pack);
#endif

#if MODIFY_ID
    //Change the servo ID of servo ID1 to 2.
    servo_modify_known_id(1, 2, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);
#endif

#if MODIFY_UNKNOWN_ID
    //Change the servo ID of the servo with an unknown ID to 1.
    servo_modify_unknown_id(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);
#endif

#if READ_TEST
    //Read the present current of servo ID1.
    servo_read_present_current(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    Sleep(80);

    if (ret == FALSE)
    {
        return FALSE;
    }
    ret = servo_read_present_current_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present current is %d\r\n", analysis_data);
    }

    //Read the present position of servo ID1.
    servo_read_present_position(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_present_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present position is %d\r\n", analysis_data);
    }

    //Read the present position and present current of servo ID1.
    servo_read_present_position_and_present_current(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_present_position_and_present_current_analysis(pack, &position, &current);
    if (ret == SUCCESS)
    {
        PRINTF("present position is : % d, present current is : % d\r\n", position, current);
    }

    //Read the present velocity of servo ID1.
    servo_read_present_velocity(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_present_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity is %d\r\n", analysis_data);
    }

    //Read the present profile position of servo ID1.
    servo_read_present_profile_position(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_read_present_profile_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present profile position is %d\r\n", analysis_data);
    }

    //Read the present profile velocity of servo ID1.
    servo_read_present_profile_velocity(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);
    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_read_present_profile_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present profile velocity is %d\r\n", analysis_data);
    }

    //Read the present PWM of servo ID1.
    servo_read_present_pwm(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_present_pwm_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present pwm analysis is %d\r\n", analysis_data);
    }

    //Read the present temperature of servo ID1.
    servo_read_present_temperature(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_present_temperature_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present temperature is %d\r\n", analysis_data);
    }

    //Read the present voltage of servo ID1.
    servo_read_present_voltage(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_present_voltage_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present voltage is %d\r\n", analysis_data);
    }

    //Read the time base target moving time of servo ID1.
    servo_read_time_base_target_moving_time(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_time_base_target_moving_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present time base target moving time is %d\r\n", analysis_data);
    }

    //Read the time base target position of servo ID1.
    servo_read_time_base_target_position(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_time_base_target_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present time base target position is %d\r\n", analysis_data);
    }

    //Read the time base target ACC of servo ID1.
    servo_read_time_base_target_acc(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_time_base_target_acc_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present time base target acc is %d\r\n", analysis_data);
    }

    //Read the time base target position and moving time of servo ID1.
    servo_read(1, 0x3C, 4, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("the time base target position and moving time pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Read the time base target ACC, position and moving time of servo ID1.
    servo_read(1, 0x3B, 5, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("the time base target acc, position and moving time pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Read the velocity base target DEC of servo ID1.
    servo_read_velocity_base_target_dec(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_velocity_base_target_dec_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity base target dec is %d\r\n", analysis_data);
    }

    //Read the velocity base target ACC of servo ID1.
    servo_read_velocity_base_target_acc(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_velocity_base_target_acc_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity base target acc is %d\r\n", analysis_data);
    }

    //Read the velocity base target velocity of servo ID1.
    servo_read_velocity_base_target_velocity(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_velocity_base_target_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity base target velocity is %d\r\n", analysis_data);
    }

    //Read the velocity base target position of servo ID1.
    servo_read_velocity_base_target_position(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_velocity_base_target_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity base target position is %d\r\n", analysis_data);
    }

    //Read the velocity base target position and velocity of servo ID1.
    servo_read(1, 0x35, 4, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("the velocity base target position and velocity pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Read the velocity base target position, velocity, ACC, and DEC of servo ID1.
    servo_read(1, 0x35, 6, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("the velocity base target position,velocity,acc and dec pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Read the target current of servo ID1.
    servo_read_target_current(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_target_current_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("target current is %d\r\n", analysis_data);
    }

    //Read the target PWM of servo ID1.
    servo_read_target_pwm(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_target_pwm_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("target pwm is %d\r\n", analysis_data);
    }

    //Read the torque switch of servo ID1.
    servo_read_torque_switch(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_torque_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("torque switch is %d\r\n", analysis_data);
    }

    //Read the LED switch of servo ID1.
    servo_read_led_switch(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_led_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("led switch is %d\r\n", analysis_data);
    }

    //Read the Flash switch of servo ID1.
    servo_read_flash_switch(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_flash_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("flash switch is %d\r\n", analysis_data);
    }

    //Read the current offset of servo ID1.
    servo_read_current_offset(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_current_offset_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("current offset is %d\r\n", analysis_data);
    }

    //Read the calibration of servo ID1.
    servo_read_calibration(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_calibration_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("calibration is %d\r\n", analysis_data);
    }

    //Read the control mode of servo ID1.
    servo_read_control_mode(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_control_mode_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("control mode is %d\r\n", analysis_data);
    }

    //Read the shutdown condition of servo ID1.
    servo_read_shutdown_condition(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_shutdown_condition_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("shutdown condition is %d\r\n", analysis_data);
    }

    //Read the LED condition of servo ID1.
    servo_read_led_condition(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_led_condition_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("led condition is %d\r\n", analysis_data);
    }

    //Read the position control D gain of servo ID1.
    servo_read_position_control_d_gain(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_position_control_d_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("position control d gain is %d\r\n", analysis_data);
    }

    //Read the position control I gain of servo ID1.
    servo_read_position_control_i_gain(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_position_control_i_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("position control i gain is %d\r\n", analysis_data);
    }

    //Read the position control P gain of servo ID1.
    servo_read_position_control_p_gain(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_position_control_p_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("position control p gain is %d\r\n", analysis_data);
    }

    //Read the position control PID gain of servo ID1.
    servo_read(1, 0x1B, 6, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("position control pid gain pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Read the PWM punch of servo ID1.
    servo_read_pwm_punch(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_pwm_punch_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("pwm punch is %d\r\n", analysis_data);
    }

    //Read the ccw deadband range of servo ID1.
    servo_read_ccw_deadband(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_ccw_deadband_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("ccw deadband is %d\r\n", analysis_data);
    }

    //Read the cw deadband range of servo ID1.
    servo_read_cw_deadband(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_cw_deadband_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("cw deadband is %d\r\n", analysis_data);
    }

    //Read the current shutdown time of servo ID1.
    servo_read_current_shutdown_time(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_current_shutdown_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("current shutdown time is %d\r\n", analysis_data);
    }

    //Read the max current limit of servo ID1.
    servo_read_max_current_limit(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_max_current_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max current limit is %d\r\n", analysis_data);
    }

    //Read the max PWM limit of servo ID1.
    servo_read_max_pwm_limit(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_max_pwm_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max pwm limit is %d\r\n", analysis_data);
    }

    //Read the max voltage limit of servo ID1.
    servo_read_max_voltage_limit(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_max_voltage_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max voltage limit is %d\r\n", analysis_data);
    }

    //Read the min voltage limit of servo ID1.
    servo_read_min_voltage_limit(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_min_voltage_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("min voltage limit is %d\r\n", analysis_data);
    }

    //Read the voltage limit of servo ID1.
    servo_read(1, 0x10, 2, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("the voltage limit pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Read the max temperature limit of servo ID1.
    servo_read_max_temperature_limit(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_max_temperature_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max temperature limit is %d\r\n", analysis_data);
    }

    //Read the max angle limit of servo ID1.
    servo_read_max_angle_limit(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_max_angle_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max angle limit is %d\r\n", analysis_data);
    }

    //Read the min angle limit of servo ID1.
    servo_read_min_angle_limit(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_min_angle_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("min angle limit is %d\r\n", analysis_data);
    }

    //Read the angle limit of servo ID1.
    servo_read(1, 0x0B, 4, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("the angle limit pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Read the return level of servo ID1.
    servo_read_return_level(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_return_level_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("return level is %d\r\n", analysis_data);
    }

    //Read the return delay time of servo ID1.
    servo_read_return_delay_time(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_return_delay_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("return delay time is %d\r\n", analysis_data);
    }

    //Read the baud rate of servo ID1.
    servo_read_baud_rate(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_baud_rate_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("baud rate is %d\r\n", analysis_data);
    }

    //Read the model information of servo ID1.
    servo_read_model_information(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_model_information_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("model information is %d\r\n", analysis_data);
    }

    //Read the firmware version of servo ID1.
    servo_read_firmware_version(1, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_read_firmware_version_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("firmware version is %d\r\n", analysis_data);
    }
#endif

#if WRITE_TEST
    //Change the return delay time of servo ID1 to 500us.
    servo_set_return_delay_time(1, 250, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_return_delay_time_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set return delay time successfully.\r\n");

    //Change the return level of servo ID1 to respond to all instruction.
    servo_set_return_level(1, 2, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_return_level_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set return level successfully.\r\n");

    //Change the baud rate of servo ID1 to 1000000.
    servo_set_baud_rate(1, 7, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_baud_rate_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set baud rate successfully.\r\n");

    //Change the min angle limit of servo ID1 to 0°.
    servo_set_min_angle_limit(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_min_angle_limit_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set min angle limit successfully.\r\n");

    //Change the max angle limit of servo ID1 to 300°.
    servo_set_max_angle_limit(1, 3000, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_max_angle_limit_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set max angle limit successfully.\r\n");

    //Change the angle limit of servo ID1 to 0°~300°.
    write_buffer[0] = 0 & 0xff;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3000 & 0xff;
    write_buffer[3] = (3000 >> 8) & 0xff;

    servo_write(1, 0x0B, 4, write_buffer, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("servo set angle limit pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Change the max temperature limit of servo ID1 to 65℃.
    servo_set_max_temperature_limit(1, 65, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_max_temperature_limit_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set max temperature limit successfully.\r\n");

    //Change the max voltage limit of servo ID1 to 8.4V.
    servo_set_max_voltage_limit(1, 84, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_max_voltage_limit_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set max voltage limit successfully.\r\n");

    //Change the min voltage limit of servo ID1 to 3.5V.
    servo_set_min_voltage_limit(1, 35, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_min_voltage_limit_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set min voltage limit successfully.\r\n");

    //Change the voltage limit of servo ID1 to 3.5~8.4V.
    write_buffer[0] = 84 & 0xff;
    write_buffer[1] = 35 & 0xff;

    servo_write(1, 0x10, 2, write_buffer, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("the voltage limit pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Change the max PWM limit of servo ID1 to 90%.
    servo_set_max_pwm_limit(1, 900, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_max_pwm_limit_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set max pwm limit successfully.\r\n");

    //Change the max current limit of servo ID1 to 900mA.
    servo_set_max_current_limit(1, 900, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_max_current_limit_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set max current limit successfully.\r\n");

    //Change the current shutdown time of servo ID1 to 500ms.
    servo_set_current_shutdown_time(1, 500, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_current_shutdown_time_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set current shutdown time successfully.\r\n");

    //Change the CW deadband of servo ID1 to 0.2°.
    servo_set_cw_deadband(1, 2, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_cw_deadband_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set cw deadband successfully.\r\n");

    //Change the CCW deadband of servo ID1 to 0.2°.
    servo_set_ccw_deadband(1, 2, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_ccw_deadband_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set ccw deadband successfully.\r\n");

    //Change the CW and CCW deadband of servo ID1 to 0.2°.
    write_buffer[0] = 2 & 0xff;
    write_buffer[1] = 2 & 0xff;

    servo_write(1, 0x18, 2, write_buffer, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("servo set cw deadband and ccw deadband pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Change the PWM punch of servo ID1 to 1%.
    servo_set_pwm_punch(1, 10, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_pwm_punch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set pwm punch successfully.\r\n");

    //Change the position control P gain of servo ID1 to 5995.
    servo_set_position_control_p_gain(1, 5995, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_position_control_p_gain_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set position control p gain successfully.\r\n");

    //Change the position control I gain of servo ID1 to 5.
    servo_set_position_control_i_gain(1, 5, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_position_control_i_gain_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set position control i gain successfully.\r\n");

    //Change the position control D gain of servo ID1 to 145.
    servo_set_position_control_d_gain(1, 145, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_position_control_d_gain_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set position control d gain successfully.\r\n");

    //Change the position control PID gain of servo ID1 to 5995, 5, and 145 respectively.
    write_buffer[0] = 5995 & 0xff;
    write_buffer[1] = (5995 >> 8) & 0xff;
    write_buffer[2] = 5 & 0xff;
    write_buffer[3] = (5 >> 8) & 0xff;
    write_buffer[4] = 145 & 0xff;
    write_buffer[5] = (145 >> 8) & 0xff;

    servo_write(1, 0x1B, 6, write_buffer, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    PRINTF("servo set position control pid gain pack is: ");
    for (uint8_t i = 0; i < ret; i++)
    {
        PRINTF("0x%x ", pack[i]);
    }
    PRINTF("\r\n");

    //Change the LED condition of servo ID1 to turn on stall error, overheating error, and angle error.
    servo_set_led_condition(1, 38, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_led_condition_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set led condition successfully.\r\n");

    //Change the shutdown condition of servo ID1 to turn on stall error, overheating error, voltage error, and angle error.
    servo_set_shutdown_conditions(1, 39, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_shutdown_conditions_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set shutdown conditions successfully.\r\n");

    //Change the Flash switch of servo ID1 to ON.
    servo_set_flash_switch(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_flash_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set flash switch successfully.\r\n");

    //Change the Flash switch of servo ID1 to OFF.
    servo_set_flash_switch(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_flash_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set flash switch successfully.\r\n");

    //Change the LED switch of servo ID1 to ON.
    servo_set_led_switch(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_led_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set led switch successfully.\r\n");

    //Change the LED switch of servo ID1 to OFF.
    servo_set_led_switch(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_led_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set led switch successfully.\r\n");

    //Change the torque switch of servo ID1 to OFF.
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

    //Change the control mode of servo ID1 to the PWM control mode.
    servo_set_control_mode(1, 3, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set control mode successfully.\r\n");

    //Change the torque switch of servo ID1 to ON.
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

    //Change the target PWM of servo ID1 to -50%.
    servo_set_target_pwm(1, -500, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(3000);

    ret = servo_set_target_pwm_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set target pwm successfully.\r\n");

    //Change the torque switch of servo ID1 to OFF.
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

    //Change the control mode of servo ID1 to the current control mode.
    servo_set_control_mode(1, 2, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set control mode successfully.\r\n");

    //Change the torque switch of servo ID1 to ON.
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

    //Change the target current of servo ID1 to -400mA.
    servo_set_target_current(1, -400, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(3000);

    ret = servo_set_target_current_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set target current successfully.\r\n");

    //Change the torque switch of servo ID1 to OFF.
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

    //Change the control mode of servo ID1 to the velocity base position control mode.
    servo_set_control_mode(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set control mode successfully.\r\n");

    //Change the torque switch of servo ID1 to ON.
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

    //Change the velocity base target velocity of servo ID1 to 360°/s.
    servo_set_velocity_base_target_velocity(1, 3600, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_velocity_base_target_velocity_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set velocity base target velocity successfully.\r\n");

    //Change the velocity base target ACC of servo ID1 to 500°/s².
    servo_set_velocity_base_target_acc(1, 10, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_velocity_base_target_acc_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set velocity base target acc successfully.\r\n");

    //Change the velocity base target DEC of servo ID1 to 50°/s².
    servo_set_velocity_base_target_dec(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_velocity_base_target_dec_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set velocity base target dec successfully.\r\n");

    //Change the velocity base target position of servo ID1 to 150°.
    servo_set_velocity_base_target_position(1, 1500, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1000);

    ret = servo_set_velocity_base_target_position_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set velocity base target position successfully.\r\n");

    //Change the torque switch of servo ID1 to OFF.
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

    //Change the control mode of servo ID1 to the time base position control mode.
    servo_set_control_mode(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set control mode successfully.\r\n");

    //Change the torque switch of servo ID1 to ON.
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set torque switch successfully.\r\n");

    //Change the time base target ACC of servo ID1 to 5.
    servo_set_time_base_target_acc(1, 5, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_time_base_target_acc_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set time base target acc successfully.\r\n");

    //Change the time base target position and moving time of servo ID1 to 300°, 500ms respectively.
    servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);

    ret = servo_set_time_base_target_position_and_moving_time_analysis(pack);
    if (ret == SUCCESS)
        PRINTF("servo set time base target position and moving time successfully.\r\n");

#endif

#if SYNC_WRITE_TEST
    servo.id_counts = 2;            //Sync write two servos
    servo.id[0] = 1;                //Set the ID of the first servo to 1
    servo.id[1] = 2;                //Set the ID of the second servo to 2

    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write torque witch successfully.\r\n");
    }
    Sleep(80);

    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write control mode successfully.\r\n");
    }
    Sleep(80);

    //Change the velocity base target ACC of servo ID1, ID2 to 500°/s² and 50°/s², respectively.  
    //Set the acceleration of servo ID1 and ID2 to 10 and 1, respectively, corresponding to the previous ID settings.
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 1;

    servo_sync_write_velocity_base_target_acc(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write velocity base target acc successfully.\r\n");
    }
    Sleep(80);

    //Change the velocity base target DEC of servo ID1, ID2 to 50°/s² and 500°/s², respectively.
    //Set the deceleration of servo ID1 and ID2 to 1 and 10, respectively, corresponding to the previous ID settings.
    servo.dec_velocity[0] = 1;
    servo.dec_velocity[1] = 10;

    servo_sync_write_velocity_base_target_dec(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write velocity base target dec successfully.\r\n");
    }
    Sleep(80);

    //Change the velocity base target velocity of the servo ID1, ID2 to 360°/s² and 720°/s², respectively.
    //Set the velocity of servo ID1 and ID2 to 3600 and 7200, respectively, corresponding to the previous ID settings.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;

    servo_sync_write_velocity_base_target_velocity(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write velocity base target velocity successfully.\r\n");
    }
    Sleep(80);

    //Change the velocity base target velocity of the servo ID1, ID2 to 150° midpoint and 0° position, respectively.
    //Set the position of servo ID1 and ID2 to 1500 and 0, respectively, corresponding to the previous ID settings.
    servo.position[0] = 1500;
    servo.position[1] = 0;

    servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write velocity base target position successfully.\r\n");
    }
    Sleep(1000);

    //Change the velocity base target velocity of servo ID1 ,ID2 to 1800 and 3600, and the position to 3000 and 3000, respectively
    servo.velocity[0] = 1800;
    servo.velocity[1] = 3600;
    servo.position[0] = 3000;
    servo.position[1] = 3000;

    servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write velocity base target position and velocity successfully.\r\n");
    }
    Sleep(1000);

    //SChange the velocity base target velocity of servo ID1 ,ID2 to 3600 and 3600, position to 0,0, acceleration to 500°/s², 500°/s², deceleration to 500°/s², 500°/s², respectively
    servo.velocity[0] = 3600;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 0;
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 10;
    servo.dec_velocity[0] = 10;
    servo.dec_velocity[1] = 10;

    servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write velocity base target acc,dec,velocity and position successfully.\r\n");
    }
    Sleep(1000);

    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write torque witch successfully.\r\n");
    }
    Sleep(80);

    //Change the control mode of servo ID1, ID2 to time base position control mode respectively.
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write control mode successfully.\r\n");
    }
    Sleep(80);

    //Change the time base target ACC of servo ID1 to 1 and 5 respectively
    servo.acc_velocity_grade[0] = 1;
    servo.acc_velocity_grade[1] = 5;

    servo_sync_write_time_base_target_acc(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write time base target acc successfully.\r\n");
    }
    Sleep(80);

    //Change the time base target position and moving time of servo ID1 to 150° midpoint and 1s, 0° and 500ms respectively.
    servo.position[0] = 1500;
    servo.position[1] = 0;
    servo.time[0] = 1000;
    servo.time[1] = 500;

    servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    else
    {
        PRINTF("Sync Write time base target position and moving time successfully.\r\n");
    }
    Sleep(1000);
#endif

    //Close the serial port
    CloseHandle(hSerial);

    return 0;
}

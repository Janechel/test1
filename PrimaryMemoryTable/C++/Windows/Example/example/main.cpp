#include <iostream>
#include "servo.h"
#include "CSerialPort.h"

#define READ_TEST 0             //��ȡ������ݲ���
#define WRITE_TEST 0            //д�������ݲ���
#define SYNC_WRITE_TEST 0       //ͬ��д����
#define PING_TEST 0             //PING�������
#define FACTORY_RESET_TEST 0    //�ָ��������ò���
#define PARAMETER_RESET_TEST 0  //�������ò���
#define REBOOT_TEST 0           //��������
#define CALIBRATION_TEST 0      //У��ƫ��ֵ����
#define MODIFY_ID 0             //�޸Ķ��ID����
#define MODIFY_UNKNOWN_ID 0     //�޸�δ֪ID�������

struct servo_sync_parameter servo;

int main()
{

    uint8_t ret;
    uint8_t order_buffer[40];                                                                         //������ɵ�ָ��
    uint8_t order_len = 0;                                                                                  //ָ���
    uint8_t pack[40];                                                                                 //��Ž��յ�Ӧ���
    uint16_t analysis_data = 0;                                                                             //Ӧ�����������������

    CSerialPort serialPort;

    //ʵ�ʴ��ڶ�ȡ�����ֽ���
    DWORD bytesRead;
    //ʵ�ʴ���д����ֽ���
    DWORD bytesWritten;

    if (serialPort.Open(12, 1000000))
    {
        PRINTF("\r\nOpen Serial successfully.");
    }
    else
    {
        // ���ڴ�ʧ��
        PRINTF("\r\nFailed to open serial port.");
        return -1;
    }

#if PING_TEST
    //��idΪ1�Ķ������pingָ��
    servo_ping(1, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_ping_analysis(pack, &analysis_data);
        if (ret == SUCCESS)
        {
            PRINTF("\r\nPing successfully! The servo model number is %d", analysis_data);
        }
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);
#endif

#if CALIBRATION_TEST
    //У��ƫ��ֵ
    servo_calibration(1, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_calibration_analysis(pack);
        if (ret == SUCCESS)
        {
            PRINTF("\r\nservo calibration successfully!");
        }
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

#endif

#if FACTORY_RESET_TEST
    //�ָ���������
    servo_factory_reset(1, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_factory_reset_analysis(pack);
        if (ret == SUCCESS)
        {
            PRINTF("\r\nservo factory reset successfully!");
        }
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);
#endif

#if PARAMETER_RESET_TEST
    //��������
    servo_parameter_reset(1, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_parameter_reset_analysis(pack);
        if (ret == SUCCESS)
        {
            PRINTF("\r\nservo parameter reset successfully!");
        }
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);
#endif

#if REBOOT_TEST
    //�������
    servo_reboot(1, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);
#endif

#if READ_TEST
    //��ȡID1����ĵ�ǰ����
    servo_read_present_current(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_present_current_analysis(pack, &analysis_data);
        PRINTF("\r\npresent current is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰλ��
    servo_read_present_position(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_present_position_analysis(pack, &analysis_data);
        PRINTF("\r\npresent position is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�ٶ�
    servo_read_present_velocity(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_present_velocity_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�Ĺ滮λ��
    servo_read_present_profile_position(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_present_profile_position_analysis(pack, &analysis_data);
        PRINTF("\r\npresent profile position is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�滮�ٶ�
    servo_read_present_profile_velocity(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_present_profile_velocity_analysis(pack, &analysis_data);
        PRINTF("\r\npresent profile velocity is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰPWM
    servo_read_present_pwm(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_present_pwm_analysis(pack, &analysis_data);
        PRINTF("\r\npresent pwm is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�¶�
    servo_read_present_temperature(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_present_temperature_analysis(pack, &analysis_data);
        PRINTF("\r\npresent temperature is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�����ѹ
    servo_read_present_voltage(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_present_voltage_analysis(pack, &analysis_data);
        PRINTF("\r\npresent voltage is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ�ʱĿ������ʱ��
    servo_read_time_base_target_moving_time(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_time_base_target_moving_time_analysis(pack, &analysis_data);
        PRINTF("\r\npresent time base target moving time is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ�ʱĿ��λ��
    servo_read_time_base_target_position(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_time_base_target_position_analysis(pack, &analysis_data);
        PRINTF("\r\npresent time base target position is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ�ʱ���ٶȵȼ�
    servo_read_time_base_target_acc(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_time_base_target_acc_analysis(pack, &analysis_data);
        PRINTF("\r\npresent time base target acc is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���Ŀ����ٶ�
    servo_read_velocity_base_target_dec(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_velocity_base_target_dec_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity base target dec is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���Ŀ����ٶ�
    servo_read_velocity_base_target_acc(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_velocity_base_target_acc_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity base target acc is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���Ŀ���ٶ�
    servo_read_velocity_base_target_velocity(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_velocity_base_target_velocity_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity base target velocity is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���Ŀ��λ��
    servo_read_velocity_base_target_position(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_velocity_base_target_position_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity base target position is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Ŀ�����
    servo_read_target_current(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_target_current_analysis(pack, &analysis_data);
        PRINTF("\r\npresent target current is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Ŀ��PWM
    servo_read_target_pwm(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_target_pwm_analysis(pack, &analysis_data);
        PRINTF("\r\npresent target pwm is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Ť�ؿ���
    servo_read_torque_switch(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_torque_switch_analysis(pack, &analysis_data);
        PRINTF("\r\npresent torque switch is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����LED����
    servo_read_led_switch(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_led_switch_analysis(pack, &analysis_data);
        PRINTF("\r\npresent led switch is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Flash����
    servo_read_flash_switch(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_flash_switch_analysis(pack, &analysis_data);
        PRINTF("\r\npresent flash switch is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ���У��ֵ
    servo_read_current_offset(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_current_offset_analysis(pack, &analysis_data);
        PRINTF("\r\npresent current offset is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�������λУ��ֵ
    servo_read_calibration(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_calibration_analysis(pack, &analysis_data);
        PRINTF("\r\npresent calibration is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���ģʽ
    servo_read_control_mode(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_control_mode_analysis(pack, &analysis_data);
        PRINTF("\r\npresent control mode is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����ж�ر�������
    servo_read_shutdown_condition(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_shutdown_condition_analysis(pack, &analysis_data);
        PRINTF("\r\npresent shutdown condition is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����LED��������
    servo_read_led_condition(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_led_condition_analysis(pack, &analysis_data);
        PRINTF("\r\npresent led condition is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����λ�ÿ���D����
    servo_read_position_control_d_gain(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_position_control_d_gain_analysis(pack, &analysis_data);
        PRINTF("\r\npresent position control d gain is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����λ�ÿ���I����
    servo_read_position_control_i_gain(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_position_control_i_gain_analysis(pack, &analysis_data);
        PRINTF("\r\npresent position control i gain is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����λ�ÿ���P����
    servo_read_position_control_p_gain(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_position_control_p_gain_analysis(pack, &analysis_data);
        PRINTF("\r\npresent position control p gain is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����PWM����ֵ
    servo_read_pwm_punch(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_pwm_punch_analysis(pack, &analysis_data);
        PRINTF("\r\npresent pwm punch is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ķ�ת����
    servo_read_ccw_deadband(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_ccw_deadband_analysis(pack, &analysis_data);
        PRINTF("\r\npresent ccw deadband is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�������ת����
    servo_read_cw_deadband(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_cw_deadband_analysis(pack, &analysis_data);
        PRINTF("\r\npresent cw deadband is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�������ʱ��
    servo_read_current_shutdown_time(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_current_shutdown_time_analysis(pack, &analysis_data);
        PRINTF("\r\npresent current shutdown time is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�������
    servo_read_max_current_limit(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_max_current_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max current limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����PWM����
    servo_read_max_pwm_limit(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_max_pwm_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max pwm limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ѹ����
    servo_read_max_voltage_limit(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_max_voltage_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max voltage limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ѹ����
    servo_read_min_voltage_limit(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_min_voltage_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent min voltage limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1������¶�����
    servo_read_max_temperature_limit(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_max_temperature_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max temperature limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1��������λ������
    servo_read_max_angle_limit(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_max_angle_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max angle limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�������Сλ������
    servo_read_min_angle_limit(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_min_angle_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent min angle limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����״̬���ؼ���
    servo_read_return_level(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_return_level_analysis(pack, &analysis_data);
        PRINTF("\r\npresent return level is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Ӧ����ʱʱ��
    servo_read_return_delay_time(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_return_delay_time_analysis(pack, &analysis_data);
        PRINTF("\r\npresent return delay time is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ĳ�����
    servo_read_baud_rate(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_baud_rate_analysis(pack, &analysis_data);
        PRINTF("\r\npresent baud rate is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĳ������
    servo_read_model_information(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_model_information_analysis(pack, &analysis_data);
        PRINTF("\r\npresent model information is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ĺ̼��汾��
    servo_read_firmware_version(1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_read_firmware_version_analysis(pack, &analysis_data);
        PRINTF("\r\npresent firmware version is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);
#endif

#if WRITE_TEST
    //����ID1�����Ӧ����ʱʱ��
    servo_set_return_delay_time(1, 250, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_return_delay_time_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����״̬���ؼ���
    servo_set_return_level(1, 2, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_return_level_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ĳ�����
    servo_set_baud_rate(1, 7, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_baud_rate_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�������Сλ������
    servo_set_min_angle_limit(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_min_angle_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1��������λ������
    servo_set_max_angle_limit(1, 3000, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_max_angle_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1������¶�����
    servo_set_max_temperature_limit(1, 100, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_max_temperature_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ĵ�ѹ����
    servo_set_max_voltage_limit(1, 90, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_max_voltage_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ĵ�ѹ����
    servo_set_min_voltage_limit(1, 33, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_min_voltage_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����PWM����
    servo_set_max_pwm_limit(1, 1000, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_max_pwm_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ĵ�������
    servo_set_max_current_limit(1, 400, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_max_current_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ĵ�������ʱ��
    servo_set_current_shutdown_time(1, 1000, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_current_shutdown_time_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�������ת����
    servo_set_cw_deadband(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_cw_deadband_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ķ�ת����
    servo_set_ccw_deadband(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_ccw_deadband_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����PWM����ֵ
    servo_set_pwm_punch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_pwm_punch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����λ�ÿ���P����
    servo_set_position_control_p_gain(1, 6000, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_position_control_p_gain_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����λ�ÿ���I����
    servo_set_position_control_i_gain(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_position_control_i_gain_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����λ�ÿ���D����
    servo_set_position_control_d_gain(1, 151, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_position_control_d_gain_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����LED��������
    servo_set_led_condition(1, 36, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_led_condition_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����ж�ر�������
    servo_set_shutdown_conditions(1, 36, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_shutdown_conditions_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Flash����
    servo_set_flash_switch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_flash_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����LED����
    servo_set_led_switch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_led_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo_set_control_mode(1, 3, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_control_mode_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ŀ��PWM
    servo_set_target_pwm(1, 1000, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_target_pwm_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(3000);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo_set_control_mode(1, 2, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_control_mode_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ŀ�����
    servo_set_target_current(1, -1000, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_target_current_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(3000);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo_set_control_mode(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_control_mode_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���Ŀ���ٶ�
    servo_set_velocity_base_target_velocity(1, 3600, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_velocity_base_target_velocity_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���Ŀ����ٶ�
    servo_set_velocity_base_target_acc(1, 150, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_velocity_base_target_acc_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���Ŀ����ٶ�
    servo_set_velocity_base_target_dec(1, 150, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_velocity_base_target_dec_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���Ŀ��λ��
    servo_set_velocity_base_target_position(1, 0, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_velocity_base_target_position_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(1000);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo_set_control_mode(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_control_mode_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ�ʱĿ����ٶȵȼ�
    servo_set_time_base_target_acc(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_time_base_target_acc_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ�ʱĿ��λ�ú�Ŀ������ʱ��
    servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = servo_set_time_base_target_position_and_moving_time_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(1000);
#endif

#if SYNC_WRITE_TEST
    servo.id_counts = 2;            //ͬ��д�������
    servo.id[0] = 1;                //��һ�����idΪ1
    servo.id[1] = 2;                //�ڶ������idΪ2

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo_set_control_mode(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_control_mode_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2�����Ť�ؿ���
    servo_set_torque_switch(2, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2����Ŀ���ģʽ
    servo_set_control_mode(2, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_control_mode_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2�����Ť�ؿ���
    servo_set_torque_switch(2, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //���ö������Ŀ���Ŀ���ٶ�

    //idΪ1��2�Ķ���ٶȷֱ�����Ϊ3600��1800��ֵ��ǰ���id���ö�Ӧ
    servo.velocity[0] = 3600;
    servo.velocity[1] = 1800;

    servo_sync_write_velocity_base_target_velocity(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    //���ö������Ŀ���Ŀ����ٶ�

    //idΪ1��2�Ķ�����ٶȷֱ�����Ϊ150��150��ֵ��ǰ���id���ö�Ӧ
    servo.acc_velocity[0] = 150;
    servo.acc_velocity[1] = 150;

    servo_sync_write_velocity_base_target_acc(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    //���ö������Ŀ���Ŀ����ٶ�

    //idΪ1��2�Ķ�����ٶȷֱ�����Ϊ150��150��ֵ��ǰ���id���ö�Ӧ
    servo.dec_velocity[0] = 150;
    servo.dec_velocity[1] = 150;

    servo_sync_write_velocity_base_target_dec(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    //���ö������Ŀ���Ŀ��λ��

    //idΪ1��2�Ķ���˶�λ�÷ֱ�����Ϊ0��0��ֵ��ǰ���id���ö�Ӧ
    servo.position[0] = 0;
    servo.position[1] = 0;

    servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);

    //���ö������Ŀ���Ŀ��λ�ú��ٶ�

    //idΪ1��2�Ķ���ٶȷֱ�����Ϊ1800��3600��λ�÷ֱ�����Ϊ3000��3000
    servo.velocity[0] = 1800;
    servo.velocity[1] = 3600;
    servo.position[0] = 3000;
    servo.position[1] = 3000;

    servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);

    //���ö������ļ��ٶȣ����ٶȣ��ٶȺ�λ��

    //idΪ1��2�Ķ���ٶȷֱ�����Ϊ3600��3600��λ�÷ֱ�����Ϊ0��0,���ٶȷֱ�����Ϊ100��100�����ٶȷֱ�����Ϊ100��100
    servo.velocity[0] = 3600;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 0;
    servo.acc_velocity[0] = 100;
    servo.acc_velocity[1] = 100;
    servo.dec_velocity[0] = 100;
    servo.dec_velocity[1] = 100;

    servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);


    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo_set_control_mode(1, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_control_mode_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2�����Ť�ؿ���
    servo_set_torque_switch(2, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2����Ŀ���ģʽ
    servo_set_control_mode(2, 0, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_control_mode_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2�����Ť�ؿ���
    servo_set_torque_switch(2, 1, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //���ö������Ŀ�ʱĿ����ٶȵȼ�

    //���ö��idΪ1��2�ļ��ٶȵȼ��ֱ�Ϊ0��0
    servo.acc_velocity_grade[0] = 0;
    servo.acc_velocity_grade[1] = 0;

    servo_sync_write_time_base_target_acc(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    //���ö������Ŀ�ʱĿ��λ�ú��˶�ʱ��

    //���ö��idΪ1��2���˶�λ��Ϊ3000��3000���˶�ʱ��Ϊ500ms��1500ms
    servo.position[0] = 3000;
    servo.position[1] = 3000;
    servo.time[0] = 500;
    servo.time[1] = 1500;


    servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);
#endif

#if MODIFY_ID
    //��idΪ1�Ķ��id�޸�Ϊ2
    servo_modify_known_id(1, 2, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nModify successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);
#endif

#if MODIFY_UNKNOWN_ID
    //�����ж��id�޸�Ϊ2
    servo_modify_unknown_id(2, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nModify successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);
#endif

    // �رմ���
    serialPort.Close();

    return 0;
}

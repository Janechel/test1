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

int main()
{

    uint8_t ret;
    uint8_t order_buffer[20] = { 0 };                                                                         //������ɵ�ָ��
    uint8_t order_len = 0;                                                                                  //ָ���
    uint8_t pack[20] = { 0 };                                                                                 //��Ž��յ�Ӧ���
    uint16_t analysis_data = 0;                                                                             //Ӧ�����������������
    uint16_t sync_write_velocity_base_target_position[5] = { 1, 0, 2, 0 };                    //ͬ��д����������Ŀ��λ��
    uint16_t sync_write_velocity_base_target_velocity[5] = { 1, 3600, 2, 3600 };              //ͬ��д����������Ŀ���ٶ�
    uint16_t sync_write_velocity_base_target_acc[5] = { 1, 150, 2, 150 };                     //ͬ��д����������Ŀ����ٶ�
    uint16_t sync_write_velocity_base_target_dec[5] = { 1, 150, 2, 150 };                     //ͬ��д����������Ŀ����ٶ�
    uint16_t sync_write_time_base_target_acc[5] = { 1, 0, 2, 0 };                             //ͬ��д��������ʱĿ����ٶ�
    uint16_t sync_write_time_base_target_position_and_moving_time[10] = { 1, 3000, 500, 2, 3000, 500 };           //ͬ��д��������ʱĿ���˶�λ�ú��˶�ʱ��
    uint16_t sync_write_velocity_base_target_position_and_velocity[10] = { 1, 1500, 1800, 2, 1500, 900 };           //ͬ��д����������Ŀ��λ�ú��ٶ�


    //���������ʹ��ڵĿ�����
    Servo servo;
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
    servo.servo_ping(1, order_buffer, &order_len);
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
        ret = servo.servo_ping_analysis(pack, &analysis_data);
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
    servo.servo_calibration(1, order_buffer, &order_len);
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
        ret = servo.servo_calibration_analysis(pack);
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
    servo.servo_factory_reset(1, order_buffer, &order_len);
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
        ret = servo.servo_factory_reset_analysis(pack);
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
    servo.servo_parameter_reset(1, order_buffer, &order_len);
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
        ret = servo.servo_parameter_reset_analysis(pack);
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
    servo.servo_reboot(1, order_buffer, &order_len);
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
    servo.servo_read_present_current(1, order_buffer, &order_len);

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
        servo.servo_read_present_current_analysis(pack, &analysis_data);
        PRINTF("\r\npresent current is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰλ��
    servo.servo_read_present_position(1, order_buffer, &order_len);

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
        servo.servo_read_present_position_analysis(pack, &analysis_data);
        PRINTF("\r\npresent position is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�ٶ�
    servo.servo_read_present_velocity(1, order_buffer, &order_len);

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
        servo.servo_read_present_velocity_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�Ĺ滮λ��
    servo.servo_read_present_profile_position(1, order_buffer, &order_len);

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
        servo.servo_read_present_profile_position_analysis(pack, &analysis_data);
        PRINTF("\r\npresent profile position is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�滮�ٶ�
    servo.servo_read_present_profile_velocity(1, order_buffer, &order_len);

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
        servo.servo_read_present_profile_velocity_analysis(pack, &analysis_data);
        PRINTF("\r\npresent profile velocity is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰPWM
    servo.servo_read_present_pwm(1, order_buffer, &order_len);

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
        servo.servo_read_present_pwm_analysis(pack, &analysis_data);
        PRINTF("\r\npresent pwm is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�¶�
    servo.servo_read_present_temperature(1, order_buffer, &order_len);

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
        servo.servo_read_present_temperature_analysis(pack, &analysis_data);
        PRINTF("\r\npresent temperature is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ǰ�����ѹ
    servo.servo_read_present_voltage(1, order_buffer, &order_len);

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
        servo.servo_read_present_voltage_analysis(pack, &analysis_data);
        PRINTF("\r\npresent voltage is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ�ʱĿ������ʱ��
    servo.servo_read_time_base_target_moving_time(1, order_buffer, &order_len);

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
        servo.servo_read_time_base_target_moving_time_analysis(pack, &analysis_data);
        PRINTF("\r\npresent time base target moving time is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ�ʱĿ��λ��
    servo.servo_read_time_base_target_position(1, order_buffer, &order_len);

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
        servo.servo_read_time_base_target_position_analysis(pack, &analysis_data);
        PRINTF("\r\npresent time base target position is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ�ʱ���ٶȵȼ�
    servo.servo_read_time_base_target_acc(1, order_buffer, &order_len);

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
        servo.servo_read_time_base_target_acc_analysis(pack, &analysis_data);
        PRINTF("\r\npresent time base target acc is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���Ŀ����ٶ�
    servo.servo_read_velocity_base_target_dec(1, order_buffer, &order_len);

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
        servo.servo_read_velocity_base_target_dec_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity base target dec is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���Ŀ����ٶ�
    servo.servo_read_velocity_base_target_acc(1, order_buffer, &order_len);

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
        servo.servo_read_velocity_base_target_acc_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity base target acc is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���Ŀ���ٶ�
    servo.servo_read_velocity_base_target_velocity(1, order_buffer, &order_len);

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
        servo.servo_read_velocity_base_target_velocity_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity base target velocity is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���Ŀ��λ��
    servo.servo_read_velocity_base_target_position(1, order_buffer, &order_len);

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
        servo.servo_read_velocity_base_target_position_analysis(pack, &analysis_data);
        PRINTF("\r\npresent velocity base target position is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Ŀ�����
    servo.servo_read_target_current(1, order_buffer, &order_len);

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
        servo.servo_read_target_current_analysis(pack, &analysis_data);
        PRINTF("\r\npresent target current is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Ŀ��PWM
    servo.servo_read_target_pwm(1, order_buffer, &order_len);

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
        servo.servo_read_target_pwm_analysis(pack, &analysis_data);
        PRINTF("\r\npresent target pwm is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Ť�ؿ���
    servo.servo_read_torque_switch(1, order_buffer, &order_len);

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
        servo.servo_read_torque_switch_analysis(pack, &analysis_data);
        PRINTF("\r\npresent torque switch is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����LED����
    servo.servo_read_led_switch(1, order_buffer, &order_len);

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
        servo.servo_read_led_switch_analysis(pack, &analysis_data);
        PRINTF("\r\npresent led switch is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Flash����
    servo.servo_read_flash_switch(1, order_buffer, &order_len);

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
        servo.servo_read_flash_switch_analysis(pack, &analysis_data);
        PRINTF("\r\npresent flash switch is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ���У��ֵ
    servo.servo_read_current_offset(1, order_buffer, &order_len);

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
        servo.servo_read_current_offset_analysis(pack, &analysis_data);
        PRINTF("\r\npresent current offset is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�������λУ��ֵ
    servo.servo_read_calibration(1, order_buffer, &order_len);

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
        servo.servo_read_calibration_analysis(pack, &analysis_data);
        PRINTF("\r\npresent calibration is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ŀ���ģʽ
    servo.servo_read_control_mode(1, order_buffer, &order_len);

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
        servo.servo_read_control_mode_analysis(pack, &analysis_data);
        PRINTF("\r\npresent control mode is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����ж�ر�������
    servo.servo_read_shutdown_condition(1, order_buffer, &order_len);

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
        servo.servo_read_shutdown_condition_analysis(pack, &analysis_data);
        PRINTF("\r\npresent shutdown condition is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����LED��������
    servo.servo_read_led_condition(1, order_buffer, &order_len);

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
        servo.servo_read_led_condition_analysis(pack, &analysis_data);
        PRINTF("\r\npresent led condition is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����λ�ÿ���D����
    servo.servo_read_position_control_d_gain(1, order_buffer, &order_len);

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
        servo.servo_read_position_control_d_gain_analysis(pack, &analysis_data);
        PRINTF("\r\npresent position control d gain is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����λ�ÿ���I����
    servo.servo_read_position_control_i_gain(1, order_buffer, &order_len);

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
        servo.servo_read_position_control_i_gain_analysis(pack, &analysis_data);
        PRINTF("\r\npresent position control i gain is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����λ�ÿ���P����
    servo.servo_read_position_control_p_gain(1, order_buffer, &order_len);

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
        servo.servo_read_position_control_p_gain_analysis(pack, &analysis_data);
        PRINTF("\r\npresent position control p gain is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����PWM����ֵ
    servo.servo_read_pwm_punch(1, order_buffer, &order_len);

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
        servo.servo_read_pwm_punch_analysis(pack, &analysis_data);
        PRINTF("\r\npresent pwm punch is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ķ�ת����
    servo.servo_read_ccw_deadband(1, order_buffer, &order_len);

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
        servo.servo_read_ccw_deadband_analysis(pack, &analysis_data);
        PRINTF("\r\npresent ccw deadband is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�������ת����
    servo.servo_read_cw_deadband(1, order_buffer, &order_len);

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
        servo.servo_read_cw_deadband_analysis(pack, &analysis_data);
        PRINTF("\r\npresent cw deadband is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�������ʱ��
    servo.servo_read_current_shutdown_time(1, order_buffer, &order_len);

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
        servo.servo_read_current_shutdown_time_analysis(pack, &analysis_data);
        PRINTF("\r\npresent current shutdown time is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�������
    servo.servo_read_max_current_limit(1, order_buffer, &order_len);

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
        servo.servo_read_max_current_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max current limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����PWM����
    servo.servo_read_max_pwm_limit(1, order_buffer, &order_len);

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
        servo.servo_read_max_pwm_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max pwm limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ѹ����
    servo.servo_read_max_voltage_limit(1, order_buffer, &order_len);

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
        servo.servo_read_max_voltage_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max voltage limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĵ�ѹ����
    servo.servo_read_min_voltage_limit(1, order_buffer, &order_len);

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
        servo.servo_read_min_voltage_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent min voltage limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1������¶�����
    servo.servo_read_max_temperature_limit(1, order_buffer, &order_len);

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
        servo.servo_read_max_temperature_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max temperature limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1��������λ������
    servo.servo_read_max_angle_limit(1, order_buffer, &order_len);

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
        servo.servo_read_max_angle_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent max angle limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�������Сλ������
    servo.servo_read_min_angle_limit(1, order_buffer, &order_len);

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
        servo.servo_read_min_angle_limit_analysis(pack, &analysis_data);
        PRINTF("\r\npresent min angle limit is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����״̬���ؼ���
    servo.servo_read_return_level(1, order_buffer, &order_len);

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
        servo.servo_read_return_level_analysis(pack, &analysis_data);
        PRINTF("\r\npresent return level is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1�����Ӧ����ʱʱ��
    servo.servo_read_return_delay_time(1, order_buffer, &order_len);

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
        servo.servo_read_return_delay_time_analysis(pack, &analysis_data);
        PRINTF("\r\npresent return delay time is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ĳ�����
    servo.servo_read_baud_rate(1, order_buffer, &order_len);

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
        servo.servo_read_baud_rate_analysis(pack, &analysis_data);
        PRINTF("\r\npresent baud rate is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����ĳ������
    servo.servo_read_model_information(1, order_buffer, &order_len);

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
        servo.servo_read_model_information_analysis(pack, &analysis_data);
        PRINTF("\r\npresent model information is: %d", analysis_data);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //��ȡID1����Ĺ̼��汾��
    servo.servo_read_firmware_version(1, order_buffer, &order_len);

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
        servo.servo_read_firmware_version_analysis(pack, &analysis_data);
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
    servo.servo_set_return_delay_time(1, 250, order_buffer, &order_len);

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
        ret = servo.servo_set_return_delay_time_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����״̬���ؼ���
    servo.servo_set_return_level(1, 2, order_buffer, &order_len);

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
        ret = servo.servo_set_return_level_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ĳ�����
    servo.servo_set_baud_rate(1, 7, order_buffer, &order_len);

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
        ret = servo.servo_set_baud_rate_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�������Сλ������
    servo.servo_set_min_angle_limit(1, 0, order_buffer, &order_len);

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
        ret = servo.servo_set_min_angle_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1��������λ������
    servo.servo_set_max_angle_limit(1, 3000, order_buffer, &order_len);

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
        ret = servo.servo_set_max_angle_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1������¶�����
    servo.servo_set_max_temperature_limit(1, 100, order_buffer, &order_len);

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
        ret = servo.servo_set_max_temperature_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ĵ�ѹ����
    servo.servo_set_max_voltage_limit(1, 90, order_buffer, &order_len);

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
        ret = servo.servo_set_max_voltage_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ĵ�ѹ����
    servo.servo_set_min_voltage_limit(1, 33, order_buffer, &order_len);

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
        ret = servo.servo_set_min_voltage_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����PWM����
    servo.servo_set_max_pwm_limit(1, 1000, order_buffer, &order_len);

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
        ret = servo.servo_set_max_pwm_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ĵ�������
    servo.servo_set_max_current_limit(1, 400, order_buffer, &order_len);

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
        ret = servo.servo_set_max_current_limit_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ĵ�������ʱ��
    servo.servo_set_current_shutdown_time(1, 1000, order_buffer, &order_len);

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
        ret = servo.servo_set_current_shutdown_time_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�������ת����
    servo.servo_set_cw_deadband(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_cw_deadband_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����ķ�ת����
    servo.servo_set_ccw_deadband(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_ccw_deadband_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����PWM����ֵ
    servo.servo_set_pwm_punch(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_pwm_punch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����λ�ÿ���P����
    servo.servo_set_position_control_p_gain(1, 6000, order_buffer, &order_len);

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
        ret = servo.servo_set_position_control_P_gain_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����λ�ÿ���I����
    servo.servo_set_position_control_i_gain(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_position_control_I_gain_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����λ�ÿ���D����
    servo.servo_set_position_control_d_gain(1, 151, order_buffer, &order_len);

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
        ret = servo.servo_set_position_control_D_gain_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����LED��������
    servo.servo_set_led_condition(1, 36, order_buffer, &order_len);

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
        ret = servo.servo_set_led_condition_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����ж�ر�������
    servo.servo_set_shutdown_conditions(1, 36, order_buffer, &order_len);

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
        ret = servo.servo_set_shutdown_conditions_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Flash����
    servo.servo_set_flash_switch(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_flash_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����LED����
    servo.servo_set_led_switch(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_led_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

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
        ret = servo.servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo.servo_set_control_mode(1, 3, order_buffer, &order_len);

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
        ret = servo.servo_set_control_mode_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ŀ��PWM
    servo.servo_set_target_pwm(1, 1000, order_buffer, &order_len);

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
        ret = servo.servo_set_target_pwm_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(3000);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

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
        ret = servo.servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo.servo_set_control_mode(1, 2, order_buffer, &order_len);

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
        ret = servo.servo_set_control_mode_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ŀ�����
    servo.servo_set_target_current(1, -1000, order_buffer, &order_len);

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
        ret = servo.servo_set_target_current_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(3000);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

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
        ret = servo.servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo.servo_set_control_mode(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_control_mode_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���Ŀ���ٶ�
    servo.servo_set_velocity_base_target_velocity(1, 3600, order_buffer, &order_len);

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
        ret = servo.servo_set_velocity_base_target_velocity_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���Ŀ����ٶ�
    servo.servo_set_velocity_base_target_acc(1, 150, order_buffer, &order_len);

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
        ret = servo.servo_set_velocity_base_target_acc_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���Ŀ����ٶ�
    servo.servo_set_velocity_base_target_dec(1, 150, order_buffer, &order_len);

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
        ret = servo.servo_set_velocity_base_target_dec_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���Ŀ��λ��
    servo.servo_set_velocity_base_target_position(1, 0, order_buffer, &order_len);
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
        ret = servo.servo_set_velocity_base_target_position_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(1000);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

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
        ret = servo.servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo.servo_set_control_mode(1, 0, order_buffer, &order_len);

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
        ret = servo.servo_set_control_mode_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

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
        ret = servo.servo_set_torque_switch_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ�ʱĿ����ٶȵȼ�
    servo.servo_set_time_base_target_acc(1, 0, order_buffer, &order_len);

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
        ret = servo.servo_set_time_base_target_acc_analysis(pack);
        if (ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ�ʱĿ��λ�ú�Ŀ������ʱ��
    servo.servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer, &order_len);
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
        ret = servo.servo_set_time_base_target_position_and_moving_time_analysis(pack);
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
    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

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
        servo.servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo.servo_set_control_mode(1, 1, order_buffer, &order_len);

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
        servo.servo_set_control_mode_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

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
        servo.servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2�����Ť�ؿ���
    servo.servo_set_torque_switch(2, 0, order_buffer, &order_len);

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
        servo.servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2����Ŀ���ģʽ
    servo.servo_set_control_mode(2, 1, order_buffer, &order_len);

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
        servo.servo_set_control_mode_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2�����Ť�ؿ���
    servo.servo_set_torque_switch(2, 1, order_buffer, &order_len);

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
        servo.servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //���ö������Ŀ���Ŀ���ٶ�
    servo.servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer, &order_len);
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
    servo.servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer, &order_len);
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
    servo.servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer, &order_len);
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
    servo.servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer, &order_len);
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
    servo.servo_sync_write_velocity_base_target_position_and_velocity(2, sync_write_velocity_base_target_position_and_velocity, order_buffer, &order_len);
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
    servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

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
        servo.servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1����Ŀ���ģʽ
    servo.servo_set_control_mode(1, 0, order_buffer, &order_len);

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
        servo.servo_set_control_mode_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID1�����Ť�ؿ���
    servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

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
        servo.servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2�����Ť�ؿ���
    servo.servo_set_torque_switch(2, 0, order_buffer, &order_len);

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
        servo.servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2����Ŀ���ģʽ
    servo.servo_set_control_mode(2, 0, order_buffer, &order_len);

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
        servo.servo_set_control_mode_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //����ID2�����Ť�ؿ���
    servo.servo_set_torque_switch(2, 1, order_buffer, &order_len);

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
        servo.servo_set_torque_switch_analysis(pack);
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //���ö������Ŀ�ʱĿ����ٶȵȼ�
    servo.servo_sync_write_time_base_target_acc(2, sync_write_time_base_target_acc, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    servo.servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer, &order_len);
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
    servo.servo_modify_known_id(1, 2, order_buffer, &order_len);
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
    servo.servo_modify_unknown_id(2, order_buffer, &order_len);
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

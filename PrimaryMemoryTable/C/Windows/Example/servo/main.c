#include "servo.h"
#include <windows.h>
#include <stdio.h>

#define READ_TEST 0                 // ��ȡ������ݲ���
#define WRITE_TEST 0                // д�������ݲ���
#define SYNC_WRITE_TEST 0           // ͬ��д����
#define PING_TEST 0                 // PING�������
#define FACTORY_RESET_TEST 0        // �ָ��������ò���
#define PARAMETER_RESET_TEST 0      // �������ò���
#define REBOOT_TEST 0               // ��������
#define CALIBRATION_TEST 0          // У��ƫ��ֵ����
#define MODIFY_ID 0                 // �޸Ķ��ID����
#define MODIFY_UNKNOWN_ID 0         // �޸�δ֪ID���ID����


//���ڳ�ʼ��
uint8_t uart_init(HANDLE hSerial)
{
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        PRINTF("Failed to open serial port\n");
        return FALSE;
    }

    // ���ô��ڲ���
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        PRINTF("Failed to get serial port parameters\n");
        CloseHandle(hSerial);
        return FALSE;
    }

    //���ô���Э�����
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

//���ڷ���
uint8_t order_send(HANDLE hSerial, uint8_t* order_buffer, uint8_t order_len)
{
    uint8_t ret;                    //״̬��־λ
    DWORD bytesWritten;             //ʵ��д�����ݳ���

    //д�봮������
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

//���ڽ�������
uint8_t order_receive(HANDLE hSerial, uint8_t pack[])
{
    uint8_t ret;                //״̬��־λ
    DWORD bytesRead;            //ʵ�ʶ�ȡ���ݳ���
    DWORD errors;               //����error��־λ
    DWORD read_len;             //��ȡ����
    COMSTAT comstat;            //��������ͨ�ŵ�״̬��Ϣ

    if (!ClearCommError(hSerial, &errors, &comstat)) {
        return FALSE;
    }

    //��ȡ���ջ������п��õ��ֽ���
    read_len = comstat.cbInQue;

    //��ȡ���ڻ���������
    ret = ReadFile(hSerial, pack, read_len, &bytesRead, NULL);

    if (ret != 0)
    {
        if (bytesRead > 0)
        {
            return TRUE;
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


    uint8_t ret;

    // �򿪴���
    HANDLE hSerial = CreateFile("COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    //���ڳ�ʼ��
    ret = uart_init(hSerial);
    if (ret == FALSE)
    {
        return FALSE;
    }

#if PING_TEST
    //��IDΪ1�Ķ������PINGָ��
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
    servo_ping_analysis(pack, &analysis_data);
    PRINTF("Ping succeed!  the model_number is %d\r\n", analysis_data);
#endif

#if CALIBRATION_TEST
    //У��ƫ��ֵ
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
    servo_calibration_analysis(pack);
#endif

#if FACTORY_RESET_TEST
    //�ָ���������
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
    servo_factory_reset_analysis(pack);
#endif

#if PARAMETER_RESET_TEST
    //��������
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
    servo_parameter_reset_analysis(pack);
#endif

#if REBOOT_TEST
    //�������
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
    // �޸�ID1���IDΪ2
    servo_modify_known_id(1, 2, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);
#endif

#if MODIFY_UNKNOWN_ID
    // ��δ֪ID���ID�޸�Ϊ2
    servo_modify_unknown_id(2, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);
#endif

#if READ_TEST
    //��ȡID1����ĵ�ǰ����
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
    servo_read_present_current_analysis(pack, &analysis_data);
    PRINTF("present current is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ǰλ��
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
    servo_read_present_position_analysis(pack, &analysis_data);
    PRINTF("present position is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ǰ�ٶ�
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
    servo_read_present_velocity_analysis(pack, &analysis_data);
    PRINTF("present velocity is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ǰ�Ĺ滮λ��
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

    servo_read_present_profile_position_analysis(pack, &analysis_data);
    PRINTF("present profile position is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ǰ�滮�ٶ�
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

    servo_read_present_profile_velocity_analysis(pack, &analysis_data);
    PRINTF("present profile velocity is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ǰPWM
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
    servo_read_present_pwm_analysis(pack, &analysis_data);
    PRINTF("present pwm analysis is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ǰ�¶�
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
    servo_read_present_temperature_analysis(pack, &analysis_data);
    PRINTF("present temperature is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ǰ�����ѹ
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
    servo_read_present_voltage_analysis(pack, &analysis_data);
    PRINTF("present voltage is %d\r\n", analysis_data);

    //��ȡID1����Ŀ�ʱĿ������ʱ��
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
    servo_read_time_base_target_moving_time_analysis(pack, &analysis_data);
    PRINTF("present time base target moving time is %d\r\n", analysis_data);

    //��ȡID1����Ŀ�ʱĿ��λ��
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
    servo_read_time_base_target_position_analysis(pack, &analysis_data);
    PRINTF("present time base target position is %d\r\n", analysis_data);

    //��ȡID1����Ŀ�ʱ���ٶȵȼ�
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
    servo_read_time_base_target_acc_analysis(pack, &analysis_data);
    PRINTF("present time base target acc is %d\r\n", analysis_data);

    //��ȡID1����Ŀ���Ŀ����ٶ�
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
    servo_read_velocity_base_target_dec_analysis(pack, &analysis_data);
    PRINTF("present velocity base target dec is %d\r\n", analysis_data);

    //��ȡID1����Ŀ���Ŀ����ٶ�
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
    servo_read_velocity_base_target_acc_analysis(pack, &analysis_data);
    PRINTF("present velocity base target acc is %d\r\n", analysis_data);

    //��ȡID1����Ŀ���Ŀ���ٶ�
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
    servo_read_velocity_base_target_velocity_analysis(pack, &analysis_data);
    PRINTF("present velocity base target velocity is %d\r\n", analysis_data);

    //��ȡID1����Ŀ���Ŀ��λ��
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
    servo_read_velocity_base_target_position_analysis(pack, &analysis_data);
    PRINTF("present velocity base target position is %d\r\n", analysis_data);

    //��ȡID1�����Ŀ�����
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
    servo_read_target_current_analysis(pack, &analysis_data);
    PRINTF("target current is %d\r\n", analysis_data);

    //��ȡID1�����Ŀ��PWM
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
    servo_read_target_pwm_analysis(pack, &analysis_data);
    PRINTF("target pwm is %d\r\n", analysis_data);

    //��ȡID1�����Ť�ؿ���
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
    servo_read_torque_switch_analysis(pack, &analysis_data);
    PRINTF("torque switch is %d\r\n", analysis_data);

    //��ȡID1�����LED����
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
    servo_read_led_switch_analysis(pack, &analysis_data);
    PRINTF("led switch is %d\r\n", analysis_data);

    //��ȡID1�����Flash����
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
    servo_read_flash_switch_analysis(pack, &analysis_data);
    PRINTF("flash switch is %d\r\n", analysis_data);

    //��ȡID1����ĵ���У��ֵ
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
    servo_read_current_offset_analysis(pack, &analysis_data);
    PRINTF("current offset is %d\r\n", analysis_data);

    //��ȡID1�������λУ��ֵ
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
    servo_read_calibration_analysis(pack, &analysis_data);
    PRINTF("calibration is %d\r\n", analysis_data);

    //��ȡID1����Ŀ���ģʽ
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
    servo_read_control_mode_analysis(pack, &analysis_data);
    PRINTF("control mode is %d\r\n", analysis_data);

    //��ȡID1�����ж�ر�������
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
    servo_read_shutdown_condition_analysis(pack, &analysis_data);
    PRINTF("shutdown condition is %d\r\n", analysis_data);

    //��ȡID1�����LED��������
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
    servo_read_led_condition_analysis(pack, &analysis_data);
    PRINTF("led condition is %d\r\n", analysis_data);

    //��ȡID1�����λ�ÿ���D����
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
    servo_read_position_control_d_gain_analysis(pack, &analysis_data);
    PRINTF("position control d gain is %d\r\n", analysis_data);

    //��ȡID1�����λ�ÿ���I����
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
    servo_read_position_control_i_gain_analysis(pack, &analysis_data);
    PRINTF("position control i gain is %d\r\n", analysis_data);

    //��ȡID1�����λ�ÿ���P����
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
    servo_read_position_control_p_gain_analysis(pack, &analysis_data);
    PRINTF("position control p gain is %d\r\n", analysis_data);

    //��ȡID1�����PWM����ֵ
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
    servo_read_pwm_punch_analysis(pack, &analysis_data);
    PRINTF("pwm punch is %d\r\n", analysis_data);

    //��ȡID1����ķ�ת����
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
    servo_read_ccw_deadband_analysis(pack, &analysis_data);
    PRINTF("ccw deadband is %d\r\n", analysis_data);

    //��ȡID1�������ת����
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
    servo_read_cw_deadband_analysis(pack, &analysis_data);
    PRINTF("cw deadband is %d\r\n", analysis_data);

    //��ȡID1����ĵ�������ʱ��
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
    servo_read_current_shutdown_time_analysis(pack, &analysis_data);
    PRINTF("current shutdown time is %d\r\n", analysis_data);

    //��ȡID1����ĵ�������
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
    servo_read_max_current_limit_analysis(pack, &analysis_data);
    PRINTF("max current limit is %d\r\n", analysis_data);

    //��ȡID1�����PWM����
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
    servo_read_max_pwm_limit_analysis(pack, &analysis_data);
    PRINTF("max pwm limit is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ѹ����
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
    servo_read_max_voltage_limit_analysis(pack, &analysis_data);
    PRINTF("max voltage limit is %d\r\n", analysis_data);

    //��ȡID1����ĵ�ѹ����
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
    servo_read_min_voltage_limit_analysis(pack, &analysis_data);
    PRINTF("min voltage limit is %d\r\n", analysis_data);

    //��ȡID1������¶�����
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
    servo_read_max_temperature_limit_analysis(pack, &analysis_data);
    PRINTF("max temperature limit is %d\r\n", analysis_data);

    //��ȡID1��������λ������
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
    servo_read_max_angle_limit_analysis(pack, &analysis_data);
    PRINTF("max angle limit is %d\r\n", analysis_data);

    //��ȡID1�������Сλ������
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
    servo_read_min_angle_limit_analysis(pack, &analysis_data);
    PRINTF("min angle limit is %d\r\n", analysis_data);

    //��ȡID1�����״̬���ؼ���
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
    servo_read_return_level_analysis(pack, &analysis_data);
    PRINTF("return level is %d\r\n", analysis_data);

    //��ȡID1�����Ӧ����ʱʱ��
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
    servo_read_return_delay_time_analysis(pack, &analysis_data);
    PRINTF("return delay time is %d\r\n", analysis_data);

    //��ȡID1����Ĳ�����
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
    servo_read_baud_rate_analysis(pack, &analysis_data);
    PRINTF("baud rate is %d\r\n", analysis_data);

    //��ȡID1����ĳ������
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
    servo_read_model_information_analysis(pack, &analysis_data);
    PRINTF("model information is %d\r\n", analysis_data);

    //��ȡID1����Ĺ̼��汾��
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
    servo_read_firmware_version_analysis(pack, &analysis_data);
    PRINTF("firmware version is %d\r\n", analysis_data);
#endif

#if WRITE_TEST
    //����ID1�����Flash����
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

    servo_set_flash_switch_analysis(pack);

    //����ID1�����Ӧ����ʱʱ��
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

    servo_set_return_delay_time_analysis(pack);

    //����ID1�����״̬���ؼ���
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

    servo_set_return_level_analysis(pack);

    //����ID1����Ĳ�����
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

    servo_set_baud_rate_analysis(pack);

    //����ID1�������Сλ������
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

    servo_set_min_angle_limit_analysis(pack);

    //����ID1��������λ������
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

    servo_set_max_angle_limit_analysis(pack);

    //����ID1������¶�����
    servo_set_max_temperature_limit(1, 100, order_buffer, &order_len);

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

    servo_set_max_temperature_limit_analysis(pack);

    //����ID1����ĵ�ѹ����
    servo_set_max_voltage_limit(1, 90, order_buffer, &order_len);

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

    servo_set_max_voltage_limit_analysis(pack);

    //����ID1����ĵ�ѹ����
    servo_set_min_voltage_limit(1, 33, order_buffer, &order_len);

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

    servo_set_min_voltage_limit_analysis(pack);

    //����ID1�����PWM����
    servo_set_max_pwm_limit(1, 1000, order_buffer, &order_len);

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

    servo_set_max_pwm_limit_analysis(pack);

    //����ID1����ĵ�������
    servo_set_max_current_limit(1, 400, order_buffer, &order_len);

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

    servo_set_max_current_limit_analysis(pack);

    //����ID1����ĵ�������ʱ��
    servo_set_current_shutdown_time(1, 1000, order_buffer, &order_len);

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

    servo_set_current_shutdown_time_analysis(pack);

    //����ID1�������ת����
    servo_set_cw_deadband(1, 1, order_buffer, &order_len);

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

    servo_set_cw_deadband_analysis(pack);

    //����ID1����ķ�ת����
    servo_set_ccw_deadband(1, 1, order_buffer, &order_len);

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

    servo_set_ccw_deadband_analysis(pack);

    //����ID1�����PWM����ֵ
    servo_set_pwm_punch(1, 1, order_buffer, &order_len);

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

    servo_set_pwm_punch_analysis(pack);

    //����ID1�����λ�ÿ���P����
    servo_set_position_control_p_gain(1, 6000, order_buffer, &order_len);

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

    servo_set_position_control_p_gain_analysis(pack);

    //����ID1�����λ�ÿ���I����
    servo_set_position_control_i_gain(1, 1, order_buffer, &order_len);

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

    servo_set_position_control_i_gain_analysis(pack);

    //����ID1�����λ�ÿ���D����
    servo_set_position_control_d_gain(1, 151, order_buffer, &order_len);

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

    servo_set_position_control_d_gain_analysis(pack);

    //����ID1�����LED��������
    servo_set_led_condition(1, 36, order_buffer, &order_len);

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

    servo_set_led_condition_analysis(pack);

    //����ID1�����ж�ر�������
    servo_set_shutdown_conditions(1, 36, order_buffer, &order_len);

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

    servo_set_shutdown_conditions_analysis(pack);

    //����ID1�����LED����
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

    servo_set_led_switch_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1����Ŀ���ģʽ
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
    servo_set_control_mode_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1�����Ŀ��PWM
    servo_set_target_pwm(1, -1000, order_buffer, &order_len);

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

    servo_set_target_pwm_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1����Ŀ���ģʽ
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
    servo_set_control_mode_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1�����Ŀ�����
    servo_set_target_current(1, 1000, order_buffer, &order_len);

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

    servo_set_target_current_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1����Ŀ���ģʽ
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
    servo_set_control_mode_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1����Ŀ���Ŀ���ٶ�
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

    servo_set_velocity_base_target_velocity_analysis(pack);

    //����ID1����Ŀ���Ŀ����ٶ�
    servo_set_velocity_base_target_acc(1, 150, order_buffer, &order_len);

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

    servo_set_velocity_base_target_acc_analysis(pack);

    //����ID1����Ŀ���Ŀ����ٶ�
    servo_set_velocity_base_target_dec(1, 150, order_buffer, &order_len);

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

    servo_set_velocity_base_target_dec_analysis(pack);

    //����ID1����Ŀ���Ŀ��λ��
    servo_set_velocity_base_target_position(1, 0, order_buffer, &order_len);

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

    servo_set_velocity_base_target_position_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1����Ŀ���ģʽ
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
    servo_set_control_mode_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1����Ŀ�ʱĿ����ٶȵȼ�
    servo_set_time_base_target_acc(1, 0, order_buffer, &order_len);
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

    servo_set_time_base_target_acc_analysis(pack);

    //����ID1����Ŀ�ʱĿ��λ�ú�Ŀ������ʱ��
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

    servo_set_time_base_target_position_and_moving_time_analysis(pack);

#endif

#if SYNC_WRITE_TEST
    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1����Ŀ���ģʽ
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
    servo_set_control_mode_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID2�����Ť�ؿ���
    servo_set_torque_switch(2, 0, order_buffer, &order_len);
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
    servo_set_torque_switch_analysis(pack);

    //����ID2����Ŀ���ģʽ
    servo_set_control_mode(2, 1, order_buffer, &order_len);
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
    servo_set_control_mode_analysis(pack);

    //����ID2�����Ť�ؿ���
    servo_set_torque_switch(2, 1, order_buffer, &order_len);
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
    servo_set_torque_switch_analysis(pack);

    //���ö������Ŀ���Ŀ����ٶ�
    servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer, &order_len);
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

    //���ö������Ŀ���Ŀ����ٶ�
    servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer, &order_len);
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

    //���ö������Ŀ���Ŀ���ٶ�
    servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer, &order_len);
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

    //���ö������Ŀ���Ŀ��λ��
    servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer, &order_len);
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

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID1����Ŀ���ģʽ
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
    servo_set_control_mode_analysis(pack);

    //����ID1�����Ť�ؿ���
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
    servo_set_torque_switch_analysis(pack);

    //����ID2�����Ť�ؿ���
    servo_set_torque_switch(2, 0, order_buffer, &order_len);
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
    servo_set_torque_switch_analysis(pack);

    //����ID2����Ŀ���ģʽ
    servo_set_control_mode(2, 0, order_buffer, &order_len);
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
    servo_set_control_mode_analysis(pack);

    //����ID2�����Ť�ؿ���
    servo_set_torque_switch(2, 1, order_buffer, &order_len);
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
    servo_set_torque_switch_analysis(pack);

    //���ö������Ŀ�ʱĿ����ٶȵȼ�
    servo_sync_write_time_base_target_acc(1, sync_write_time_base_target_acc, order_buffer, &order_len);
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

    //���ö������Ŀ�ʱĿ��λ�ú��˶�ʱ��
    servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer, &order_len);
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
#endif

    //�رմ���
    CloseHandle(hSerial);

    return 0;
}

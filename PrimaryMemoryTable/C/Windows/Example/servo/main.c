#include "servo.h"
#include <windows.h>
#include <stdio.h>

#define READ_TEST 0                 // 读取舵机数据测试
#define WRITE_TEST 0                // 写入舵机数据测试
#define SYNC_WRITE_TEST 0           // 同步写测试
#define PING_TEST 0                 // PING命令测试
#define FACTORY_RESET_TEST 0        // 恢复出厂设置测试
#define PARAMETER_RESET_TEST 0      // 参数重置测试
#define REBOOT_TEST 0               // 重启测试
#define CALIBRATION_TEST 0          // 校正偏移值测试
#define MODIFY_ID 0                 // 修改舵机ID测试
#define MODIFY_UNKNOWN_ID 0         // 修改未知ID舵机ID测试


//串口初始化
uint8_t uart_init(HANDLE hSerial)
{
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        PRINTF("Failed to open serial port\n");
        return FALSE;
    }

    // 配置串口参数
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        PRINTF("Failed to get serial port parameters\n");
        CloseHandle(hSerial);
        return FALSE;
    }

    //设置串口协议参数
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

//串口发送
uint8_t order_send(HANDLE hSerial, uint8_t* order_buffer, uint8_t order_len)
{
    uint8_t ret;                    //状态标志位
    DWORD bytesWritten;             //实际写入数据长度

    //写入串口数据
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

//串口接收数据
uint8_t order_receive(HANDLE hSerial, uint8_t pack[])
{
    uint8_t ret;                //状态标志位
    DWORD bytesRead;            //实际读取数据长度
    DWORD errors;               //串口error标志位
    DWORD read_len;             //读取长度
    COMSTAT comstat;            //描述串口通信的状态信息

    if (!ClearCommError(hSerial, &errors, &comstat)) {
        return FALSE;
    }

    //获取接收缓冲区中可用的字节数
    read_len = comstat.cbInQue;

    //读取串口缓冲区数据
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

    uint8_t order_buffer[20] = { 0 };                                                                         //存放生成的指令
    uint8_t order_len = 0;                                                                                  //指令长度
    uint8_t pack[20] = { 0 };                                                                                 //存放接收的应答包
    uint16_t analysis_data = 0;                                                                             //应答包解析出来的数据
    uint16_t sync_write_velocity_base_target_position[5] = { 1, 0, 2, 0 };                    //同步写多个舵机控速目标位置
    uint16_t sync_write_velocity_base_target_velocity[5] = { 1, 3600, 2, 3600 };              //同步写多个舵机控速目标速度
    uint16_t sync_write_velocity_base_target_acc[5] = { 1, 150, 2, 150 };                     //同步写多个舵机控速目标加速度
    uint16_t sync_write_velocity_base_target_dec[5] = { 1, 150, 2, 150 };                     //同步写多个舵机控速目标减速度
    uint16_t sync_write_time_base_target_acc[5] = { 1, 0, 2, 0 };                             //同步写多个舵机控时目标加速度
    uint16_t sync_write_time_base_target_position_and_moving_time[10] = { 1, 3000, 500, 2, 3000, 500 };           //同步写多个舵机控时目标运动位置和运动时间


    uint8_t ret;

    // 打开串口
    HANDLE hSerial = CreateFile("COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    //串口初始化
    ret = uart_init(hSerial);
    if (ret == FALSE)
    {
        return FALSE;
    }

#if PING_TEST
    //向ID为1的舵机发送PING指令
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
    //校正偏移值
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
    //恢复出厂设置
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
    //参数重置
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
    //舵机重启
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
    // 修改ID1舵机ID为2
    servo_modify_known_id(1, 2, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);
#endif

#if MODIFY_UNKNOWN_ID
    // 将未知ID舵机ID修改为2
    servo_modify_unknown_id(2, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);
#endif

#if READ_TEST
    //读取ID1舵机的当前电流
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

    //读取ID1舵机的当前位置
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

    //读取ID1舵机的当前速度
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

    //读取ID1舵机的当前的规划位置
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

    //读取ID1舵机的当前规划速度
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

    //读取ID1舵机的当前PWM
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

    //读取ID1舵机的当前温度
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

    //读取ID1舵机的当前输入电压
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

    //读取ID1舵机的控时目标运行时间
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

    //读取ID1舵机的控时目标位置
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

    //读取ID1舵机的控时加速度等级
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

    //读取ID1舵机的控速目标减速度
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

    //读取ID1舵机的控速目标加速度
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

    //读取ID1舵机的控速目标速度
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

    //读取ID1舵机的控速目标位置
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

    //读取ID1舵机的目标电流
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

    //读取ID1舵机的目标PWM
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

    //读取ID1舵机的扭矩开关
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

    //读取ID1舵机的LED开关
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

    //读取ID1舵机的Flash开关
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

    //读取ID1舵机的电流校正值
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

    //读取ID1舵机的中位校正值
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

    //读取ID1舵机的控制模式
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

    //读取ID1舵机的卸载保护条件
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

    //读取ID1舵机的LED报警条件
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

    //读取ID1舵机的位置控制D增益
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

    //读取ID1舵机的位置控制I增益
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

    //读取ID1舵机的位置控制P增益
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

    //读取ID1舵机的PWM叠加值
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

    //读取ID1舵机的反转死区
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

    //读取ID1舵机的正转死区
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

    //读取ID1舵机的电流保护时间
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

    //读取ID1舵机的电流上限
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

    //读取ID1舵机的PWM上限
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

    //读取ID1舵机的电压上限
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

    //读取ID1舵机的电压下限
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

    //读取ID1舵机的温度上限
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

    //读取ID1舵机的最大位置限制
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

    //读取ID1舵机的最小位置限制
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

    //读取ID1舵机的状态返回级别
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

    //读取ID1舵机的应答延时时间
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

    //读取ID1舵机的波特率
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

    //读取ID1舵机的出厂编号
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

    //读取ID1舵机的固件版本号
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
    //设置ID1舵机的Flash开关
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

    //设置ID1舵机的应答延时时间
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

    //设置ID1舵机的状态返回级别
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

    //设置ID1舵机的波特率
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

    //设置ID1舵机的最小位置限制
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

    //设置ID1舵机的最大位置限制
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

    //设置ID1舵机的温度上限
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

    //设置ID1舵机的电压上限
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

    //设置ID1舵机的电压下限
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

    //设置ID1舵机的PWM上限
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

    //设置ID1舵机的电流上限
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

    //设置ID1舵机的电流保护时间
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

    //设置ID1舵机的正转死区
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

    //设置ID1舵机的反转死区
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

    //设置ID1舵机的PWM叠加值
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

    //设置ID1舵机的位置控制P增益
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

    //设置ID1舵机的位置控制I增益
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

    //设置ID1舵机的位置控制D增益
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

    //设置ID1舵机的LED报警条件
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

    //设置ID1舵机的卸载保护条件
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

    //设置ID1舵机的LED开关
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的目标PWM
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的目标电流
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控速目标速度
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

    //设置ID1舵机的控速目标加速度
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

    //设置ID1舵机的控速目标减速度
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

    //设置ID1舵机的控速目标位置
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控时目标加速度等级
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

    //设置ID1舵机的控时目标位置和目标运行时间
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
    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID2舵机的扭矩开关
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

    //设置ID2舵机的控制模式
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

    //设置ID2舵机的扭矩开关
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

    //设置多个舵机的控速目标加速度
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

    //设置多个舵机的控速目标减速度
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

    //设置多个舵机的控速目标速度
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

    //设置多个舵机的控速目标位置
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID2舵机的扭矩开关
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

    //设置ID2舵机的控制模式
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

    //设置ID2舵机的扭矩开关
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

    //设置多个舵机的控时目标加速度等级
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

    //设置多个舵机的控时目标位置和运动时间
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

    //关闭串口
    CloseHandle(hSerial);

    return 0;
}

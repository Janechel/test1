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
#define MODIFY_ID 0                 // 修改已知舵机ID测试
#define MODIFY_UNKNOWN_ID 0         // 修改未知ID舵机ID测试

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
    uint8_t ret = 0;                                                                                        //错误检验标志
    uint8_t order_buffer[50] = { 0 };                                                                       //存放生成的指令
    uint8_t order_len = 0;                                                                                  //指令长度
    uint8_t pack[20] = { 0 };                                                                               //存放接收的应答包
    uint16_t analysis_data = 0;                                                                             //应答包解析出来的数据
    uint16_t position = 0;                                                                                  //当前位置
    uint16_t current = 0;                                                                                   //当前电流
    uint8_t write_buffer[20] = { 0 };                                                                         //写入内存表数据

    // 打开串口
    HANDLE hSerial = CreateFile("\\\\.\\COM12", GENERIC_READ | GENERIC_WRITE, 0, NULL,
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
    ret = servo_ping_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("Ping succeed!  the model_number is %d\r\n", analysis_data);
    }
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
    ret = servo_calibration_analysis(pack);
    if (ret == SUCCESS)
    {
        PRINTF("servo calibration successfully!\r\n");
    }
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
    ret = servo_factory_reset_analysis(pack);
    if (ret == SUCCESS)
    {
        PRINTF("servo factory reset successfully!\r\n");
    }
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
    ret = servo_parameter_reset_analysis(pack);
    if (ret == SUCCESS)
    {
        PRINTF("servo parameter reset successfully!\r\n");
    }
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
    //修改ID1舵机ID为2
    servo_modify_known_id(1, 2, order_buffer, &order_len);
    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);
#endif

#if MODIFY_UNKNOWN_ID
    //将未知ID舵机的ID编号修改为1
    servo_modify_unknown_id(1, order_buffer, &order_len);
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
    ret = servo_read_present_current_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present current is %d\r\n", analysis_data);
    }

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
    ret = servo_read_present_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present position is %d\r\n", analysis_data);
    }

    //读取ID1舵机的当前位置和当前电流
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
    ret = servo_read_present_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity is %d\r\n", analysis_data);
    }

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

    ret = servo_read_present_profile_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present profile position is %d\r\n", analysis_data);
    }

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

    ret = servo_read_present_profile_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present profile velocity is %d\r\n", analysis_data);
    }

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
    ret = servo_read_present_pwm_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present pwm analysis is %d\r\n", analysis_data);
    }

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
    ret = servo_read_present_temperature_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present temperature is %d\r\n", analysis_data);
    }

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
    ret = servo_read_present_voltage_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present voltage is %d\r\n", analysis_data);
    }

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
    ret = servo_read_time_base_target_moving_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present time base target moving time is %d\r\n", analysis_data);
    }

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
    ret = servo_read_time_base_target_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present time base target position is %d\r\n", analysis_data);
    }

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
    ret = servo_read_time_base_target_acc_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present time base target acc is %d\r\n", analysis_data);
    }

    //读取ID1舵机的控时目标位置和运行时间
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

    //读取ID1舵机的控时目标加速度等级、位置和运行时间
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
    ret = servo_read_velocity_base_target_dec_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity base target dec is %d\r\n", analysis_data);
    }

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
    ret = servo_read_velocity_base_target_acc_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity base target acc is %d\r\n", analysis_data);
    }

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
    ret = servo_read_velocity_base_target_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity base target velocity is %d\r\n", analysis_data);
    }

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
    ret = servo_read_velocity_base_target_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("present velocity base target position is %d\r\n", analysis_data);
    }

    //读取ID1舵机的控速目标位置和速度
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

    //读取ID1舵机的控速目标位置、速度和加减速度
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
    ret = servo_read_target_current_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("target current is %d\r\n", analysis_data);
    }

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
    ret = servo_read_target_pwm_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("target pwm is %d\r\n", analysis_data);
    }

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
    ret = servo_read_torque_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("torque switch is %d\r\n", analysis_data);
    }

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
    ret = servo_read_led_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("led switch is %d\r\n", analysis_data);
    }

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
    ret = servo_read_flash_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("flash switch is %d\r\n", analysis_data);
    }

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
    ret = servo_read_current_offset_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("current offset is %d\r\n", analysis_data);
    }

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
    ret = servo_read_calibration_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("calibration is %d\r\n", analysis_data);
    }

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
    ret = servo_read_control_mode_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("control mode is %d\r\n", analysis_data);
    }

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
    ret = servo_read_shutdown_condition_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("shutdown condition is %d\r\n", analysis_data);
    }

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
    ret = servo_read_led_condition_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("led condition is %d\r\n", analysis_data);
    }

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
    ret = servo_read_position_control_d_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("position control d gain is %d\r\n", analysis_data);
    }

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
    ret = servo_read_position_control_i_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("position control i gain is %d\r\n", analysis_data);
    }

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
    ret = servo_read_position_control_p_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("position control p gain is %d\r\n", analysis_data);
    }

    //读取ID1舵机的位置控制PID增益
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
    ret = servo_read_pwm_punch_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("pwm punch is %d\r\n", analysis_data);
    }

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
    ret = servo_read_ccw_deadband_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("ccw deadband is %d\r\n", analysis_data);
    }

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
    ret = servo_read_cw_deadband_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("cw deadband is %d\r\n", analysis_data);
    }

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
    ret = servo_read_current_shutdown_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("current shutdown time is %d\r\n", analysis_data);
    }

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
    ret = servo_read_max_current_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max current limit is %d\r\n", analysis_data);
    }

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
    ret = servo_read_max_pwm_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max pwm limit is %d\r\n", analysis_data);
    }

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
    ret = servo_read_max_voltage_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max voltage limit is %d\r\n", analysis_data);
    }

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
    ret = servo_read_min_voltage_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("min voltage limit is %d\r\n", analysis_data);
    }

    //读取ID1舵机的电压限制
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
    ret = servo_read_max_temperature_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max temperature limit is %d\r\n", analysis_data);
    }

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
    ret = servo_read_max_angle_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("max angle limit is %d\r\n", analysis_data);
    }

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
    ret = servo_read_min_angle_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("min angle limit is %d\r\n", analysis_data);
    }

    //读取ID1舵机的位置限制
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
    ret = servo_read_return_level_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("return level is %d\r\n", analysis_data);
    }

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
    ret = servo_read_return_delay_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("return delay time is %d\r\n", analysis_data);
    }

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
    ret = servo_read_baud_rate_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("baud rate is %d\r\n", analysis_data);
    }

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
    ret = servo_read_model_information_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("model information is %d\r\n", analysis_data);
    }

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
    ret = servo_read_firmware_version_analysis(pack, &analysis_data);
    if (ret == SUCCESS)
    {
        PRINTF("firmware version is %d\r\n", analysis_data);
    }
#endif

#if WRITE_TEST
    //将ID1舵机的应答延迟时间修改为500us
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

    //将ID1舵机的状态返回级别修改为应答所有指令
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

    //设置ID1舵机的波特率为1000000
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

    //将舵机ID1的最小位置限制修改为0°
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

    //将舵机ID1的最大位置限制修改为300°
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

    //将舵机ID1的位置限制修改为0°~300°
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

    //将ID1舵机的温度上限修改为65℃
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

    //将ID1舵机的电压上限修改为8.4V
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

    //将ID1舵机的电压下限修改为3.5V
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

    //将ID1舵机的电压限制修改为3.5V~8.4V
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

    //将ID1舵机的PWM上限修改为90%
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

    //将ID1舵机的电流上限修改为900mA
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

    //将ID1舵机的电流保护时间修改为500ms
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

    //将ID1舵机的正转死区修改为0.2°
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

    //将ID1舵机的反转死区修改为0.2°
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

    //将ID1舵机的正反转死区修改为0.2°
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

    //将ID1舵机的PWM叠加值修改为1%
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

    //将ID1舵机的位置控制P增益修改为5995
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

    //将ID1舵机的位置控制I增益修改为5
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

    //将ID1舵机的位置控制D增益修改为145
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

    //将ID1舵机的位置控制PID增益，分别修改为5500、100、250
    write_buffer[0] = 5500 & 0xff;
    write_buffer[1] = (5500 >> 8) & 0xff;
    write_buffer[2] = 100 & 0xff;
    write_buffer[3] = (100 >> 8) & 0xff;
    write_buffer[4] = 250 & 0xff;
    write_buffer[5] = (250 >> 8) & 0xff;

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

    //将ID1舵机的LED报警条件修改为开启堵转报错、过热报错和角度报错
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

    //将ID1舵机的卸载保护条件修改为开启堵转报错、过热报错、电压报错和角度报错
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

    //将ID1舵机的Flash开关状态修改为打开
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

    //将ID1舵机的Flash开关状态修改为关闭
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

    //将ID1舵机的LED开关状态修改为打开
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

    //将ID1舵机的LED开关状态修改为关闭
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

    //设置ID1舵机的扭矩开关为关闭
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

    //将ID1舵机的控制模式修改为PWM输出控制模式
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

    //设置ID1舵机的扭矩开关为开启
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

    //将ID1舵机的目标PWM修改为-50%
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

    //设置ID1舵机的扭矩开关为关闭
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

    //将ID1舵机的控制模式修改为电流控制模式
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

    //设置ID1舵机的扭矩开关为开启
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

    //将ID1舵机的目标电流修改为-400mA
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

    //设置ID1舵机的扭矩开关为关闭
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

    //将ID1舵机的控制模式修改为控速模式
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

    //设置ID1舵机的扭矩开关为开启
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

    //将ID1舵机的控速目标速度修改为360°/s
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

    //将ID1舵机的控速目标加速度修改为500°/s²
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

    //将ID1舵机的控速目标减速度修改为50°/s²
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

    //将ID1舵机的控速目标位置修改为150°
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

    //设置ID1舵机的扭矩开关为关闭
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

    //将ID1舵机的控制模式修改为控时模式
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

    //设置ID1舵机的扭矩开关为开启
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

    //将ID1舵机的控时目标加速度等级修改为5
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

    //将ID1舵机的控时目标位置和运行时间，分别修改为300°、500ms
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
    servo.id_counts = 2;            //同步写两个舵机
    servo.id[0] = 1;                //第一个舵机id为1
    servo.id[1] = 2;                //第二个舵机id为2

    //将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
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

    //将ID1、ID2舵机的控制模式，分别修改为控速模式
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

    //设置多个舵机的控速目标加速度   

    //id为1，2的舵机加速度分别设置为150，150，值和前面的id设置对应
    servo.acc_velocity[0] = 150;
    servo.acc_velocity[1] = 150;

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

    //设置多个舵机的控速目标减速度

    //id为1，2的舵机减速度分别设置为150，150，值和前面的id设置对应
    servo.dec_velocity[0] = 150;
    servo.dec_velocity[1] = 150;

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

    //设置多个舵机的控速目标速度

    //id为1，2的舵机速度分别设置为3600，1800，值和前面的id设置对应
    servo.velocity[0] = 3600;
    servo.velocity[1] = 1800;

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

    //设置多个舵机的控速目标位置

    //id为1，2的舵机运动位置分别设置为0，0，值和前面的id设置对应
    servo.position[0] = 0;
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

    //设置多个舵机的控速目标位置和速度

    //id为1，2的舵机速度分别设置为1800，3600，位置分别设置为3000，3000
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

    //设置多个舵机的加速度，减速度，速度和位置

    //id为1，2的舵机速度分别设置为3600，3600，位置分别设置为0，0,加速度分别设置为100，100，减速度分别设置为100，100
    servo.velocity[0] = 3600;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 0;
    servo.acc_velocity[0] = 100;
    servo.acc_velocity[1] = 100;
    servo.dec_velocity[0] = 100;
    servo.dec_velocity[1] = 100;

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

    //将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
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

    //将ID1、ID2舵机的控制模式，分别修改为控时模式
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

    //设置多个舵机的控时目标加速度等级

    //设置舵机id为1，2的加速度等级分别为0，0
    servo.acc_velocity_grade[0] = 0;
    servo.acc_velocity_grade[1] = 0;

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

    //设置多个舵机的控时目标位置和运动时间

    //设置舵机id为1，2的运动位置为3000，3000，运动时间为500ms，1500ms
    servo.position[0] = 3000;
    servo.position[1] = 3000;
    servo.time[0] = 500;
    servo.time[1] = 1500;

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

    //关闭串口
    CloseHandle(hSerial);

    return 0;
}

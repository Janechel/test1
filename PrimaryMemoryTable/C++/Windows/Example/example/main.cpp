#include <iostream>
#include "servo.h"
#include "CSerialPort.h"

#define READ_TEST 0             //读取舵机数据测试
#define WRITE_TEST 0            //写入舵机数据测试
#define SYNC_WRITE_TEST 0       //同步写测试
#define PING_TEST 0             //PING命令测试
#define FACTORY_RESET_TEST 0    //恢复出厂设置测试
#define PARAMETER_RESET_TEST 0  //参数重置测试
#define REBOOT_TEST 0           //重启测试
#define CALIBRATION_TEST 0      //校正偏移值测试
#define MODIFY_ID 0             //修改舵机ID测试
#define MODIFY_UNKNOWN_ID 0     //修改未知ID舵机测试

struct servo_sync_parameter servo;

int main()
{

    uint8_t ret;
    uint8_t order_buffer[40];                                                                         //存放生成的指令
    uint8_t order_len = 0;                                                                                  //指令长度
    uint8_t pack[40];                                                                                 //存放接收的应答包
    uint16_t analysis_data = 0;                                                                             //应答包解析出来的数据

    CSerialPort serialPort;

    //实际串口读取到的字节数
    DWORD bytesRead;
    //实际串口写入的字节数
    DWORD bytesWritten;

    if (serialPort.Open(12, 1000000))
    {
        PRINTF("\r\nOpen Serial successfully.");
    }
    else
    {
        // 串口打开失败
        PRINTF("\r\nFailed to open serial port.");
        return -1;
    }

#if PING_TEST
    //向id为1的舵机发送ping指令
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
    //校正偏移值
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
    //恢复出厂设置
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
    //参数重置
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
    //重启舵机
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
    //读取ID1舵机的当前电流
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

    //读取ID1舵机的当前位置
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

    //读取ID1舵机的当前速度
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

    //读取ID1舵机的当前的规划位置
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

    //读取ID1舵机的当前规划速度
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

    //读取ID1舵机的当前PWM
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

    //读取ID1舵机的当前温度
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

    //读取ID1舵机的当前输入电压
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

    //读取ID1舵机的控时目标运行时间
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

    //读取ID1舵机的控时目标位置
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

    //读取ID1舵机的控时加速度等级
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

    //读取ID1舵机的控速目标减速度
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

    //读取ID1舵机的控速目标加速度
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

    //读取ID1舵机的控速目标速度
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

    //读取ID1舵机的控速目标位置
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

    //读取ID1舵机的目标电流
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

    //读取ID1舵机的目标PWM
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

    //读取ID1舵机的扭矩开关
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

    //读取ID1舵机的LED开关
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

    //读取ID1舵机的Flash开关
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

    //读取ID1舵机的电流校正值
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

    //读取ID1舵机的中位校正值
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

    //读取ID1舵机的控制模式
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

    //读取ID1舵机的卸载保护条件
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

    //读取ID1舵机的LED报警条件
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

    //读取ID1舵机的位置控制D增益
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

    //读取ID1舵机的位置控制I增益
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

    //读取ID1舵机的位置控制P增益
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

    //读取ID1舵机的PWM叠加值
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

    //读取ID1舵机的反转死区
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

    //读取ID1舵机的正转死区
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

    //读取ID1舵机的电流保护时间
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

    //读取ID1舵机的电流上限
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

    //读取ID1舵机的PWM上限
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

    //读取ID1舵机的电压上限
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

    //读取ID1舵机的电压下限
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

    //读取ID1舵机的温度上限
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

    //读取ID1舵机的最大位置限制
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

    //读取ID1舵机的最小位置限制
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

    //读取ID1舵机的状态返回级别
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

    //读取ID1舵机的应答延时时间
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

    //读取ID1舵机的波特率
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

    //读取ID1舵机的出厂编号
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

    //读取ID1舵机的固件版本号
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
    //设置ID1舵机的应答延时时间
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

    //设置ID1舵机的状态返回级别
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

    //设置ID1舵机的波特率
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

    //设置ID1舵机的最小位置限制
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

    //设置ID1舵机的最大位置限制
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

    //设置ID1舵机的温度上限
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

    //设置ID1舵机的电压上限
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

    //设置ID1舵机的电压下限
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

    //设置ID1舵机的PWM上限
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

    //设置ID1舵机的电流上限
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

    //设置ID1舵机的电流保护时间
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

    //设置ID1舵机的正转死区
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

    //设置ID1舵机的反转死区
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

    //设置ID1舵机的PWM叠加值
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

    //设置ID1舵机的位置控制P增益
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

    //设置ID1舵机的位置控制I增益
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

    //设置ID1舵机的位置控制D增益
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

    //设置ID1舵机的LED报警条件
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

    //设置ID1舵机的卸载保护条件
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

    //设置ID1舵机的Flash开关
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

    //设置ID1舵机的LED开关
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的目标PWM
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的目标电流
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控速目标速度
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

    //设置ID1舵机的控速目标加速度
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

    //设置ID1舵机的控速目标减速度
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

    //设置ID1舵机的控速目标位置
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控时目标加速度等级
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

    //设置ID1舵机的控时目标位置和目标运行时间
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
    servo.id_counts = 2;            //同步写两个舵机
    servo.id[0] = 1;                //第一个舵机id为1
    servo.id[1] = 2;                //第二个舵机id为2

    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID2舵机的扭矩开关
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

    //设置ID2舵机的控制模式
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

    //设置ID2舵机的扭矩开关
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

    //设置多个舵机的控速目标速度

    //id为1，2的舵机速度分别设置为3600，1800，值和前面的id设置对应
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

    //设置多个舵机的控速目标加速度

    //id为1，2的舵机加速度分别设置为150，150，值和前面的id设置对应
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

    //设置多个舵机的控速目标减速度

    //id为1，2的舵机减速度分别设置为150，150，值和前面的id设置对应
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

    //设置多个舵机的控速目标位置

    //id为1，2的舵机运动位置分别设置为0，0，值和前面的id设置对应
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

    //设置多个舵机的控速目标位置和速度

    //id为1，2的舵机速度分别设置为1800，3600，位置分别设置为3000，3000
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
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);


    //设置ID1舵机的扭矩开关
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

    //设置ID1舵机的控制模式
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

    //设置ID1舵机的扭矩开关
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

    //设置ID2舵机的扭矩开关
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

    //设置ID2舵机的控制模式
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

    //设置ID2舵机的扭矩开关
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

    //设置多个舵机的控时目标加速度等级

    //设置舵机id为1，2的加速度等级分别为0，0
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

    //设置多个舵机的控时目标位置和运动时间

    //设置舵机id为1，2的运动位置为3000，3000，运动时间为500ms，1500ms
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
    //将id为1的舵机id修改为2
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
    //将所有舵机id修改为2
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

    // 关闭串口
    serialPort.Close();

    return 0;
}

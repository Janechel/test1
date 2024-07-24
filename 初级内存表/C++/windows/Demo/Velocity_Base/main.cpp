#include <iostream>
#include "servo.h"
#include "CSerialPort.h"

int main()
{
    uint8_t ret;
    uint8_t order_buffer[20] = {0};                                                                         //存放生成的指令
    uint8_t order_len = 0;                                                                                  //指令长度
    uint8_t pack[20] = {0};                                                                                 //存放接收的应答包

    //创建舵机类和串口的控制类
    Servo servo;
    CSerialPort serialPort;

    //实际串口读取到的字节数
    DWORD bytesRead;
    //实际串口写入的字节数
    DWORD bytesWritten;

    if (serialPort.Open(14, 1000000))
    {
        PRINTF("\r\nOpen Serial successfully.");
    }
    else
    {
        // 串口打开失败
        PRINTF("\r\nFailed to open serial port.");
        return -1;
    }

    //参数重置
    servo.servo_parameter_reset(1,order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nData sent successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_parameter_reset_analysis(pack);
        if(ret == SUCCESS)
        {
            PRINTF("\r\nservo parameter reset successfully!");
        }
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //设置舵机的扭矩开关
    servo.servo_set_torque_switch(1, 0, order_buffer,&order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_set_torque_switch_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //设置舵机的控制模式
    servo.servo_set_control_mode(1, 1, order_buffer,&order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_set_control_mode_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //设置舵机的扭矩开关
    servo.servo_set_torque_switch(1, 1, order_buffer,&order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_set_torque_switch_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //设置舵机的控速目标位置
    servo.servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_set_velocity_base_target_position_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(1000);

    //设置舵机的控速目标速度
    servo.servo_set_velocity_base_target_velocity(1, 3600, order_buffer,&order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_set_velocity_base_target_velocity_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //设置舵机的控速目标加速度
    servo.servo_set_velocity_base_target_acc(1, 10, order_buffer,&order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_set_velocity_base_target_acc_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //设置舵机的控速目标减速度
    servo.servo_set_velocity_base_target_dec(1, 1, order_buffer,&order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_set_velocity_base_target_dec_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(20);

    //设置舵机的控速目标位置
    servo.servo_set_velocity_base_target_position(1, 0, order_buffer,&order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten)) {
        PRINTF("\r\nWrite successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1);

    if (serialPort.Read(pack,&bytesRead))
    {
        ret = servo.servo_set_velocity_base_target_position_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("\r\nSet successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(1000);

    // 关闭串口
    serialPort.Close();

    return 0;
}

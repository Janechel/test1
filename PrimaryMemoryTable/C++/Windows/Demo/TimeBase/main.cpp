#include <iostream>
#include "servo.h"
#include "CSerialPort.h"

int main()
{
    uint8_t ret;          
    uint8_t order_buffer[20] = { 0 };                               //存放生成的指令
    uint8_t order_len = 0;                                             //指令长度
    uint8_t pack[20] = { 0 };                                         //存放接收的应答包
    uint8_t write_buffer[20] = { 0 };                              //写入内存表数据   
    struct servo_sync_parameter servo;

    servo.id_counts = 2;            //同步写两个舵机
    servo.id[0] = 1;                //第一个舵机id为1
    servo.id[1] = 2;                //第二个舵机id为2

    //创建串口的控制类
    CSerialPort serialPort;

    //实际串口读取到的字节数
    DWORD bytesRead;
    //实际串口写入的字节数
    DWORD bytesWritten;

    if (serialPort.Open(16, 1000000))
    {
        PRINTF("\r\nOpen Serial successfully.");
    }
    else
    {
        // 串口打开失败
        PRINTF("\r\nFailed to open serial port.");
        return -1;
    }

    //将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    servo_sync_write_torque_switch(servo, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write torque witch successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    //将ID1、ID2舵机的控制模式，分别修改为控时模式
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write control mode successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    //设置舵机的控时目标位置和目标运行时间
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

    //设置舵机的控时目标位置和目标运行时间
    servo_set_time_base_target_position_and_moving_time(1, 0, 1000, order_buffer, &order_len);
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

    //在控时模式下，让ID1舵机以500ms运动到150°位置，让ID2舵机以1s匀速运动到0°位置

    //设置舵机id为1，2的运动位置为3000，3000，运动时间为500ms，1500ms
    servo.position[0] = 3000;
    servo.position[1] = 3000;
    servo.time[0] = 500;
    servo.time[1] = 1500;

    servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write time base target position and moving time successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);

    //设置舵机id为1，2的运动位置为0，3000，运动时间为1000ms，500ms
    servo.position[0] = 0;
    servo.position[1] = 3000;
    servo.time[0] = 1000;
    servo.time[1] = 500;

    servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("\r\nSync Write time base target position and moving time successfully.");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);

    // 关闭串口
    serialPort.Close();

    return 0;
}

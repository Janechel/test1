#include <iostream>
#include "servo.h"
#include "CSerialPort.h"

int main()
{
    uint8_t ret;
    uint8_t order_buffer[40] = { 0 };                                                                         //存放生成的指令
    uint8_t order_len = 0;                                                                                  //指令长度
    uint8_t pack[20] = { 0 };                                                                                 //存放接收的应答包
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
        PRINTF("Sync Write torque witch successfully.\r\n");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    //将ID1、ID2舵机的控制模式，分别修改为控速模式
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    servo_sync_write_control_mode(servo, order_buffer, &order_len);

    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("Sync Write control mode successfully.\r\n");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(20);

    //设置舵机的控速目标位置
    servo_set_velocity_base_target_position(1, 1500, order_buffer, &order_len);
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

    //在控速模式下，让ID1以360°/s的控速目标速度运动到300°位置
    write_buffer[0] = 3000 & 0xff;
    write_buffer[1] = (3000 >> 8) & 0xff;
    write_buffer[2] = 3600 & 0xff;
    write_buffer[3] = (3600 >> 8) & 0xff;

    servo_write(1, 0x35, 4, write_buffer, order_buffer, &order_len);

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
        PRINTF("servo pack is: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(1000);

    //在控速模式下，让ID1舵机以360°/s的控速目标速度、500°/s²的控速目标加速度、50°/s²的控速目标减速度运动到0°位置
    write_buffer[0] = 0 & 0xff;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3600 & 0xff;
    write_buffer[3] = (3600 >> 8) & 0xff;
    write_buffer[4] = 10 & 0xff;
    write_buffer[5] = 1 & 0xff;

    servo_write(1, 0x35, 6, write_buffer, order_buffer, &order_len);

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
        PRINTF("servo pack is: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("\r\nFailed to read data.");
    }
    Sleep(1000);

    //在控速模式下，让ID1舵机运动到150中位°，让ID2舵机运动到0°位置

    //id为1，2的舵机运动位置分别设置为1500，0，值和前面的id设置对应
    servo.position[0] = 1500;
    servo.position[1] = 0;

    servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("Sync Write velocity base target position successfully.\r\n");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);

    //在控速模式下，让ID1以360°/s的控速目标速度运动到300°位置，让ID2以720°/s的控速目标速度运动到150°位置

    //id为1，2的舵机速度分别设置为3600，7200，位置分别设置为3000，1500
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;
    servo.position[0] = 3000;
    servo.position[1] = 1500;

    servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("Sync Write velocity base target position and velocity successfully.\r\n");
    }
    else
    {
        PRINTF("\r\nFailed to send data.");
    }
    Sleep(1000);

    //在控速模式下，让ID1舵机以720°/s的控速目标速度、500°/s²的控速目标加速度、50°/s²的控速目标减速度运动到0°位置，让ID2舵机以360°/s的控速目标速度、50°/s²的控速目标加速度、500°/s²的控速目标减速度运动到300°位置

    //id为1，2的舵机速度分别设置为7200，3600，位置分别设置为0，3000,加速度分别设置为10，1，减速度分别设置为1，10
    servo.velocity[0] = 7200;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 3000;
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 1;
    servo.dec_velocity[0] = 1;
    servo.dec_velocity[1] = 10;

    servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);
    if (serialPort.Write(order_buffer, order_len, &bytesWritten))
    {
        PRINTF("Sync Write velocity base target acc,dec,velocity and position successfully.\r\n");
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

#include "servo.h"
#include <windows.h>
#include <stdio.h>

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
    uint8_t ret;
    uint8_t write_buffer[20] = { 0 };                                                                         //写入内存表数据
    struct servo_sync_parameter servo;

    // 打开串口
    HANDLE hSerial = CreateFile("\\\\.\\COM16", GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    //串口初始化
    ret = uart_init(hSerial);
    if (ret == FALSE)
    {
        return FALSE;
    }

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

    //将ID1、ID2舵机的控制模式，分别修改为控shi模式
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


    //在控时模式下，让ID1舵机以500ms运动到300°位置
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
    Sleep(1000);

    servo_set_time_base_target_position_and_moving_time_analysis(pack);

    //在控时模式下，让ID1舵机以1s匀速运动到0°位置
    servo_set_time_base_target_position_and_moving_time(1, 0, 1000, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE)
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

    servo_set_time_base_target_position_and_moving_time_analysis(pack);

    //在控时模式下，让ID1舵机以500ms运动到150°位置，让ID2舵机以1s匀速运动到0°位置
    servo.position[0] = 1500;
    servo.position[1] = 0;
    servo.time[0] = 500;
    servo.time[1] = 1000;

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

    //在控时模式下，让ID1舵机以1000ms运动到0°位置，让ID2舵机以500ms匀速运动到3000°位置
    servo.position[0] = 0;
    servo.position[1] = 3000;
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

    //关闭串口
    CloseHandle(hSerial);

    return 0;
}

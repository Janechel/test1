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
uint8_t order_send(HANDLE hSerial, uint8_t *order_buffer,uint8_t order_len)
{
    uint8_t ret;                    //状态标志位
    DWORD bytesWritten;             //实际写入数据长度

    //写入串口数据
    ret = WriteFile(hSerial, order_buffer, order_len, &bytesWritten, NULL);

    if(ret != 0)
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

    if(ret != 0)
    {
        if(bytesRead > 0)
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

    uint8_t order_buffer[20] = {0};                                                                         //存放生成的指令
    uint8_t order_len = 0;                                                                                  //指令长度
    uint8_t pack[20] = {0};                                                                                 //存放接收的应答包

    uint8_t ret;

    // 打开串口
    HANDLE hSerial = CreateFile("COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    //串口初始化
    ret = uart_init(hSerial);
    if (ret == FALSE) {
        return FALSE;
    }

    //设置舵机的扭矩开关
    servo_set_torque_switch(1, 0, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE) {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE) {
        return FALSE;
    }
    Sleep(80);
    servo_set_torque_switch_analysis(pack);

    //设置舵机的控制模式
    servo_set_control_mode(1, 2, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE) {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE) {
        return FALSE;
    }
    Sleep(80);
    servo_set_control_mode_analysis(pack);

    //设置舵机的扭矩开关
    servo_set_torque_switch(1, 1, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE) {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE) {
        return FALSE;
    }
    Sleep(80);
    servo_set_torque_switch_analysis(pack);

    //设置舵机的目标电流
    servo_set_target_current(1, 100, order_buffer, &order_len);

    ret = order_send(hSerial, order_buffer, order_len);
    if (ret == FALSE) {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if (ret == FALSE) {
        return FALSE;
    }
    Sleep(3000);

    servo_set_target_current_analysis(pack);

    //关闭串口
    CloseHandle(hSerial);

    return 0;

}

#include "servo.h"
#include <windows.h>
#include <stdio.h>

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

//uart send
uint8_t order_send(HANDLE hSerial, uint8_t *order_buffer,uint8_t order_len)
{
    uint8_t ret;
    DWORD bytesWritten;

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

//uart receiver
uint8_t order_receive(HANDLE hSerial, uint8_t pack[])
{
    uint8_t ret;
    DWORD bytesRead;
    DWORD errors;
    DWORD read_len;
    COMSTAT comstat;

    if (!ClearCommError(hSerial, &errors, &comstat)) {
        return FALSE;
    }

    read_len = comstat.cbInQue;

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

    uint8_t order_buffer[20] = {0};                                                                         //Store Generated Instructions
    uint8_t order_len = 0;                                                                                  //Instruction Length
    uint8_t pack[20] = {0};                                                                                 //Store the received status packet
    uint8_t ret;

    //Open Serial
    HANDLE hSerial = CreateFile("COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    //uart init
    ret = uart_init(hSerial);
    if(ret == FALSE)
    {
        return FALSE;
    }

    //Change the torque switch of servo ID1 to OFF.
    servo_set_torque_switch(1, 0, order_buffer,&order_len);

    ret = order_send(hSerial, order_buffer,order_len);
    if(ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if(ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    servo_set_torque_switch_analysis(pack);

    //Change the control mode of servo ID1 to the PWM control mode.
    servo_set_control_mode(1, 3, order_buffer,&order_len);

    ret = order_send(hSerial, order_buffer,order_len);
    if(ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if(ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    servo_set_control_mode_analysis(pack);

    //Change the torque switch of servo ID1 to ON.
    servo_set_torque_switch(1, 1, order_buffer,&order_len);

    ret = order_send(hSerial, order_buffer,order_len);
    if(ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if(ret == FALSE)
    {
        return FALSE;
    }
    Sleep(80);
    servo_set_torque_switch_analysis(pack);

    //Change the target PWM of servo ID1 to -50%.
    servo_set_target_pwm(1, -500, order_buffer,&order_len);

    ret = order_send(hSerial, order_buffer,order_len);
    if(ret == FALSE)
    {
        return FALSE;
    }
    Sleep(1);

    ret = order_receive(hSerial, pack);
    if(ret == FALSE)
    {
        return FALSE;
    }
    Sleep(3000);

    servo_set_target_pwm_analysis(pack);

    //Close Serial
    CloseHandle(hSerial);

    return 0;
}

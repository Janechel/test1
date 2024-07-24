#pragma once

#include <string>
#include <windows.h>
#include <tchar.h>

#ifdef _UNICODE
using _tstring = std::wstring;
#else
using _tstring = std::string;
#endif

class CSerialPort
{
public:
    CSerialPort();
    CSerialPort(const CSerialPort& r) = delete;
    ~CSerialPort();

    // 打开串口
    bool Open(
            int nComm,                      //串口号
            DWORD dwBaudRate = CBR_115200,  //波特率
            BYTE bByteSize = 8,             //数据位
            BYTE bParity = NOPARITY,        //校验位
            BYTE bStopBits = ONESTOPBIT     //停止位
    );

    // 打开串口
    bool Open(
            const _tstring& strName = _T("COM1"),        //串口名
            DWORD dwBaudRate = CBR_115200,  //波特率
            BYTE bByteSize = 8,             //数据位
            BYTE bParity = NOPARITY,        //校验位
            BYTE bStopBits = ONESTOPBIT     //停止位
    );

    // 检测是否已经打开
    bool IsOpen() const;

    // 设置状态配置
    bool SetState(
            DWORD dwBaudRate = CBR_115200,  //波特率
            BYTE bByteSize = 8,             //数据位
            BYTE bParity = NOPARITY,        //校验位
            BYTE bStopBits = ONESTOPBIT     //停止位
    );

    // 设置状态配置
    bool SetState(LPDCB lpDcb);

    // 设置缓冲区大小
    bool SetupComm(
            DWORD dwInQueue = 14,   // 输入缓冲区 1 - 14
            DWORD dwOutQueue = 16   // 输出缓冲区 1 - 16
    );

    // 设置超时配置
    bool SetTimeOut(
            DWORD ReadInterval = 50,          // 读间隔超时
            DWORD ReadTotalMultiplier = 5,   // 读时间系数
            DWORD ReadTotalConstant = 1000,            // 读时间常量
            DWORD WriteTotalMultiplier = 10,         // 写时间系数
            DWORD WriteTotalConstant = 200            // 写时间常量
    );

    // 设置超时配置
    bool SetTimeOut(LPCOMMTIMEOUTS lpTimeOuts);

    // 清空缓冲区
    bool Purge(
            DWORD  dwFlags = PURGE_TXCLEAR | PURGE_RXCLEAR
    );

    // 清除错误
    bool ClearError();

    // 关闭串口
    void Close();

    // 发送数据
    bool Write(
            LPCVOID lpData,
            DWORD cbSize,
            LPDWORD lpBytesWritten = nullptr,
            DWORD nTimeOut = 1000
    );

    // 接收数据
    bool Read(
            LPVOID lpData,
            LPDWORD lpBytesRead = nullptr,
            DWORD nTimeOut = 3000
    );

private:

    // 打开串口
    bool OpenComm(int nComm);

    // 打开串口
    bool OpenComm(const _tstring& strName);

    // 等待重叠操作完成
    bool WaitForOverlapped(
            HANDLE hFile,
            LPOVERLAPPED lpOv,
            LPDWORD lpBytesTransferred,
            DWORD nTimeOut
    );

private:

    HANDLE m_hComm;             // 串口句柄
    HANDLE m_hReadEvent;        // 重叠读事件句柄
    HANDLE m_hWriteEvent;       // 重叠写事件句柄
    bool m_bExit;               // 退出标记
};
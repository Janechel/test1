# 舵机测试 C

本项目通过C语言完成舵机EM3的功能测试，在windows平台，可以直接添加main.c、servo.c和servo.h文件进行测试。

## 依赖

- windows.h

## 使用

1. 准备硬件连接

2. 在C项目中添加main.c、servo.c和servo.h文件

3. 添加windows头文件

4. 将所需要测试的部分对应值改为1，修改舵机ID可能会导致其他功能测试找不到指定ID！
```c
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
```

5. 需要printf输出在servo.h中将PRINTF_ENABLE宏定义修改为1
```c
#define PRINTF_ENABLE 1

#if PRINTF_ENABLE
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
```
6. 运行
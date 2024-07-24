# ESP32测试 Micro Python

## 使用

1. 准备硬件连接

2. 下载main.py和servo.mpy文件

3. 将所需要测试的部分对应值改为1
```python
READ_TEST = 1  # 读取舵机数据测试
FACTORY_RESET_TEST = 0  # 恢复出厂设置测试
PARAMETER_RESET_TEST = 0  # 参数重置测试
CALIBRATION_TEST = 0  # 校正偏移值测试
REBOOT_TEST = 0  # 重启测试
WRITE_TEST = 1  # 写入舵机数据测试
SYNC_WRITE = 1  # 同步写测试
MODIFY_ID = 0  # 修改舵机ID测试
MODIFY_UNKNOWN_ID = 0  # 修改未知ID舵机ID测试
```
修改舵机ID可能会导致其他功能测试找不到指定ID！

4. 运行

ps:由于直接导入servo.py模块会因为内存碎片化的问题导致导入模块失败，现通过冻结字节码将py文件转换成mpy文件，解决内存分配问题。

mpy文件无法查看源代码，要查看源代码请在”servo-sdk\python\src“中查看servo.py
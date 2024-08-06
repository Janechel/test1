# 舵机测试 Python

在新项目中加入main.py和servo.py文件

## 依赖

- time
- serial

## 使用

1. 准备硬件连接

2. 导入servo、time和serial模块
```python
from servo import *
import serial
import time
```
3. 将所需要测试的部分对应值改为1
```python
READ_TEST = 0  # 读取舵机数据测试
FACTORY_RESET_TEST = 0  # 恢复出厂设置测试
PARAMETER_RESET_TEST = 0  # 参数重置测试
CALIBRATION_TEST = 0  # 校正偏移值测试
REBOOT_TEST = 0  # 重启测试
WRITE_TEST = 0  # 写入舵机数据测试
SYNC_WRITE = 0  # 同步写测试
MODIFY_ID = 0  # 修改舵机ID测试
MODIFY_UNKNOWN_ID = 0  # 修改未知ID舵机ID测试
```
修改舵机ID可能会导致其他功能测试找不到指定ID！

4. 运行
# ServoSDK
---
欢迎使用ServoSDK，这是专为 _EnergizeLab Servo_ 设计的开源软件开发工具包。本SDK提供了多语言、多平台的舵机使用Example和运动控制Demo，旨在帮助用户轻松、快捷地掌握EM-2030等各型号舵机的使用。强烈推荐结合所使用舵机对应的内存表参数解析来理解和验证。
- 支持的协议：Energize Lab舵机通讯协议
- 支持的内存表：初级内存表
- 支持的舵机：EM系列、EH-3030
- 支持的编程语言：C、C++、Python、MicroPython、Arduino
- 支持的平台或硬件环境：Windows、STC89C52、STM32F103C8T6、Esp32、Arduino Uno

# 支持内存表
## 初级内存表
- 支持的舵机：EM系列、EH-3030
### 支持的编程语言及开发环境
- C：包含基础库，基于Windows、STC89C52、STM32F103C8T6的Example和Demo
- C++：包含基础库，基于Windows的Example和Demo
- Python：包含基础库，基于Windows的Example和Demo
- MicroPython：包含基础库，基于Esp32的Example和Demo
- Arduino：包含基础库，基于Esp32、ArduinoUno的Example和Demo
### 文件说明
- Src：基础库，包含Servo整个内存表的指令生成和解析。
- Example：演示Ping指令、读指令、写指令、同步写指令、恢复参数指令、恢复出厂指令、重启指令的使用
- Demo：演示舵机不同控制模式的使用
- FAQ：基于本项目的FAQ、基于各平台或开发环境的FAQ
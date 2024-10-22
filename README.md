# ServoSDK
Welcome to ServoSDK, an open-source software development kit designed specifically for the Energize Lab Servo. This SDK provides multi-language, multi-platform servo usage examples and motion control demos, aimed at helping users easily and quickly master the use of various models, including the EM-2030. It is strongly recommended to refer to the corresponding memory table parameter for understanding and verification.
- Supported Protocols:
  - Energize Lab Servo Communication Protocol  
- Supported Memory Tables: 
  - Primary Memory Table 
- Supported Servos:  
  - EM Series, EH-3030 
- Supported Programming Languages:
  - Arduino: Includes source code, with compatible examples and demos across multiple platforms.
  - C: Includes source code, with examples and demos based on Windows, STC89C52, and STM32F103C8T6.
  - C++: Includes source code, with examples and demos based on Windows.
  - MicroPython: Includes source code, with examples and demos based on ESP32.
  - Python: Includes source code, with examples and demos based on Windows.
- File Structure: 
  - ServoSDK -> Programming Language -> Src & Development Environment  -> Memory Table (-> Example & Demo)
- File Description: 
  - Src: Source code that includes instruction generation and parsing for the entire memory table of servo.
  - Example: Demonstrate the use of Ping instruction, read data instruction, write data instruction, sync write instruction, parameter reset instruction, factory reset instruction, and reboot instruction.
  - Demo: Demonstrate the motion of the servo in different control modes.

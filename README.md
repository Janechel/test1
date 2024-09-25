# ServoSDK
---
Welcome to ServoSDK, an open-source software development kit designed specifically for the EnergizeLab Servo. This SDK provides multi-language, multi-platform examples and motion control demos for servo usage, aimed at helping users easily and quickly master the use of various models, including the EM-2030. It is strongly recommended to refer to the corresponding memory table parameter for understanding and verification.
- Supported Protocols:Energize Lab Servo Communication Protocol
- Supported Memory Tables:Basic Memory Table
- Supported Servos:EM Series, EH-3030
- Supported Programming Languages:
  - C:Includes basic libraries，Includes basic libraries, with Example and Demo based on Windows, STC89C52, and STM32F103C8T6.
  - C++:Includes basic libraries, with Example and Demo based on Windows.
  - Python:Includes basic libraries, with Example and Demo based on Windows.
  - MicroPython:Includes basic libraries, with Example and Demo based on ESP32.
  - Arduino:Includes basic libraries, with Example and Demo based on ESP32 and Arduino Uno.
- File Structure:ServoSDK -> Memory Table -> Programming Language -> Src & Development Environment (-> Example & Demo)
- File Description:
  - Src:Basic libraries, including instruction generation and parsing for the entire servo memory table.
  - Example:Demonstrates the use of Ping instruction, read data instruction, write data instruction, sync write instruction, parameter reset instruction, factory reset instruction, and reboot instruction.
  - Demo:Demonstrates servo motion in different control modes.
  - FAQ:FAQs related to this project and platform-specific FAQs.



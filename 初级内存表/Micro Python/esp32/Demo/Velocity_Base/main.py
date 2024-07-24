import micropython
from machine import Pin, UART
import time
from servo import *

output_buffer = bytearray(20)  # 存放生成的指令
output_buffer_len = [0]  # 指令长度
receive_data = bytearray(20)  # 存放接收的应答包
receive_data_len = 0  # 接收数据的长度
analysis_data = [0]  # 应答包解析出来的数据

# 配置串口2 (UART2)
uart2 = UART(2, baudrate=1000000, tx=17, rx=16)

# 循环控制舵机
while True:
    # 参数重置
    Servo.servo_parameter_reset(1, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_parameter_reset_analysis(receive_data)
    if ret == State.SUCCESS:
        print("PARAMETER RESET SUCCESS")
    time.sleep(1)

    # 设置舵机的扭矩开关
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 设置舵机的控制模式
    Servo.servo_set_control_mode(1, 1, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # 设置舵机的扭矩开关
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)
    
    # 设置舵机的控速目标位置
    Servo.servo_set_velocity_base_target_position(1, 1500, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set velocity base target position is successful")
    time.sleep(1)


    # 设置舵机的控速目标速度
    Servo.servo_set_velocity_base_target_velocity(1, 3600, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_velocity_base_target_velocity_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_velocity is successful")
    time.sleep(1)

    # 设置舵机的控速目标加速度
    Servo.servo_set_velocity_base_target_acc(1, 10, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_velocity_base_target_acc_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_acc is successful")
    time.sleep(1)

    # 设置舵机的控速目标减速度
    Servo.servo_set_velocity_base_target_dec(1, 1, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_velocity_base_target_dec_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_dec is successful")
    time.sleep(1)

    # 设置舵机的控速目标位置
    Servo.servo_set_velocity_base_target_position(1, 0, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set velocity base target position is successful")
    time.sleep(1)


from servo import *
import serial
import time

MAX_RECEIVE_LEN = 30  # 读取串口最大数据长度

serial = serial.Serial('COM12', 1000000, timeout=0.01)  # 打开指定串口，并设置超时
if serial.isOpen():
    print("open success")
else:
    print("open failed")

output_buffer = [0] * 20  # 存放生成的指令
output_buffer_len = [0]  # 指令长度
receive_data = [0] * 20  # 存放接收的应答包
analysis_data = [0]  # 应答包解析出来的数据

# 参数重置
Servo.servo_parameter_reset(1, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_parameter_reset_analysis(receive_data)
if ret == State.SUCCESS:
    print("PARAMETER RESET SUCCESS")
time.sleep(1)

# 设置舵机的扭矩开关
Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_torque_switch_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_torque_switch is successful")
time.sleep(1)

# 设置舵机的控制模式
Servo.servo_set_control_mode(1, 1, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_control_mode_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_control_mode is successful")
time.sleep(1)

# 设置舵机的扭矩开关
Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_torque_switch_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_torque_switch is successful")
time.sleep(1)

# 设置舵机的控速目标位置
Servo.servo_set_velocity_base_target_position(1, 1500, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo set velocity base target position is successful")
time.sleep(1)

# 设置舵机的控速目标速度
Servo.servo_set_velocity_base_target_velocity(1, 3600, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_velocity_base_target_velocity_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_velocity_base_target_velocity is successful")
time.sleep(1)

# 设置舵机的控速目标加速度
Servo.servo_set_velocity_base_target_acc(1, 10, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_velocity_base_target_acc_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_velocity_base_target_acc is successful")
time.sleep(1)

# 设置舵机的控速目标减速度
Servo.servo_set_velocity_base_target_dec(1, 1, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_velocity_base_target_dec_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_velocity_base_target_dec is successful")
time.sleep(1)

# 设置舵机的控速目标位置
Servo.servo_set_velocity_base_target_position(1, 0, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo set velocity base target position is successful")
time.sleep(1)

# 关闭串口
serial.close()

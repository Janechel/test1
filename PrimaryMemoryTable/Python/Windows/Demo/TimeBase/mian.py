from servo import *
import serial
import time

MAX_RECEIVE_LEN = 30  # 读取串口最大数据长度


serial = serial.Serial('COM16', 1000000, timeout=0.01)  # 打开指定串口，并设置超时
if serial.isOpen():
    print("open success")
else:
    print("open failed")

output_buffer = [0] * 20  # 存放生成的指令
output_buffer_len = [0]  # 指令长度
receive_data = [0] * 20  # 存放接收的应答包
analysis_data = [0]  # 应答包解析出来的数据

servo_sync_parameter = Servo_Sync_Parameter()  # 创建同步写内存表类

# 同步写操作两个舵机
servo_sync_parameter.id_counts = 2

#第一个舵机id为1
servo_sync_parameter.id[0] = 1

#第一个舵机id为2
servo_sync_parameter.id[1] = 2

# 将ID1、ID2舵机的扭矩开关状态，分别修改为关闭
servo_sync_parameter.torque_switch[0] = 0
servo_sync_parameter.torque_switch[1] = 0
Servo.servo_sync_write_torque_switch(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("Sync Write torque witch successfully.\r\n")
time.sleep(1)

# 将ID1、ID2舵机的控制模式，分别修改为控时模式
servo_sync_parameter.control_mode[0] = 0
servo_sync_parameter.control_mode[1] = 0
Servo.servo_sync_write_control_mode(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("Sync Write control mode successfully.\r\n")
time.sleep(1)


# 设置舵机的控时目标位置和目标运行时间
Servo.servo_set_time_base_target_position_and_moving_time(1, 3000, 500, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_time_base_target_position_and_moving_time_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_time_base_target_position_and_moving_time is successful")
time.sleep(1)

# 设置舵机的控时目标位置和目标运行时间
Servo.servo_set_time_base_target_position_and_moving_time(1, 0, 1000, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_time_base_target_position_and_moving_time_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_time_base_target_position_and_moving_time is successful")
time.sleep(1)

# 设置多个舵机的控时目标位置和运动时间

# 在控时模式下，让ID1舵机以500ms运动到150°位置，让ID2舵机以1s匀速运动到0°位置
servo_sync_parameter.position[0] = 1500
servo_sync_parameter.position[1] = 0
servo_sync_parameter.time[0] = 500
servo_sync_parameter.time[1] = 1000

Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("Sync Write time base target position and moving time successfully.\r\n")
time.sleep(1)

# 在控时模式下，让ID1舵机以1000ms运动到0°位置，让ID2舵机以500ms匀速运动到3000°位置
servo_sync_parameter.position[0] = 0
servo_sync_parameter.position[1] = 3000
servo_sync_parameter.time[0] = 1000
servo_sync_parameter.time[1] = 500

Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("Sync Write time base target position and moving time successfully.\r\n")
time.sleep(1)

# 关闭串口
serial.close()

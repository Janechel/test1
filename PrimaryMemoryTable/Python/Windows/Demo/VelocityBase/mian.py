from servo import *
import serial
import time

MAX_RECEIVE_LEN = 30  # 读取串口最大数据长度

serial = serial.Serial('COM16', 1000000, timeout=0.01)  # 打开指定串口，并设置超时
if serial.isOpen():
    print("open success")
else:
    print("open failed")

output_buffer = [0] * 40  # 存放生成的指令
output_buffer_len = [0]  # 指令长度
receive_data = [0] * 20  # 存放接收的应答包
analysis_data = [0]  # 应答包解析出来的数据
write_buffer = [0] * 20    # 写入内存表数据

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

# 将ID1、ID2舵机的控制模式，分别修改为控速模式
servo_sync_parameter.control_mode[0] = 1
servo_sync_parameter.control_mode[1] = 1
Servo.servo_sync_write_control_mode(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("Sync Write control mode successfully.\r\n")
time.sleep(1)

# 设置舵机的控速目标位置
Servo.servo_set_velocity_base_target_position(1, 1500, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo set velocity base target position is successful")
time.sleep(1)

# 在控速模式下，让ID1以360°/s的控速目标速度运动到300°位置
write_buffer[0] = 3000 & 0xff
write_buffer[1] = (3000 >> 8) & 0xff
write_buffer[2] = 3600 & 0xff
write_buffer[3] = (3600 >> 8) & 0xff

Servo.servo_write(1, 0x35, 4, write_buffer, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("servo set pack is:", end=' ')
for i in range(len(receive_data)):
    print(f"0x{receive_data[i]:02x}", end=' ')
print("\r\n")
time.sleep(1)

# 在控速模式下，让ID1舵机以360°/s的控速目标速度、500°/s²的控速目标加速度、50°/s²的控速目标减速度运动到0°位置
write_buffer[0] = 0 & 0xff
write_buffer[1] = (0 >> 8) & 0xff
write_buffer[2] = 3600 & 0xff
write_buffer[3] = (3600 >> 8) & 0xff
write_buffer[4] = 10 & 0xff
write_buffer[5] = 1 & 0xff

Servo.servo_write(1, 0x35, 6, write_buffer, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("servo set pack is:", end=' ')
for i in range(len(receive_data)):
    print(f"0x{receive_data[i]:02x}", end=' ')
print("\r\n")
time.sleep(1)

# 在控速模式下，让ID1舵机运动到150中位°，让ID2舵机运动到0°位置

# id为1，2的舵机运动位置分别设置为1500，0，值和前面的id设置对应
servo_sync_parameter.position[0] = 1500
servo_sync_parameter.position[1] = 0

Servo.servo_sync_write_velocity_base_target_position(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("Sync Write velocity base target position successfully.\r\n")
time.sleep(1)

# 在控速模式下，让ID1以360°/s的控速目标速度运动到300°位置，让ID2以720°/s的控速目标速度运动到150°位置

# id为1，2的舵机速度分别设置为3600，7200，位置分别设置为3000，1500
servo_sync_parameter.velocity[0] = 3600
servo_sync_parameter.velocity[1] = 7200
servo_sync_parameter.position[0] = 3000
servo_sync_parameter.position[1] = 1500

Servo.servo_sync_write_velocity_base_target_position_and_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("Sync Write velocity base target position and velocity successfully.\r\n")
time.sleep(1)

# 在控速模式下，让ID1舵机以720°/s的控速目标速度、500°/s²的控速目标加速度、50°/s²的控速目标减速度运动到0°位置，让ID2舵机以360°/s的控速目标速度、50°/s²的控速目标加速度、500°/s²的控速目标减速度运动到300°位置

# id为1，2的舵机速度分别设置为7200，3600，位置分别设置为0，3000,加速度分别设置为10，1，减速度分别设置为1，10
servo_sync_parameter.velocity[0] = 7200
servo_sync_parameter.velocity[1] = 3600
servo_sync_parameter.position[0] = 0
servo_sync_parameter.position[1] = 3000
servo_sync_parameter.acc_velocity[0] = 10
servo_sync_parameter.acc_velocity[1] = 1
servo_sync_parameter.dec_velocity[0] = 1
servo_sync_parameter.dec_velocity[1] = 10

Servo.servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("Sync Write velocity base target acc,dec,velocity and position successfully.\r\n")
time.sleep(1)

# 关闭串口
serial.close()

from PrimaryServo import *
import serial
import time

MAX_RECEIVE_LEN = 30  # Read the maximum data length from the serial port.

serial = serial.Serial('COM18', 1000000, timeout=0.01)  # Open the specified serial port and set the timeout.
if serial.isOpen():
    print("open successful")
else:
    print("open failed")

ret = 0   # Status Flag
output_buffer = [0] * 40    # Store Generated Instructions
output_buffer_len = [0]    # Instruction Length
receive_data = [0] * 40    # Store the received status packet
analysis_data = [0]    # Data parsed from the status packet
write_buffer = [0] * 20    # Write data to the memory table

servo_sync_parameter = Primary_Servo_Sync_Parameter()  # Create sync write memory table class.

# Sync write two servos
servo_sync_parameter.id_counts = 2

# Set the ID of the first servo to 1
servo_sync_parameter.id[0] = 1

# Set the ID of the second servo to 2
servo_sync_parameter.id[1] = 2

# Change the torque switch of the servo ID1, ID2 to OFF respectively.
servo_sync_parameter.torque_switch[0] = 0
servo_sync_parameter.torque_switch[1] = 0
Primary_Servo.servo_sync_write_torque_switch(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("sync write torque witch successful.")
time.sleep(1)

# Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
servo_sync_parameter.control_mode[0] = 1
servo_sync_parameter.control_mode[1] = 1
Primary_Servo.servo_sync_write_control_mode(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("sync write control mode successful.")
time.sleep(1)

# Change the velocity base target position of servo ID1 to 150°.
Primary_Servo.servo_set_velocity_base_target_position(1, 1500, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Primary_Servo.servo_set_velocity_base_target_position_analysis(receive_data)
if ret == Primary_State.SUCCESS:
    print("set velocity base target position successful")
time.sleep(1)

# In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s.
write_buffer[0] = 3000 & 0xff
write_buffer[1] = (3000 >> 8) & 0xff
write_buffer[2] = 3600 & 0xff
write_buffer[3] = (3600 >> 8) & 0xff

Primary_Servo.servo_write(1, 0x35, 4, write_buffer, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("set pack is:", end=' ')
for i in range(len(receive_data)):
    print(f"0x{receive_data[i]:02x}", end=' ')
print("\r")
time.sleep(1)

# Change the velocity base target position, velocity base target velocity, velocity base target ACC,
# and velocity base target DEC of servo ID1 to 0° position, 360°/s, 500°/s², and 50°/s², respectively.
write_buffer[0] = 0 & 0xff
write_buffer[1] = (0 >> 8) & 0xff
write_buffer[2] = 3600 & 0xff
write_buffer[3] = (3600 >> 8) & 0xff
write_buffer[4] = 10 & 0xff
write_buffer[5] = 1 & 0xff

Primary_Servo.servo_write(1, 0x35, 6, write_buffer, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("set pack is:", end=' ')
for i in range(len(receive_data)):
    print(f"0x{receive_data[i]:02x}", end=' ')
print("\r")
time.sleep(1)

# In velocity base position control mode, let servo ID1 move to the 150° midpoint and let servo ID2 move to the 0° position.
servo_sync_parameter.position[0] = 1500
servo_sync_parameter.position[1] = 0

Primary_Servo.servo_sync_write_velocity_base_target_position(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("sync write velocity base target position successful.")
time.sleep(1)

# In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s,
# and let servo ID2 move to the 150° position at a velocity base target velocity of 720°/s.
servo_sync_parameter.velocity[0] = 3600
servo_sync_parameter.velocity[1] = 7200
servo_sync_parameter.position[0] = 3000
servo_sync_parameter.position[1] = 1500

Primary_Servo.servo_sync_write_velocity_base_target_position_and_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("sync write velocity base target position and velocity successfully.")
time.sleep(1)

# In velocity base position control mode, let servo ID1 move to the 0° position at a velocity base target velocity of 720°/s, a velocity base target ACC of 500°/s², and a velocity base target DEC of 50°/s².
# Let servo ID2 move to the 300° position at a velocity base target velocity of 360°/s, a velocity base target ACC of 50°/s², and a velocity base target DEC of 500°/s².
servo_sync_parameter.velocity[0] = 7200
servo_sync_parameter.velocity[1] = 3600
servo_sync_parameter.position[0] = 0
servo_sync_parameter.position[1] = 3000
servo_sync_parameter.acc_velocity[0] = 10
servo_sync_parameter.acc_velocity[1] = 1
servo_sync_parameter.dec_velocity[0] = 1
servo_sync_parameter.dec_velocity[1] = 10

Primary_Servo.servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("sync write velocity base target acc,dec,velocity and position successfully.")
time.sleep(1)

serial.close()

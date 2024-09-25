import micropython
from machine import Pin, UART
import time
from servo import *

ret = 0   # Status Flag
output_buffer = bytearray(40)  # Store Generated Instructions
output_buffer_len = [0]  # Instruction Length
receive_data = bytearray(40)  # Store the received status packet
receive_data_len = 0  # Length of received data.
analysis_data = [0]  # Data parsed from the status packet

servo_sync_parameter = Servo_Sync_Parameter()  # Create sync write memory table class.

# Sync write two servos
servo_sync_parameter.id_counts = 2

# Set the ID of the first servo to 1
servo_sync_parameter.id[0] = 1

# Set the ID of the second servo to 2
servo_sync_parameter.id[1] = 2

# Configure serial port 2 (UART2).
uart2 = UART(2, baudrate=1000000, tx=17, rx=16)

while True:
    # Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo_sync_parameter.torque_switch[0] = 0
    servo_sync_parameter.torque_switch[1] = 0
    Servo.servo_sync_write_torque_switch(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write torque witch successfully.\r\n")
    time.sleep(1)

    # Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo_sync_parameter.control_mode[0] = 1
    servo_sync_parameter.control_mode[1] = 1
    Servo.servo_sync_write_control_mode(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write control mode successfully.\r\n")
    time.sleep(1)

    # Change the velocity base target position of servo ID1 to 150°.
    Servo.servo_set_velocity_base_target_position(1, 1500, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set velocity base target position is successful")
    time.sleep(1)
    
    # In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s.   
    write_buffer[0] = 3000 & 0xff
    write_buffer[1] = (3000 >> 8) & 0xff
    write_buffer[2] = 3600 & 0xff
    write_buffer[3] = (3600 >> 8) & 0xff

    Servo.servo_write(1, 0x35, 4, write_buffer, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    print("servo pack is:", end=' ')
    for i in range(receive_data_len):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # Change the velocity base target position, velocity base target velocity, velocity base target ACC,
    # and velocity base target DEC of servo ID1 to 0° position, 360°/s, 500°/s², and 50°/s², respectively.
    write_buffer[0] = 0 & 0xff
    write_buffer[1] = (0 >> 8) & 0xff
    write_buffer[2] = 3600 & 0xff
    write_buffer[3] = (3600 >> 8) & 0xff
    write_buffer[4] = 10 & 0xff
    write_buffer[5] = 1 & 0xff

    Servo.servo_write(1, 0x35, 6, write_buffer, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    print("servo set angle limit pack is:", end=' ')
    for i in range(receive_data_len):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # In velocity base position control mode, let servo ID1 move to the 150° midpoint and let servo ID2 move to the 0° position.
    servo_sync_parameter.position[0] = 1500
    servo_sync_parameter.position[1] = 0

    Servo.servo_sync_write_velocity_base_target_position(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target position successfully.\r\n")
    time.sleep(1)

    # In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s,
    # and let servo ID2 move to the 150° position at a velocity base target velocity of 720°/s.
    servo_sync_parameter.velocity[0] = 3600
    servo_sync_parameter.velocity[1] = 7200
    servo_sync_parameter.position[0] = 3000
    servo_sync_parameter.position[1] = 1500

    Servo.servo_sync_write_velocity_base_target_position_and_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target position and velocity successfully.\r\n")
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

    Servo.servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target acc,dec,velocity and position successfully.\r\n")
    time.sleep(1)

    

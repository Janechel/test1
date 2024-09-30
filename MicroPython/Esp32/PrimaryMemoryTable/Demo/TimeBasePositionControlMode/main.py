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

# Configure serial port 2 (UART2).
uart2 = UART(2, baudrate=1000000, tx=17, rx=16)

servo_sync_parameter = Servo_Sync_Parameter()  # Create sync write memory table class.

while True:
    # Sync write two servos
    servo_sync_parameter.id_counts = 2

    # Set the ID of the first servo to 1
    servo_sync_parameter.id[0] = 1

    # Set the ID of the second servo to 2
    servo_sync_parameter.id[1] = 2

    # Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo_sync_parameter.torque_switch[0] = 0
    servo_sync_parameter.torque_switch[1] = 0
    Servo.servo_sync_write_torque_switch(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write torque witch successfully.\r\n")
    time.sleep(1)

    # Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
    servo_sync_parameter.control_mode[0] = 0
    servo_sync_parameter.control_mode[1] = 0
    Servo.servo_sync_write_control_mode(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write control mode successfully.\r\n")
    time.sleep(1)

    
    # Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.
    Servo.servo_set_time_base_target_position_and_moving_time(1, 3000, 500, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    ret = Servo.servo_set_time_base_target_position_and_moving_time_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_time_base_target_position_and_moving_time is successful")
    time.sleep(1)

    # Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
    write_buffer[0] = 0
    write_buffer[1] = 3000 & 0xff
    write_buffer[2] = (3000 >> 8) & 0xff
    write_buffer[3] = 1000 & 0xff
    write_buffer[4] = (1000 >> 8) & 0xff

    Servo.servo_write(1, 0x3B, 5, write_buffer, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep_ms(1)
    receive_data_len = uart2.readinto(receive_data)
    print("servo pack is:", end=' ')
    for i in range(receive_data_len):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # In time base position control mode, let servo ID1 move to the 150° position at a velocity of 500ms,
    # and let servo ID2 move to the 0° position at a constant velocity of 1s.
    servo_sync_parameter.position[0] = 1500
    servo_sync_parameter.position[1] = 0
    servo_sync_parameter.time[0] = 500
    servo_sync_parameter.time[1] = 1000

    Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write time base target position and moving time successfully.\r\n")
    time.sleep(1)
    
    # In time base position control mode, let servo ID1 move to the 0° position at a velocity of 1s,
    # and let servo ID2 move to the 3000° position at a constant velocity of 500ms.
    servo_sync_parameter.position[0] = 0
    servo_sync_parameter.position[1] = 3000
    servo_sync_parameter.time[0] = 1000
    servo_sync_parameter.time[1] = 500

    Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
    uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write time base target position and moving time successfully.\r\n")
    time.sleep(1)

        

  

    




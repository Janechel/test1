from servo import *
import serial
import time

MAX_RECEIVE_LEN = 30  # Read the maximum data length from the serial port.

serial = serial.Serial('COM12', 1000000, timeout=0.01)  # Open the specified serial port and set the timeout.
if serial.isOpen():
    print("open success")
else:
    print("open failed")

ret = 0   # Status Flag
output_buffer = [0] * 40    # Store Generated Instructions
output_buffer_len = [0]    # Instruction Length
receive_data = [0] * 40    # Store the received status packet
analysis_data = [0]    # Data parsed from the status packet

# Change the torque switch of servo ID1 to OFF.
Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_torque_switch_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_torque_switch is successful")
time.sleep(1)

# Change the control mode of servo ID1 to the current control mode.
Servo.servo_set_control_mode(1, 2, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_control_mode_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_control_mode is successful")
time.sleep(1)

# Change the torque switch of servo ID1 to ON.
Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_torque_switch_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_torque_switch is successful")
time.sleep(1)

# Change the target PWM of servo ID1 to 100mA.
Servo.servo_set_target_current(1, 100, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
if ret == State.SUCCESS:
    print("servo_set_target_current is successful")
time.sleep(3)

serial.close()

from servo import *
import serial
import time

PING_TEST = 0  # PING指令测试
READ_TEST = 0  # 读取舵机数据测试
FACTORY_RESET_TEST = 0  # 恢复出厂设置测试
PARAMETER_RESET_TEST = 0  # 参数重置测试
CALIBRATION_TEST = 0  # 校正偏移值测试
REBOOT_TEST = 0  # 重启测试
WRITE_TEST = 0  # 写入舵机数据测试
SYNC_WRITE = 0  # 同步写测试
MODIFY_ID = 0  # 修改已知舵机ID测试
MODIFY_UNKNOWN_ID = 0  # 修改未知ID舵机ID测试

MAX_RECEIVE_LEN = 30  # 读取串口最大数据长度

serial = serial.Serial('COM12', 1000000, timeout=0.01)  # 打开指定串口，并设置超时
if serial.isOpen():
    print("open success")
else:
    print("open failed")

ret = 0   # 错误检验标志
output_buffer = [0] * 40    # 存放生成的指令
output_buffer_len = [0]    # 指令长度
receive_data = [0] * 40    # 存放接收的应答包
analysis_data = [0]    # 应答包解析出来的数据
position = [0]    # 当前位置
current = [0]    # 当前电流
write_buffer = [0] * 20    # 写入内存表数据

servo_sync_parameter = Servo_Sync_Parameter()  # 创建同步写内存表类

# 恢复出厂设置
if FACTORY_RESET_TEST:
    Servo.servo_factory_reset(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_factory_reset_analysis(receive_data)
    if ret == State.SUCCESS:
        print("FACTORY RESET SUCCESS")
    time.sleep(1)

# 参数重置
if PARAMETER_RESET_TEST:
    Servo.servo_parameter_reset(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_parameter_reset_analysis(receive_data)
    if ret == State.SUCCESS:
        print("PARAMETER RESET SUCCESS")
    time.sleep(1)

# 校正偏移值
if CALIBRATION_TEST:
    Servo.servo_calibration(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_calibration_analysis(receive_data)
    if ret == State.SUCCESS:
        print("CALIBRATION SUCCESS")
    time.sleep(1)

# 重启舵机
if REBOOT_TEST:
    Servo.servo_reboot(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

# 将舵机ID1改为2
if MODIFY_ID:
    Servo.servo_modify_known_id(1, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

# 将未知ID舵机的ID编号修改为1
if MODIFY_UNKNOWN_ID:
    Servo.servo_modify_unknown_id(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

# 向id为1的舵机发出ping指令
if PING_TEST:
    Servo.servo_ping(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_ping_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the servo model number is", analysis_data[0])
    time.sleep(1)

# 读取舵机数据
if READ_TEST:
    # 读取ID1舵机的当前电流
    Servo.servo_read_present_current(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_current_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present current is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的当前位置
    Servo.servo_read_present_position(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_position_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present position is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的当前位置和当前电流
    Servo.servo_read_present_position_and_present_current(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_position_and_present_current_analysis(receive_data, position, current)
    if ret == State.SUCCESS:
        print(f"present position is: {position[0]}, present current is: {current[0]}")
    time.sleep(1)

    # 读取ID1舵机的当前速度
    Servo.servo_read_present_velocity(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_velocity_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的当前的规划位置
    Servo.servo_read_present_profile_position(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_profile_position_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present profile position is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的当前规划速度
    Servo.servo_read_present_profile_velocity(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_profile_velocity_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present profile velocity is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的当前PWM
    Servo.servo_read_present_pwm(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_pwm_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present pwm is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的当前温度
    Servo.servo_read_present_temperature(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_temperature_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present temperature is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的当前输入电压
    Servo.servo_read_present_voltage(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_voltage_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present voltage is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控时目标运行时间
    Servo.servo_read_time_base_target_moving_time(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_time_base_target_moving_time_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present time base target moving time is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控时目标位置
    Servo.servo_read_time_base_target_position(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_time_base_target_position_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present time base target position is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控时加速度等级
    Servo.servo_read_time_base_target_acc(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_time_base_target_acc_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present time base target acc is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控时目标位置和运行时间
    Servo.servo_read(1, 0x3C, 4, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("the time base target position and moving time pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # 读取ID1舵机的控时目标加速度等级、位置和运行时间
    Servo.servo_read(1, 0x3B, 5, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("the time base target acc, position and moving time pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # 读取ID1舵机的控速目标减速度
    Servo.servo_read_velocity_base_target_dec(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_velocity_base_target_dec_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity base target dec is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控速目标加速度
    Servo.servo_read_velocity_base_target_acc(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_velocity_base_target_acc_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity base target acc is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控速目标速度
    Servo.servo_read_velocity_base_target_velocity(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_velocity_base_target_velocity_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity base target velocity is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控速目标位置
    Servo.servo_read_velocity_base_target_position(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_velocity_base_target_position_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity base target position is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控速目标位置和速度
    Servo.servo_read(1, 0x35, 4, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("the velocity base target position and velocity pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # 读取ID1舵机的控速目标位置、速度和加减速度
    Servo.servo_read(1, 0x35, 6, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("the velocity base target position,velocity,acc and dec pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # 读取ID1舵机的目标电流
    Servo.servo_read_target_current(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_target_current_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present target current is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的目标PWM
    Servo.servo_read_target_pwm(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_target_pwm_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present target pwm is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的扭矩开关
    Servo.servo_read_torque_switch(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_torque_switch_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present torque switch is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的LED开关
    Servo.servo_read_led_switch(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_led_switch_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present led switch is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的Flash开关
    Servo.servo_read_flash_switch(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_flash_switch_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present flash switch is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的电流校正值
    Servo.servo_read_current_offset(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_current_offset_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present current offset is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的中位校正值
    Servo.servo_read_calibration(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_calibration_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present calibration is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的控制模式
    Servo.servo_read_control_mode(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_control_mode_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present control mode is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的卸载保护条件
    Servo.servo_read_shutdown_condition(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_shutdown_condition_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present shutdown condition is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的LED报警条件
    Servo.servo_read_led_condition(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_led_condition_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present led condition is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的位置控制D增益
    Servo.servo_read_position_control_d_gain(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_position_control_d_gain_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present position control d gain is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的位置控制I增益
    Servo.servo_read_position_control_i_gain(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_position_control_i_gain_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present position control I gain is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的位置控制P增益
    Servo.servo_read_position_control_p_gain(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_position_control_p_gain_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present position control P gain is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的位置控制PID增益
    Servo.servo_read(1, 0x1B, 6, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("position control pid gain pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # 读取ID1舵机的PWM叠加值
    Servo.servo_read_pwm_punch(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_pwm_punch_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present pwm punch is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的反转死区
    Servo.servo_read_ccw_deadband(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_ccw_deadband_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present ccw deadband is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的正转死区
    Servo.servo_read_cw_deadband(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_cw_deadband_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present cw deadband is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的电流保护时间
    Servo.servo_read_current_shutdown_time(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_current_shutdown_time_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present current shutdown time is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的电流上限
    Servo.servo_read_max_current_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_current_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max current limit is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的PWM上限
    Servo.servo_read_max_pwm_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_pwm_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max pwm limit is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的电压上限
    Servo.servo_read_max_voltage_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_voltage_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max voltage limit is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的电压下限
    Servo.servo_read_min_voltage_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_min_voltage_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present min voltage limit is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的电压限制
    Servo.servo_read(1, 0x10, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("the voltage limit pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # 读取ID1舵机的温度上限
    Servo.servo_read_max_temperature_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_temperature_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max temperature limit is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的最大位置限制
    Servo.servo_read_max_angle_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_angle_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max angle limit is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的最小位置限制
    Servo.servo_read_min_angle_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_min_angle_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present min angle limit is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的位置限制
    Servo.servo_read(1, 0x0B, 4, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("the angle limit pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # 读取ID1舵机的状态返回级别
    Servo.servo_read_return_level(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_return_level_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present return level is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的应答延时时间
    Servo.servo_read_return_delay_time(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_return_delay_time_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present return delay time is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的波特率
    Servo.servo_read_baud_rate(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_baud_rate_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present baud rate is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的出厂编号
    Servo.servo_read_model_information(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_model_information_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present model information is", analysis_data[0])
    time.sleep(1)

    # 读取ID1舵机的固件版本号
    Servo.servo_read_firmware_version(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_firmware_version_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present firmware version is", analysis_data[0])
    time.sleep(1)

    #
# 写入舵机数据
if WRITE_TEST:
    # 将ID1舵机的应答延迟时间修改为500us
    Servo.servo_set_return_delay_time(1, 250, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_return_delay_time_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set return delay time is successful")
    time.sleep(1)
    
    # 将ID1舵机的状态返回级别修改为应答所有指令
    Servo.servo_set_return_level(1, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_return_level_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set return level is successful")
    time.sleep(1)
    
    # 设置ID1舵机的波特率为1000000
    Servo.servo_set_baud_rate(1, 7, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_baud_rate_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set baud rate is successful")
    time.sleep(1)
    
    # 将舵机ID1的最小位置限制修改为0°
    Servo.servo_set_min_angle_limit(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_min_angle_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set min angle limit is successful")
    time.sleep(1)
    
    # 将舵机ID1的最大位置限制修改为300°
    Servo.servo_set_max_angle_limit(1, 3000, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_angle_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set max angle limit is successful")
    time.sleep(1)

    # 将舵机ID1的位置限制修改为0°~300°
    write_buffer[0] = 0 & 0xff
    write_buffer[1] = (0 >> 8) & 0xff
    write_buffer[2] = 3000 & 0xff
    write_buffer[3] = (3000 >> 8) & 0xff

    Servo.servo_write(1, 0x0B, 4, write_buffer, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("servo set angle limit pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)
    
    # 将ID1舵机的温度上限修改为65℃
    Servo.servo_set_max_temperature_limit(1, 65, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_temperature_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set max temperature limit is successful")
    time.sleep(1)
    
    # 将ID1舵机的电压上限修改为8.4V
    Servo.servo_set_max_voltage_limit(1, 84, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_voltage_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_max_voltage_limit is successful")
    time.sleep(1)
    
    # 将ID1舵机的电压下限修改为3.5V
    Servo.servo_set_min_voltage_limit(1, 35, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_min_voltage_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_min_voltage_limit is successful")
    time.sleep(1)

    # 将ID1舵机的电压限制修改为3.5V~8.4V
    write_buffer[0] = 84 & 0xff
    write_buffer[1] = 35 & 0xff

    Servo.servo_write(1, 0x10, 2, write_buffer, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("the voltage limit pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)
    
    # 将ID1舵机的PWM上限修改为90%
    Servo.servo_set_max_pwm_limit(1, 900, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_pwm_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_max_pwm_limit is successful")
    time.sleep(1)
    
    # 将ID1舵机的电流上限修改为900mA
    Servo.servo_set_max_current_limit(1, 900, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_current_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_max_current_limit is successful")
    time.sleep(1)
    
    # 将ID1舵机的电流保护时间修改为500ms
    Servo.servo_set_current_shutdown_time(1, 500, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_current_shutdown_time_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_current_shutdown_time is successful")
    time.sleep(1)
    
    # 将ID1舵机的正转死区修改为0.2°
    Servo.servo_set_cw_deadband(1, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_cw_deadband_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_cw_deadband is successful")
    time.sleep(1)
    
    # 将ID1舵机的反转死区修改为0.2°
    Servo.servo_set_ccw_deadband(1, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_ccw_deadband_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_ccw_deadband is successful")
    time.sleep(1)

    # 将ID1舵机的正反转死区修改为0.2°
    write_buffer[0] = 2 & 0xff
    write_buffer[1] = 2 & 0xff

    Servo.servo_write(1, 0x18, 2, write_buffer, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("servo set cw deadband and ccw deadband pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    
    # 将ID1舵机的PWM叠加值修改为1%
    Servo.servo_set_pwm_punch(1, 10, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_pwm_punch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_pwm_punch is successful")
    time.sleep(1)
    
    # 将ID1舵机的位置控制P增益修改为5995
    Servo.servo_set_position_control_p_gain(1, 5995, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_position_control_p_gain_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_position_control_p_gain is successful")
    time.sleep(1)
    
    # 将ID1舵机的位置控制I增益修改为5
    Servo.servo_set_position_control_i_gain(1, 5, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_position_control_i_gain_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_position_control_i_gain is successful")
    time.sleep(1)
    
    # 将ID1舵机的位置控制D增益修改为145
    Servo.servo_set_position_control_d_gain(1, 145, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_position_control_d_gain_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_position_control_d_gain is successful")
    time.sleep(1)

    # 将ID1舵机的位置控制PID增益，分别修改为5500、100、250
    write_buffer[0] = 5500 & 0xff
    write_buffer[1] = (5500 >> 8) & 0xff
    write_buffer[2] = 100 & 0xff
    write_buffer[3] = (100 >> 8) & 0xff
    write_buffer[4] = 250 & 0xff
    write_buffer[5] = (250 >> 8) & 0xff

    Servo.servo_write(1, 0x1B, 6, write_buffer, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    print("servo set position control pid gain pack is:", end=' ')
    for i in range(len(receive_data)):
        print(f"0x{receive_data[i]:02x}", end=' ')
    print("\r\n")
    time.sleep(1)

    # 将ID1舵机的LED报警条件修改为开启堵转报错、过热报错和角度报错
    Servo.servo_set_led_condition(1, 38, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_led_condition_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_led_condition is successful")
    time.sleep(1)

    # 将ID1舵机的卸载保护条件修改为开启堵转报错、过热报错、电压报错和角度报错
    Servo.servo_set_shutdown_conditions(1, 39, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_shutdown_conditions_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_shutdown_conditions is successful")
    time.sleep(1)

    # 将ID1舵机的Flash开关状态修改为打开
    Servo.servo_set_flash_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_flash_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_flash_switch is successful")
    time.sleep(1)

    # 将ID1舵机的Flash开关状态修改为关闭
    Servo.servo_set_flash_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_flash_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_flash_switch is successful")
    time.sleep(1)

    # 将ID1舵机的LED开关状态修改为打开
    Servo.servo_set_led_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_led_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_led_switch is successful")
    time.sleep(1)
    
    # 将ID1舵机的LED开关状态修改为关闭
    Servo.servo_set_led_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_led_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_led_switch is successful")
    time.sleep(1)

    # 设置ID1舵机的扭矩开关为关闭
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 将ID1舵机的控制模式修改为PWM输出控制模式
    Servo.servo_set_control_mode(1, 3, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # 设置ID1舵机的扭矩开关为开启
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 将ID1舵机的目标PWM修改为-50%
    Servo.servo_set_target_pwm(1, -500, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_target_pwm_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_target_pwm is successful")
    time.sleep(3)

    # 设置ID1舵机的扭矩开关为关闭
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 将ID1舵机的控制模式修改为电流控制模式
    Servo.servo_set_control_mode(1, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # 设置ID1舵机的扭矩开关为开启
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 将ID1舵机的目标电流修改为-400mA
    Servo.servo_set_target_current(1, -400, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_target_current is successful")
    time.sleep(3)

    # 设置ID1舵机的扭矩开关为关闭
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 将ID1舵机的控制模式修改为控速模式
    Servo.servo_set_control_mode(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # 设置ID1舵机的扭矩开关为开启
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 将ID1舵机的控速目标速度修改为360°/s
    Servo.servo_set_velocity_base_target_velocity(1, 3600, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_velocity_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_velocity is successful")
    time.sleep(1)

    # 将ID1舵机的控速目标加速度修改为500°/s²
    Servo.servo_set_velocity_base_target_acc(1, 10, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_acc_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_acc is successful")
    time.sleep(1)

    # 将ID1舵机的控速目标减速度修改为50°/s²
    Servo.servo_set_velocity_base_target_dec(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_dec_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_dec is successful")
    time.sleep(1)

    # 将ID1舵机的控速目标位置修改为150°
    Servo.servo_set_velocity_base_target_position(1, 1500, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set velocity base target position is successful")
    time.sleep(1)

    # 设置ID1舵机的扭矩开关为关闭
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 将ID1舵机的控制模式修改为控时模式
    Servo.servo_set_control_mode(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # 设置ID1舵机的扭矩开关为开启
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # 将ID1舵机的控时目标加速度等级修改为5
    Servo.servo_set_time_base_target_acc(1, 5, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_time_base_target_acc_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_time_base_target_acc is successful")
    time.sleep(1)

    # 将ID1舵机的控时目标位置和运行时间，分别修改为300°、500ms
    Servo.servo_set_time_base_target_position_and_moving_time(1, 3000, 500, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_time_base_target_position_and_moving_time_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_time_base_target_position_and_moving_time is successful")
    time.sleep(1)

# 同步写测试
if SYNC_WRITE:
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

    # 设置多个舵机的控速目标加速度

    # id为1，2的舵机加速度分别设置为150，150，值和前面的id设置对应
    servo_sync_parameter.acc_velocity[0] = 150      
    servo_sync_parameter.acc_velocity[1] = 150

    Servo.servo_sync_write_velocity_base_target_acc(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target acc successfully.\r\n")
    time.sleep(1)

    # 设置多个舵机的控速目标减速度

    # id为1，2的舵机减速度分别设置为150，150，值和前面的id设置对应
    servo_sync_parameter.dec_velocity[0] = 150        
    servo_sync_parameter.dec_velocity[1] = 150

    Servo.servo_sync_write_velocity_base_target_dec(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target dec successfully.\r\n")
    time.sleep(1)

    # 设置多个舵机的控速目标速度

    # id为1，2的舵机速度分别设置为3600，1800，值和前面的id设置对应
    servo_sync_parameter.velocity[0] = 3600
    servo_sync_parameter.velocity[1] = 1800

    Servo.servo_sync_write_velocity_base_target_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target velocity successfully.\r\n")
    time.sleep(1)

    # 设置多个舵机的控速目标位置

    # id为1，2的舵机运动位置分别设置为0，0，值和前面的id设置对应
    servo_sync_parameter.position[0] = 0
    servo_sync_parameter.position[1] = 0

    Servo.servo_sync_write_velocity_base_target_position(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target position successfully.\r\n")
    time.sleep(1)

    # 设置多个舵机的控速目标位置和速度

    # id为1，2的舵机速度分别设置为1800，3600，位置分别设置为3000，3000
    servo_sync_parameter.velocity[0] = 1800
    servo_sync_parameter.velocity[1] = 3600
    servo_sync_parameter.position[0] = 3000
    servo_sync_parameter.position[1] = 3000

    Servo.servo_sync_write_velocity_base_target_position_and_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target position and velocity successfully.\r\n")
    time.sleep(1)

    # 设置多个舵机的加速度，减速度，速度和位置

    # id为1，2的舵机速度分别设置为3600，3600，位置分别设置为0，0,加速度分别设置为100，100，减速度分别设置为100，100
    servo_sync_parameter.velocity[0] = 3600
    servo_sync_parameter.velocity[1] = 3600
    servo_sync_parameter.position[0] = 0
    servo_sync_parameter.position[1] = 0
    servo_sync_parameter.acc_velocity[0] = 100
    servo_sync_parameter.acc_velocity[1] = 100
    servo_sync_parameter.dec_velocity[0] = 100
    servo_sync_parameter.dec_velocity[1] = 100

    Servo.servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write velocity base target acc,dec,velocity and position successfully.\r\n")
    time.sleep(1)

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

    # 设置多个舵机的控时目标加速度等级

    # 设置舵机id为1，2的加速度等级分别为0，0
    servo_sync_parameter.acc_velocity_grade[0] = 0
    servo_sync_parameter.acc_velocity_grade[1] = 0

    Servo.servo_sync_write_time_base_target_acc(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write time base target acc successfully.\r\n")
    time.sleep(1)

    # 设置多个舵机的控时目标位置和运动时间

    # 设置舵机id为1，2的运动位置为3000，3000，运动时间为500ms，1500ms
    servo_sync_parameter.position[0] = 3000
    servo_sync_parameter.position[1] = 3000
    servo_sync_parameter.time[0] = 500
    servo_sync_parameter.time[1] = 1500

    Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    print("Sync Write time base target position and moving time successfully.\r\n")
    time.sleep(1)

# 关闭串口
serial.close()

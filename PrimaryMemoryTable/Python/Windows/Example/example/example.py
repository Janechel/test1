from servo import *
import serial
import time

PING_TEST = 0  # PINGָ�����
READ_TEST = 0  # ��ȡ������ݲ���
FACTORY_RESET_TEST = 0  # �ָ��������ò���
PARAMETER_RESET_TEST = 0  # �������ò���
CALIBRATION_TEST = 0  # У��ƫ��ֵ����
REBOOT_TEST = 0  # ��������
WRITE_TEST = 0  # д�������ݲ���
SYNC_WRITE = 0  # ͬ��д����
MODIFY_ID = 0  # �޸Ķ��ID����
MODIFY_UNKNOWN_ID = 0  # �޸�δ֪ID���ID����

MAX_RECEIVE_LEN = 30  # ��ȡ����������ݳ���

serial = serial.Serial('COM12', 1000000, timeout=0.01)  # ��ָ�����ڣ������ó�ʱ
if serial.isOpen():
    print("open success")
else:
    print("open failed")

output_buffer = [0] * 40  # ������ɵ�ָ��
output_buffer_len = [0]  # ָ���
receive_data = [0] * 40  # ��Ž��յ�Ӧ���
analysis_data = [0]  # Ӧ�����������������
servo_sync_parameter = Servo_Sync_Parameter()  # ����ͬ��д�ڴ����

# �ָ���������
if FACTORY_RESET_TEST:
    Servo.servo_factory_reset(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_factory_reset_analysis(receive_data)
    if ret == State.SUCCESS:
        print("FACTORY RESET SUCCESS")
    time.sleep(1)

# ��������
if PARAMETER_RESET_TEST:
    Servo.servo_parameter_reset(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_parameter_reset_analysis(receive_data)
    if ret == State.SUCCESS:
        print("PARAMETER RESET SUCCESS")
    time.sleep(1)

# У��ƫ��ֵ
if CALIBRATION_TEST:
    Servo.servo_calibration(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_calibration_analysis(receive_data)
    if ret == State.SUCCESS:
        print("CALIBRATION SUCCESS")
    time.sleep(1)

# �������
if REBOOT_TEST:
    Servo.servo_reboot(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

# �����ID1��Ϊ2
if MODIFY_ID:
    Servo.servo_modify_known_id(1, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

# ��δ֪ID���ID��Ϊ2
if MODIFY_ID:
    Servo.servo_modify_unknown_id(2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

# ��idΪ1�Ķ������pingָ��
if PING_TEST:
    Servo.servo_ping(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_ping_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the servo model number is", analysis_data[0])
    time.sleep(1)

# ��ȡ�������
if READ_TEST:
    # ��ȡID1����ĵ�ǰ����
    Servo.servo_read_present_current(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_current_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present current is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ǰλ��
    Servo.servo_read_present_position(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_position_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present position is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ǰ�ٶ�
    Servo.servo_read_present_velocity(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_velocity_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ǰ�Ĺ滮λ��
    Servo.servo_read_present_profile_position(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_profile_position_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present profile position is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ǰ�滮�ٶ�
    Servo.servo_read_present_profile_velocity(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_profile_velocity_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present profile velocity is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ǰPWM
    Servo.servo_read_present_pwm(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_pwm_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present pwm is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ǰ�¶�
    Servo.servo_read_present_temperature(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_temperature_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present temperature is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ǰ�����ѹ
    Servo.servo_read_present_voltage(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_present_voltage_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present voltage is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ŀ�ʱĿ������ʱ��
    Servo.servo_read_time_base_target_moving_time(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_time_base_target_moving_time_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present time base target moving time is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ŀ�ʱĿ��λ��
    Servo.servo_read_time_base_target_position(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_time_base_target_position_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present time base target position is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ŀ�ʱ���ٶȵȼ�
    Servo.servo_read_time_base_target_acc(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_time_base_target_acc_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present time base target acc is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ŀ���Ŀ����ٶ�
    Servo.servo_read_velocity_base_target_dec(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_velocity_base_target_dec_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity base target dec is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ŀ���Ŀ����ٶ�
    Servo.servo_read_velocity_base_target_acc(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_velocity_base_target_acc_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity base target acc is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ŀ���Ŀ���ٶ�
    Servo.servo_read_velocity_base_target_velocity(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_velocity_base_target_velocity_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity base target velocity is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ŀ���Ŀ��λ��
    Servo.servo_read_velocity_base_target_position(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_velocity_base_target_position_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present velocity base target position is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����Ŀ�����
    Servo.servo_read_target_current(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_target_current_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present target current is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����Ŀ��PWM
    Servo.servo_read_target_pwm(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_target_pwm_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present target pwm is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����Ť�ؿ���
    Servo.servo_read_torque_switch(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_torque_switch_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present torque switch is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����LED����
    Servo.servo_read_led_switch(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_led_switch_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present led switch is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����Flash����
    Servo.servo_read_flash_switch(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_flash_switch_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present flash switch is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ���У��ֵ
    Servo.servo_read_current_offset(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_current_offset_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present current offset is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�������λУ��ֵ
    Servo.servo_read_calibration(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_calibration_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present calibration is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ŀ���ģʽ
    Servo.servo_read_control_mode(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_control_mode_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present control mode is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����ж�ر�������
    Servo.servo_read_shutdown_condition(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_shutdown_condition_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present shutdown condition is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����LED��������
    Servo.servo_read_led_condition(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_led_condition_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present led condition is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����λ�ÿ���D����
    Servo.servo_read_position_control_d_gain(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_position_control_d_gain_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present position control d gain is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����λ�ÿ���I����
    Servo.servo_read_position_control_i_gain(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_position_control_i_gain_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present position control I gain is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����λ�ÿ���P����
    Servo.servo_read_position_control_p_gain(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_position_control_p_gain_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present position control P gain is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����PWM����ֵ
    Servo.servo_read_pwm_punch(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_pwm_punch_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present pwm punch is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ķ�ת����
    Servo.servo_read_ccw_deadband(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_ccw_deadband_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present ccw deadband is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�������ת����
    Servo.servo_read_cw_deadband(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_cw_deadband_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present cw deadband is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�������ʱ��
    Servo.servo_read_current_shutdown_time(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_current_shutdown_time_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present current shutdown time is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�������
    Servo.servo_read_max_current_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_current_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max current limit is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����PWM����
    Servo.servo_read_max_pwm_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_pwm_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max pwm limit is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ѹ����
    Servo.servo_read_max_voltage_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_voltage_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max voltage limit is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĵ�ѹ����
    Servo.servo_read_min_voltage_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_min_voltage_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present min voltage limit is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1������¶�����
    Servo.servo_read_max_temperature_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_temperature_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max temperature limit is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1��������λ������
    Servo.servo_read_max_angle_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_max_angle_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present max angle limit is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�������Сλ������
    Servo.servo_read_min_angle_limit(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_min_angle_limit_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present min angle limit is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����״̬���ؼ���
    Servo.servo_read_return_level(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_return_level_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present return level is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1�����Ӧ����ʱʱ��
    Servo.servo_read_return_delay_time(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_return_delay_time_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present return delay time is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ĳ�����
    Servo.servo_read_baud_rate(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_baud_rate_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present baud rate is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����ĳ������
    Servo.servo_read_model_information(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_model_information_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present model information is", analysis_data[0])
    time.sleep(1)

    # ��ȡID1����Ĺ̼��汾��
    Servo.servo_read_firmware_version(1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_read_firmware_version_analysis(receive_data, analysis_data)
    if ret == State.SUCCESS:
        print("the present firmware version is", analysis_data[0])
    time.sleep(1)

    #
# д��������
if WRITE_TEST:
    # ����ID1�����Ӧ����ʱʱ��
    Servo.servo_set_return_delay_time(1, 250, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_return_delay_time_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set return delay time is successful")
    time.sleep(1)

    # ����ID1�����״̬���ؼ���
    Servo.servo_set_return_level(1, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_return_level_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set return level is successful")
    time.sleep(1)

    # ����ID1����Ĳ�����
    Servo.servo_set_baud_rate(1, 7, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_baud_rate_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set baud rate is successful")
    time.sleep(1)

    # ����ID1�������Сλ������
    Servo.servo_set_min_angle_limit(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_min_angle_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set min angle limit is successful")
    time.sleep(1)

    # ����ID1��������λ������
    Servo.servo_set_max_angle_limit(1, 3000, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_angle_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set max angle limit is successful")
    time.sleep(1)

    # ����ID1������¶�����
    Servo.servo_set_max_temperature_limit(1, 100, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_temperature_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set max temperature limit is successful")
    time.sleep(1)

    # ����ID1����ĵ�ѹ����
    Servo.servo_set_max_voltage_limit(1, 90, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_voltage_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_max_voltage_limit is successful")
    time.sleep(1)

    # ����ID1����ĵ�ѹ����
    Servo.servo_set_min_voltage_limit(1, 33, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_min_voltage_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_min_voltage_limit is successful")
    time.sleep(1)

    # ����ID1�����PWM����
    Servo.servo_set_max_pwm_limit(1, 1000, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_pwm_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_max_pwm_limit is successful")
    time.sleep(1)

    # ����ID1����ĵ�������
    Servo.servo_set_max_current_limit(1, 400, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_max_current_limit_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_max_current_limit is successful")
    time.sleep(1)

    # ����ID1����ĵ�������ʱ��
    Servo.servo_set_current_shutdown_time(1, 1000, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_current_shutdown_time_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_current_shutdown_time is successful")
    time.sleep(1)

    # ����ID1�������ת����
    Servo.servo_set_cw_deadband(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_cw_deadband_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_cw_deadband is successful")
    time.sleep(1)

    # ����ID1����ķ�ת����
    Servo.servo_set_ccw_deadband(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_ccw_deadband_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_ccw_deadband is successful")
    time.sleep(1)

    # ����ID1�����PWM����ֵ
    Servo.servo_set_pwm_punch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_pwm_punch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_pwm_punch is successful")
    time.sleep(1)

    # ����ID1�����λ�ÿ���P����
    Servo.servo_set_position_control_p_gain(1, 6000, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_position_control_p_gain_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_position_control_p_gain is successful")
    time.sleep(1)

    # ����ID1�����λ�ÿ���I����
    Servo.servo_set_position_control_i_gain(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_position_control_i_gain_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_position_control_i_gain is successful")
    time.sleep(1)

    # ����ID1�����λ�ÿ���D����
    Servo.servo_set_position_control_d_gain(1, 150, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_position_control_d_gain_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_position_control_d_gain is successful")
    time.sleep(1)

    # ����ID1�����LED��������
    Servo.servo_set_led_condition(1, 36, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_led_condition_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_led_condition is successful")
    time.sleep(1)

    # ����ID1�����ж�ر�������
    Servo.servo_set_shutdown_conditions(1, 36, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_shutdown_conditions_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_shutdown_conditions is successful")
    time.sleep(1)

    # ����ID1�����Flash����
    Servo.servo_set_flash_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_flash_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_flash_switch is successful")
    time.sleep(1)

    # ����ID1�����LED����
    Servo.servo_set_led_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_led_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_led_switch is successful")
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1����Ŀ���ģʽ
    Servo.servo_set_control_mode(1, 3, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1�����Ŀ��PWM
    Servo.servo_set_target_pwm(1, 1000, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_target_pwm_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_target_pwm is successful")
    time.sleep(3)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1����Ŀ���ģʽ
    Servo.servo_set_control_mode(1, 2, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1�����Ŀ�����
    Servo.servo_set_target_current(1, -1000, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_target_current is successful")
    time.sleep(3)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1����Ŀ���ģʽ
    Servo.servo_set_control_mode(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1����Ŀ���Ŀ���ٶ�
    Servo.servo_set_velocity_base_target_velocity(1, 3600, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_velocity_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_velocity is successful")
    time.sleep(1)

    # ����ID1����Ŀ���Ŀ����ٶ�
    Servo.servo_set_velocity_base_target_acc(1, 150, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_acc_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_acc is successful")
    time.sleep(1)

    # ����ID1����Ŀ���Ŀ����ٶ�
    Servo.servo_set_velocity_base_target_dec(1, 150, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_dec_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_velocity_base_target_dec is successful")
    time.sleep(1)

    # ����ID1����Ŀ���Ŀ��λ��
    Servo.servo_set_velocity_base_target_position(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_velocity_base_target_position_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo set velocity base target position is successful")
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1����Ŀ���ģʽ
    Servo.servo_set_control_mode(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1����Ŀ�ʱĿ����ٶȵȼ�
    Servo.servo_set_time_base_target_acc(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_time_base_target_acc_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_time_base_target_acc is successful")
    time.sleep(1)

    # ����ID1����Ŀ�ʱĿ��λ�ú�Ŀ������ʱ��
    Servo.servo_set_time_base_target_position_and_moving_time(1, 3000, 500, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_time_base_target_position_and_moving_time_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_time_base_target_position_and_moving_time is successful")
    time.sleep(1)

# ͬ��д����
if SYNC_WRITE:
    # ͬ��д�����������
    servo_sync_parameter.id_counts = 2

    #��һ�����idΪ1
    servo_sync_parameter.id[0] = 1

    #��һ�����idΪ2
    servo_sync_parameter.id[1] = 2

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1����Ŀ���ģʽ
    Servo.servo_set_control_mode(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID2�����Ť�ؿ���
    Servo.servo_set_torque_switch(2, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID2����Ŀ���ģʽ
    Servo.servo_set_control_mode(2, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # ����ID2�����Ť�ؿ���
    Servo.servo_set_torque_switch(2, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ���ö������Ŀ���Ŀ����ٶ�

    # idΪ1��2�Ķ�����ٶȷֱ�����Ϊ150��150��ֵ��ǰ���id���ö�Ӧ
    servo_sync_parameter.acc_velocity[0] = 150;          
    servo_sync_parameter.acc_velocity[1] = 150;  

    Servo.servo_sync_write_velocity_base_target_acc(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

    # ���ö������Ŀ���Ŀ����ٶ�

    # idΪ1��2�Ķ�����ٶȷֱ�����Ϊ150��150��ֵ��ǰ���id���ö�Ӧ
    servo_sync_parameter.dec_velocity[0] = 150;           
    servo_sync_parameter.dec_velocity[1] = 150;  

    Servo.servo_sync_write_velocity_base_target_dec(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

    # ���ö������Ŀ���Ŀ���ٶ�

    # idΪ1��2�Ķ���ٶȷֱ�����Ϊ3600��1800��ֵ��ǰ���id���ö�Ӧ
    servo_sync_parameter.velocity[0] = 3600;
    servo_sync_parameter.velocity[1] = 1800;

    Servo.servo_sync_write_velocity_base_target_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

    # ���ö������Ŀ���Ŀ��λ��

    # idΪ1��2�Ķ���˶�λ�÷ֱ�����Ϊ0��0��ֵ��ǰ���id���ö�Ӧ
    servo_sync_parameter.position[0] = 0;
    servo_sync_parameter.position[1] = 0;

    Servo.servo_sync_write_velocity_base_target_position(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

    # ���ö������Ŀ���Ŀ��λ�ú��ٶ�

    # idΪ1��2�Ķ���ٶȷֱ�����Ϊ1800��3600��λ�÷ֱ�����Ϊ3000��3000
    servo_sync_parameter.velocity[0] = 1800;
    servo_sync_parameter.velocity[1] = 3600;
    servo_sync_parameter.position[0] = 3000;
    servo_sync_parameter.position[1] = 3000;

    Servo.servo_sync_write_velocity_base_target_position_and_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

    # ���ö������ļ��ٶȣ����ٶȣ��ٶȺ�λ��

    # idΪ1��2�Ķ���ٶȷֱ�����Ϊ3600��3600��λ�÷ֱ�����Ϊ0��0,���ٶȷֱ�����Ϊ100��100�����ٶȷֱ�����Ϊ100��100
    servo_sync_parameter.velocity[0] = 3600;
    servo_sync_parameter.velocity[1] = 3600;
    servo_sync_parameter.position[0] = 0;
    servo_sync_parameter.position[1] = 0;
    servo_sync_parameter.acc_velocity[0] = 100;
    servo_sync_parameter.acc_velocity[1] = 100;
    servo_sync_parameter.dec_velocity[0] = 100;
    servo_sync_parameter.dec_velocity[1] = 100;

    Servo.servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID1����Ŀ���ģʽ
    Servo.servo_set_control_mode(1, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # ����ID1�����Ť�ؿ���
    Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID2�����Ť�ؿ���
    Servo.servo_set_torque_switch(2, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ����ID2����Ŀ���ģʽ
    Servo.servo_set_control_mode(2, 0, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_control_mode_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_control_mode is successful")
    time.sleep(1)

    # ����ID2�����Ť�ؿ���
    Servo.servo_set_torque_switch(2, 1, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    receive_data = serial.read(MAX_RECEIVE_LEN)
    ret = Servo.servo_set_torque_switch_analysis(receive_data)
    if ret == State.SUCCESS:
        print("servo_set_torque_switch is successful")
    time.sleep(1)

    # ���ö������Ŀ�ʱĿ����ٶȵȼ�

    # ���ö��idΪ1��2�ļ��ٶȵȼ��ֱ�Ϊ0��0
    servo_sync_parameter.acc_velocity_grade[0] = 0;
    servo_sync_parameter.acc_velocity_grade[1] = 0;

    Servo.servo_sync_write_time_base_target_acc(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

    # ���ö������Ŀ�ʱĿ��λ�ú��˶�ʱ��

    # ���ö��idΪ1��2���˶�λ��Ϊ3000��3000���˶�ʱ��Ϊ500ms��1500ms
    servo_sync_parameter.position[0] = 3000;
    servo_sync_parameter.position[1] = 3000;
    servo_sync_parameter.time[0] = 500;
    servo_sync_parameter.time[1] = 1500;

    Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
    serial.write(bytes(output_buffer[:output_buffer_len[0]]))
    time.sleep(1)

# �رմ���
serial.close()

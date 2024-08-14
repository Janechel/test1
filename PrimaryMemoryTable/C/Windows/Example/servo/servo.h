#ifndef C_SERVO_H
#define C_SERVO_H

#define MAX_SERVERS 20              //���ͬ��д�������
#define PRINTF_ENABLE 1             //��ӡ���ʹ��

#if PRINTF_ENABLE
#define PRINTF printf
#else
#define PRINTF
#endif


typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef short int16_t;

//Basic instruction types
#define PING                0x01    //Used to obtain Status Packet.
#define READ_DATA           0x02    //Read analysis_data from the memory table.
#define WRITE_DATA          0x03    //Write analysis_data on the memory table.
#define SYNC_WRITE          0X83    //Simultaneously write instructions on multiple servos.
#define FACTORY_RESET       0x06    //Resets the servo's value in the memory table to its initial factory default settings.
#define PARAMETER_RESET     0x10    //Resets the values of certain parameters of the servo to its initial factory default settings.
#define CALIBRATION         0x15    //Calibrate the servo's offset value.
#define REBOOT              0x64    //Reboot the servo

//error message
#define SUCCESS               0     //No errors occurred
#define ID_ERROR              1     //ID number exceeds the range of values
#define INPUT_DATA_ERROR      2     //Input analysis_data cannot be empty
#define DATA_LENGTH_ERROR     3     //The length of input analysis_data cannot be 0
#define QUANTITY_ERROR        4     //The number of servos cannot be 0
#define STATUS_ERROR          5     //Errors that occur during instruction package processing.
#define HEADER_ERROR          6     //There is an error in the header.
#define FAILURE               7     //Parsing failed

#define VOLTAGE_ERROR           0x01
#define ANGLE_ERROR             0x02
#define OVERHEATING_ERROR       0x04
#define RANGE_ERROR             0x08
#define CHECKSUM_ERROR          0x10
#define STALL_ERROR             0x20
#define PARSING_ERROR           0x40
#define UNPACK_ERROR            0x80

#ifndef EM_ADDRESS
#define EM_ADDRESS
//Memory address
#define MODEL_NUMBER                0x00    //Servo Model Number
#define FIRMWARE_VERSION            0x02    //Firmware Version Number
#define MODEL_INFORMATION           0x03    //Factory Serial Number
#define SERVO_ID                    0x07    //Servo ID
#define BAUD_RATE                   0x08    //Baud Rate fixed at 1Mbps
#define RETURN_DELAY_TIME           0x09    //Response Delay Time for servo return to packet
#define RETURN_LEVEL                0x0A    //Returned level of servo status.
#define MIN_ANGLE_LIMIT_L           0x0B    //Minimum Angle Limit for servo rotation
#define MAX_ANGLE_LIMIT_L           0x0D    //Maximum Angle Limit for servo rotation
#define MAX_TEMPERATURE_LIMIT       0x0F    //Maximum Temperature Limit for operating servo
#define MAX_VOLTAGE_LIMIT           0x10    //Maximum Voltage Limit for operating Servo
#define MIN_VOLTAGE_LIMIT           0x11    //Minimum Voltage Limit for operating Servo
#define MAX_PWM_LIMIT_L             0x12    //Maximum PWM Output Limit of servo
#define MAX_CURRENT_LIMIT_L         0x14    //Maximum Current Limit for operating servo
#define CURRENT_TIME_L              0x16    //Trigger Time for overload protection activation after reaching current limit
#define CW_DEADBAND                 0x18    //Dead Band for clockwise direction
#define CCW_DEADBAND                0x19    //Dead Band for counterclockwise direction
#define PWM_PUNCH                   0x1A    //Minimum PWM Value for driving the motor
#define POSITION_P_L                0x1B    //Gain Proportion (P) of Servo's PID Control
#define POSITION_I_L                0x1D    //Gain Integration (I)  of Servo's PID Control
#define POSITION_D_L                0x1F    //Gain Differential (D)  of Servo's PID Control
#define LED_CONDITION               0x21    //Conditions for Alarm LED
#define SHUTDOWN_CONDITION          0x22    //Conditions for torque unloading
#define CONTROL_MODE                0x23    //Servo Control Mode: 0 Time-based Position Control, 1 Acceleration, 2 Current, 3 PWM
#define CALIBRATION_L               0x24    //Offset Value for midpoint Calibration of Servo
#define CURRENT_OFFSET              0x26    //Offset Value for motor current sampling.
#define FLASH_SW                    0x2E    //Flash area write switch: 0 for write disabled, 1 for write enabled.
#define LED_SW                      0x2F    //Servo indicator light switch: 0 for off, 1 for on.
#define TORQUE_SW                   0x30    //Servo torque switch: 0 for torque disabled, 1 for torque enabled, 2 for brake mode.
#define TARGET_PWM_L                0x31    //Direct control of PWM output to the motor.
#define TARGET_CURRENT_L            0x33    //Aim current for servo operation.
#define VELOCITY_BASE_TARGET_POSITION_L     0x35    //Used in Velocity-based Position Control Mode,Plan Profile Position.
#define VELOCITY_BASE_TARGET_VELOCITY_L     0x37    //Used in Velocity-based Position Control Mode,Plan Profile Velocity.
#define VELOCITY_BASE_TARGET_ACC            0x39    //Used in Velocity-based Position Control Mode,Plan Profile Acceleration.
#define VELOCITY_BASE_TARGET_DEC            0x3A    //Used in Velocity-based Position Control Mode,Plan Profile Deceleration.
#define TIME_BASE_TARGET_ACC               0x3B    //Acceleration level in Time-based Position Control Mode.
#define TIME_BASE_TARGET_POSITION_L        0x3C    //Plan position and moving time must be written into the analysis_data simultaneously.
#define TIME_BASE_TARGET_MOVINGTIME_L      0x3E    //Plan position and moving time must be written into the analysis_data simultaneously.
#define PRESENT_VOLTAGE             0x40    //Actual voltage at which the servo is currently operating.
#define PRESENT_TEMPERATURE         0x41    //Actual internal temperature of the servo.
#define PRESENT_PWM_L               0x42    //Present PWM value being output by the servo.
#define PRESENT_PROFILE_VELOCITY_L  0x44    //Present profile velocity of the Profile Planner.
#define PRESENT_PROFILE_POSITION_L  0x46    //Present profile position of the Profile Planner.
#define PRESENT_VELOCITY_L          0x48    //Present actual velocity of the Servo.
#define PRESENT_POSITION_L          0x4A    //Present actual position of the Servo.
#define PRESENT_CURRENT_L           0x4C    //Present actual current of the Servo.
#endif

//������
uint8_t servo_pack(uint8_t id, uint8_t instruction, uint8_t address, uint8_t byte_length, const uint8_t* input_buffer,
    uint8_t* output_buffer, uint8_t* output_length);

//Ӧ�������
uint8_t servo_unpack(uint8_t* response_packet, uint8_t** data_buffer);

//PING
uint8_t servo_ping(uint8_t id, uint8_t* output_buffer, uint8_t* len);

//������
uint8_t servo_read(uint8_t id, uint8_t address, uint8_t read_data_len, uint8_t* output_buffer, uint8_t* output_buffer_len);

//д����
uint8_t servo_write(uint8_t id, uint8_t address, uint8_t write_data_len, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//ͬ��д
uint8_t sync_write_data(uint8_t address, uint8_t servo_counts, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//�ָ���������
uint8_t servo_factory_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��������
uint8_t servo_parameter_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//У��ƫ��ֵ
uint8_t servo_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//����
uint8_t servo_reboot(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//�޸�ָ�����ID
uint8_t servo_modify_known_id(uint8_t id, uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//�޸�δ֪���ID
uint8_t servo_modify_unknown_id(uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����Ӧ����ʱʱ��
uint8_t servo_set_return_delay_time(uint8_t id, uint8_t response_delay_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����״̬���ؼ���
uint8_t servo_set_return_level(uint8_t id, uint8_t return_level, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���Ĳ�����
uint8_t servo_set_baud_rate(uint8_t id, uint8_t baud_rate_number, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö������Сλ������
uint8_t servo_set_min_angle_limit(uint8_t id, uint16_t min_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö�������λ������
uint8_t servo_set_max_angle_limit(uint8_t id, uint16_t max_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö�����¶�����
uint8_t servo_set_max_temperature_limit(uint8_t id, uint8_t max_temperature_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���ĵ�ѹ����
uint8_t servo_set_max_voltage_limit(uint8_t id, uint8_t max_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���ĵ�ѹ����
uint8_t servo_set_min_voltage_limit(uint8_t id, uint8_t min_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����PWM����
uint8_t servo_set_max_pwm_limit(uint8_t id, uint16_t max_pwm_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���ĵ�������
uint8_t servo_set_max_current_limit(uint8_t id, uint16_t max_current_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���ĵ�������ʱ��
uint8_t servo_set_current_shutdown_time(uint8_t id, uint16_t current_shutdown_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö������ת����
uint8_t servo_set_cw_deadband(uint8_t id, uint8_t cw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���ķ�ת����
uint8_t servo_set_ccw_deadband(uint8_t id, uint8_t ccw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����PWM����ֵ
uint8_t servo_set_pwm_punch(uint8_t id, uint8_t pwm_punch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����λ�ÿ���P����
uint8_t servo_set_position_control_p_gain(uint8_t id, uint16_t position_control_P_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����λ�ÿ���I����
uint8_t servo_set_position_control_i_gain(uint8_t id, uint16_t position_control_I_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����λ�ÿ���D����
uint8_t servo_set_position_control_d_gain(uint8_t id, uint16_t position_control_D_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����LED��������
uint8_t servo_set_led_condition(uint8_t id, uint8_t led_condition, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����ж�ر�������
uint8_t servo_set_shutdown_conditions(uint8_t id, uint8_t shutdown_conditions, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���Ŀ���ģʽ
uint8_t servo_set_control_mode(uint8_t id, uint8_t control_mode, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����Flash����
uint8_t servo_set_flash_switch(uint8_t id, uint8_t flash_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����LED����
uint8_t servo_set_led_switch(uint8_t id, uint8_t led_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����Ť�ؿ���
uint8_t servo_set_torque_switch(uint8_t id, uint8_t torque_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����Ŀ��PWM
uint8_t servo_set_target_pwm(uint8_t id, int16_t target_pwm, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö����Ŀ�����
uint8_t servo_set_target_current(uint8_t id, int16_t target_current, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���Ŀ���Ŀ��λ��
uint8_t servo_set_velocity_base_target_position(uint8_t id, uint16_t target_position, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���Ŀ���Ŀ���ٶ�
uint8_t servo_set_velocity_base_target_velocity(uint8_t id, uint16_t target_velocity, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���Ŀ���Ŀ����ٶ�
uint8_t servo_set_velocity_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���Ŀ���Ŀ����ٶ�
uint8_t servo_set_velocity_base_target_dec(uint8_t id, uint8_t target_dec, uint8_t* output_buffer, uint8_t* output_buffer_len);

//brief���ö���Ŀ�ʱĿ����ٶȵȼ�
uint8_t servo_set_time_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö���Ŀ�ʱĿ��λ�ú�Ŀ������ʱ��
uint8_t servo_set_time_base_target_position_and_moving_time(uint8_t id, uint16_t target_position, uint16_t moving_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ǰ����

uint8_t servo_read_present_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ǰλ��

uint8_t servo_read_present_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ǰ�ٶ�
uint8_t servo_read_present_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ǰ�滮λ��
uint8_t servo_read_present_profile_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ǰ�滮�ٶ�
uint8_t servo_read_present_profile_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ǰPWM
uint8_t servo_read_present_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ǰ�¶�
uint8_t servo_read_present_temperature(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ǰ�����ѹ
uint8_t servo_read_present_voltage(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ŀ�ʱĿ������ʱ��
uint8_t servo_read_time_base_target_moving_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ŀ�ʱĿ��λ��
uint8_t servo_read_time_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ŀ�ʱĿ����ٶȵȼ�
uint8_t servo_read_time_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ŀ���Ŀ����ٶ�
uint8_t servo_read_velocity_base_target_dec(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ŀ���Ŀ����ٶ�
uint8_t servo_read_velocity_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ŀ���Ŀ���ٶ�
uint8_t servo_read_velocity_base_target_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ŀ���Ŀ��λ��
uint8_t servo_read_velocity_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����Ŀ�����
uint8_t servo_read_target_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����Ŀ��PWM
uint8_t servo_read_target_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����Ť�ؿ���״̬
uint8_t servo_read_torque_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����LED����״̬
uint8_t servo_read_led_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����FLASH����״̬
uint8_t servo_read_flash_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ���У��ֵ
uint8_t servo_read_current_offset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�������λУ��ֵ
uint8_t servo_read_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ŀ���ģʽ
uint8_t servo_read_control_mode(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����ж�ر�������
uint8_t servo_read_shutdown_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����LED��������
uint8_t servo_read_led_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����λ�ÿ���D����
uint8_t servo_read_position_control_d_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����λ�ÿ���I����
uint8_t servo_read_position_control_i_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����λ�ÿ���P����
uint8_t servo_read_position_control_p_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����PWM����ֵ
uint8_t servo_read_pwm_punch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ķ�ת����
uint8_t servo_read_ccw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�������ת����
uint8_t servo_read_cw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�������ʱ��
uint8_t servo_read_current_shutdown_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�������
uint8_t servo_read_max_current_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����PWM����
uint8_t servo_read_max_pwm_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ѹ����
uint8_t servo_read_max_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĵ�ѹ����
uint8_t servo_read_min_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ������¶�����
uint8_t servo_read_max_temperature_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ��������λ������
uint8_t servo_read_max_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�������Сλ������
uint8_t servo_read_min_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����״̬���ؼ���
uint8_t servo_read_return_level(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ�����Ӧ���ӳ�ʱ��
uint8_t servo_read_return_delay_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ĳ����ʱ��
uint8_t servo_read_baud_rate(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����ĳ������
uint8_t servo_read_model_information(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//��ȡ����Ĺ̼��汾��
uint8_t servo_read_firmware_version(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö������Ŀ���Ŀ��λ��
uint8_t servo_sync_write_velocity_base_target_position(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö������Ŀ���Ŀ���ٶ�
uint8_t servo_sync_write_velocity_base_target_velocity(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö������Ŀ���Ŀ����ٶ�
uint8_t servo_sync_write_velocity_base_target_acc(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö������Ŀ���Ŀ����ٶ�
uint8_t servo_sync_write_velocity_base_target_dec(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö������Ŀ�ʱĿ����ٶȵȼ�
uint8_t servo_sync_write_time_base_target_acc(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//���ö������Ŀ�ʱĿ��λ�ú�����ʱ��
uint8_t servo_sync_write_time_base_target_position_and_moving_time(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//ping�������Ӧ�������
uint8_t servo_ping_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//�ָ���������ָ���Ӧ�������
uint8_t servo_factory_reset_analysis(uint8_t* response_packet);

//��������ָ���Ӧ�������
uint8_t servo_parameter_reset_analysis(uint8_t* response_packet);

//У׼ƫ��ֵָ���Ӧ�������
uint8_t servo_calibration_analysis(uint8_t* response_packet);

//����ָ���Ӧ�������
uint8_t servo_reboot_analysis(uint8_t* response_packet);

//���ö����Ӧ���ӳ�ʱ��ָ���Ӧ�������
uint8_t servo_set_return_delay_time_analysis(uint8_t* response_packet);

//���ö��״̬���ؼ���ָ���Ӧ�������
uint8_t servo_set_return_level_analysis(uint8_t* response_packet);

//���ö�������ʱ��ָ���Ӧ�������
uint8_t servo_set_baud_rate_analysis(uint8_t* response_packet);

//���ö����Сδ֪����ָ���Ӧ�������
uint8_t servo_set_min_angle_limit_analysis(uint8_t* response_packet);

//���ö�����δ֪����ָ���Ӧ�������
uint8_t servo_set_max_angle_limit_analysis(uint8_t* response_packet);

//���ö���¶�����ָ���Ӧ�������
uint8_t servo_set_max_temperature_limit_analysis(uint8_t* response_packet);

//���ö����ѹ����ָ���Ӧ�������
uint8_t servo_set_max_voltage_limit_analysis(uint8_t* response_packet);

//���ö����ѹ����ָ���Ӧ�������
uint8_t servo_set_min_voltage_limit_analysis(uint8_t* response_packet);

//���ö��PWM����ָ���Ӧ�������
uint8_t servo_set_max_pwm_limit_analysis(uint8_t* response_packet);

//���ö����������ָ���Ӧ�������
uint8_t servo_set_max_current_limit_analysis(uint8_t* response_packet);

//���ö����������ʱ��ָ���Ӧ�������
uint8_t servo_set_current_shutdown_time_analysis(uint8_t* response_packet);

//���ö����ת����ָ���Ӧ�������
uint8_t servo_set_cw_deadband_analysis(uint8_t* response_packet);

//���ö����ת����ָ���Ӧ�������
uint8_t servo_set_ccw_deadband_analysis(uint8_t* response_packet);

//���ö��PWM����ֵָ���Ӧ�������
uint8_t servo_set_pwm_punch_analysis(uint8_t* response_packet);

//���ö��λ�ÿ���P����ָ���Ӧ�������
uint8_t servo_set_position_control_p_gain_analysis(uint8_t* response_packet);

//���ö��λ�ÿ���I����ָ���Ӧ�������
uint8_t servo_set_position_control_i_gain_analysis(uint8_t* response_packet);

//���ö��λ�ÿ���D����ָ���Ӧ�������
uint8_t servo_set_position_control_d_gain_analysis(uint8_t* response_packet);

//���ö��LED��������ָ���Ӧ�������
uint8_t servo_set_led_condition_analysis(uint8_t* response_packet);

//���ö��ж�ر�������ָ���Ӧ�������
uint8_t servo_set_shutdown_conditions_analysis(uint8_t* response_packet);

//���ö������ģʽָ���Ӧ�������
uint8_t servo_set_control_mode_analysis(uint8_t* response_packet);

//���ö��FLASH����״ָ̬���Ӧ�������
uint8_t servo_set_flash_switch_analysis(uint8_t* response_packet);

//���ö��LED����״ָ̬���Ӧ�������
uint8_t servo_set_led_switch_analysis(uint8_t* response_packet);

//���ö��Ť�ؿ���״ָ̬���Ӧ�������
uint8_t servo_set_torque_switch_analysis(uint8_t* response_packet);

//���ö��Ŀ��PWMָ���Ӧ�������
uint8_t servo_set_target_pwm_analysis(uint8_t* response_packet);

//���ö��Ŀ�����ָ���Ӧ�������
uint8_t servo_set_target_current_analysis(uint8_t* response_packet);

//���ö������Ŀ��λ��ָ���Ӧ�������
uint8_t servo_set_velocity_base_target_position_analysis(uint8_t* response_packet);

//���ö������Ŀ���ٶ�ָ���Ӧ�������
uint8_t servo_set_velocity_base_target_velocity_analysis(uint8_t* response_packet);

//���ö������Ŀ����ٶ�ָ���Ӧ�������
uint8_t servo_set_velocity_base_target_acc_analysis(uint8_t* response_packet);

//���ö������Ŀ����ٶ�ָ���Ӧ�������
uint8_t servo_set_velocity_base_target_dec_analysis(uint8_t* response_packet);

//���ö����ʱĿ����ٶ�ָ���Ӧ�������
uint8_t servo_set_time_base_target_acc_analysis(uint8_t* response_packet);

//���ö����ʱĿ��λ�ú�����ʱ��ָ���Ӧ�������
uint8_t servo_set_time_base_target_position_and_moving_time_analysis(uint8_t* response_packet);

//��ȡ����ĵ�ǰ������ָ��Ӧ�������
uint8_t servo_read_present_current_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ǰλ�õ�ָ��Ӧ�������
uint8_t servo_read_present_position_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ǰ�ٶȵ�ָ��Ӧ�������
uint8_t servo_read_present_velocity_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ǰ�滮λ�õ�ָ��Ӧ�������
uint8_t servo_read_present_profile_position_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ǰ�滮�ٶȵ�ָ��Ӧ�������
uint8_t servo_read_present_profile_velocity_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ǰPWM��ָ��Ӧ�������
uint8_t servo_read_present_pwm_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ǰ�¶ȵ�ָ��Ӧ�������
uint8_t servo_read_present_temperature_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ǰ�����ѹ��ָ��Ӧ�������
uint8_t servo_read_present_voltage_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ŀ�ʱĿ������ʱ���ָ��Ӧ�������
uint8_t servo_read_time_base_target_moving_time_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ŀ�ʱĿ��λ�õ�ָ��Ӧ�������
uint8_t servo_read_time_base_target_position_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ŀ�ʱĿ����ٶȵȼ���ָ��Ӧ�������
uint8_t servo_read_time_base_target_acc_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ŀ���Ŀ����ٶȵ�ָ��Ӧ�������
uint8_t servo_read_velocity_base_target_dec_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ŀ���Ŀ����ٶȵ�ָ��Ӧ�������
uint8_t servo_read_velocity_base_target_acc_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ŀ���Ŀ���ٶȵ�ָ��Ӧ�������
uint8_t servo_read_velocity_base_target_velocity_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ŀ���Ŀ��λ�õ�ָ��Ӧ�������
uint8_t servo_read_velocity_base_target_position_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����Ŀ�������ָ��Ӧ�������
uint8_t servo_read_target_current_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����Ŀ��PWM��ָ��Ӧ�������
uint8_t servo_read_target_pwm_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����Ť�ؿ���״̬��ָ��Ӧ�������
uint8_t servo_read_torque_switch_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����LED����״̬��ָ��Ӧ�������
uint8_t servo_read_led_switch_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����FLASH����״̬��ָ��Ӧ�������
uint8_t servo_read_flash_switch_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ���У��ֵ��ָ��Ӧ�������
uint8_t servo_read_current_offset_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�������λУ��ֵ��ָ��Ӧ�������
uint8_t servo_read_calibration_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ŀ���ģʽ��ָ��Ӧ�������
uint8_t servo_read_control_mode_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����ж�ر���������ָ��Ӧ�������
uint8_t servo_read_shutdown_condition_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����LED����������ָ��Ӧ�������
uint8_t servo_read_led_condition_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����λ�ÿ���D�����ָ��Ӧ�������
uint8_t servo_read_position_control_d_gain_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����λ�ÿ���I�����ָ��Ӧ�������
uint8_t servo_read_position_control_i_gain_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����λ�ÿ���P�����ָ��Ӧ�������
uint8_t servo_read_position_control_p_gain_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����PWM����ֵ��ָ��Ӧ�������
uint8_t servo_read_pwm_punch_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ķ�ת������ָ��Ӧ�������
uint8_t servo_read_ccw_deadband_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�������ת������ָ��Ӧ�������
uint8_t servo_read_cw_deadband_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�������ʱ���ָ��Ӧ�������
uint8_t servo_read_current_shutdown_time_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ������޵�ָ��Ӧ�������
uint8_t servo_read_max_current_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����PWM���޵�ָ��Ӧ�������
uint8_t servo_read_max_pwm_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ѹ���޵�ָ��Ӧ�������
uint8_t servo_read_max_voltage_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĵ�ѹ���޵�ָ��Ӧ�������
uint8_t servo_read_min_voltage_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ������¶����޵�ָ��Ӧ�������
uint8_t servo_read_max_temperature_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ��������λ�����Ƶ�ָ��Ӧ�������
uint8_t servo_read_max_angle_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�������Сλ�����Ƶ�ָ��Ӧ�������
uint8_t servo_read_min_angle_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����״̬���ؼ����ָ��Ӧ�������
uint8_t servo_read_return_level_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ�����Ӧ���ӳ�ʱ���ָ��Ӧ�������
uint8_t servo_read_return_delay_time_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ĳ����ʱ�ŵ�ָ��Ӧ�������
uint8_t servo_read_baud_rate_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����ĳ�����ŵ�ָ��Ӧ�������
uint8_t servo_read_model_information_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//��ȡ����Ĺ̼��汾�ŵ�ָ��Ӧ�������
uint8_t servo_read_firmware_version_analysis(uint8_t* response_packet, uint16_t* analysis_data);


#endif

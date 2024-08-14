#ifndef C_SERVO_H
#define C_SERVO_H

#define MAX_SERVERS 20              //最大同步写舵机数量
#define PRINTF_ENABLE 1             //打印输出使能

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

//命令打包
uint8_t servo_pack(uint8_t id, uint8_t instruction, uint8_t address, uint8_t byte_length, const uint8_t* input_buffer,
    uint8_t* output_buffer, uint8_t* output_length);

//应答包解析
uint8_t servo_unpack(uint8_t* response_packet, uint8_t** data_buffer);

//PING
uint8_t servo_ping(uint8_t id, uint8_t* output_buffer, uint8_t* len);

//读命令
uint8_t servo_read(uint8_t id, uint8_t address, uint8_t read_data_len, uint8_t* output_buffer, uint8_t* output_buffer_len);

//写命令
uint8_t servo_write(uint8_t id, uint8_t address, uint8_t write_data_len, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//同步写
uint8_t sync_write_data(uint8_t address, uint8_t servo_counts, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//恢复出厂设置
uint8_t servo_factory_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//参数重置
uint8_t servo_parameter_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//校正偏移值
uint8_t servo_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//重启
uint8_t servo_reboot(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//修改指定舵机ID
uint8_t servo_modify_known_id(uint8_t id, uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//修改未知舵机ID
uint8_t servo_modify_unknown_id(uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的应答延时时间
uint8_t servo_set_return_delay_time(uint8_t id, uint8_t response_delay_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的状态返回级别
uint8_t servo_set_return_level(uint8_t id, uint8_t return_level, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的波特率
uint8_t servo_set_baud_rate(uint8_t id, uint8_t baud_rate_number, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的最小位置限制
uint8_t servo_set_min_angle_limit(uint8_t id, uint16_t min_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的最大位置限制
uint8_t servo_set_max_angle_limit(uint8_t id, uint16_t max_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的温度上限
uint8_t servo_set_max_temperature_limit(uint8_t id, uint8_t max_temperature_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的电压上限
uint8_t servo_set_max_voltage_limit(uint8_t id, uint8_t max_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的电压下限
uint8_t servo_set_min_voltage_limit(uint8_t id, uint8_t min_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的PWM上限
uint8_t servo_set_max_pwm_limit(uint8_t id, uint16_t max_pwm_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的电流上限
uint8_t servo_set_max_current_limit(uint8_t id, uint16_t max_current_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的电流保护时间
uint8_t servo_set_current_shutdown_time(uint8_t id, uint16_t current_shutdown_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的正转死区
uint8_t servo_set_cw_deadband(uint8_t id, uint8_t cw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的反转死区
uint8_t servo_set_ccw_deadband(uint8_t id, uint8_t ccw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的PWM叠加值
uint8_t servo_set_pwm_punch(uint8_t id, uint8_t pwm_punch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的位置控制P增益
uint8_t servo_set_position_control_p_gain(uint8_t id, uint16_t position_control_P_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的位置控制I增益
uint8_t servo_set_position_control_i_gain(uint8_t id, uint16_t position_control_I_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的位置控制D增益
uint8_t servo_set_position_control_d_gain(uint8_t id, uint16_t position_control_D_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的LED报警条件
uint8_t servo_set_led_condition(uint8_t id, uint8_t led_condition, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的卸载保护条件
uint8_t servo_set_shutdown_conditions(uint8_t id, uint8_t shutdown_conditions, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的控制模式
uint8_t servo_set_control_mode(uint8_t id, uint8_t control_mode, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的Flash开关
uint8_t servo_set_flash_switch(uint8_t id, uint8_t flash_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的LED开关
uint8_t servo_set_led_switch(uint8_t id, uint8_t led_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的扭矩开关
uint8_t servo_set_torque_switch(uint8_t id, uint8_t torque_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的目标PWM
uint8_t servo_set_target_pwm(uint8_t id, int16_t target_pwm, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的目标电流
uint8_t servo_set_target_current(uint8_t id, int16_t target_current, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的控速目标位置
uint8_t servo_set_velocity_base_target_position(uint8_t id, uint16_t target_position, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的控速目标速度
uint8_t servo_set_velocity_base_target_velocity(uint8_t id, uint16_t target_velocity, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的控速目标加速度
uint8_t servo_set_velocity_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的控速目标减速度
uint8_t servo_set_velocity_base_target_dec(uint8_t id, uint8_t target_dec, uint8_t* output_buffer, uint8_t* output_buffer_len);

//brief设置舵机的控时目标加速度等级
uint8_t servo_set_time_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置舵机的控时目标位置和目标运行时间
uint8_t servo_set_time_base_target_position_and_moving_time(uint8_t id, uint16_t target_position, uint16_t moving_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的当前电流

uint8_t servo_read_present_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的当前位置

uint8_t servo_read_present_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的当前速度
uint8_t servo_read_present_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的当前规划位置
uint8_t servo_read_present_profile_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的当前规划速度
uint8_t servo_read_present_profile_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的当前PWM
uint8_t servo_read_present_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的当前温度
uint8_t servo_read_present_temperature(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的当前输入电压
uint8_t servo_read_present_voltage(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的控时目标运行时间
uint8_t servo_read_time_base_target_moving_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的控时目标位置
uint8_t servo_read_time_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的控时目标加速度等级
uint8_t servo_read_time_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的控速目标减速度
uint8_t servo_read_velocity_base_target_dec(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的控速目标加速度
uint8_t servo_read_velocity_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的控速目标速度
uint8_t servo_read_velocity_base_target_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的控速目标位置
uint8_t servo_read_velocity_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的目标电流
uint8_t servo_read_target_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的目标PWM
uint8_t servo_read_target_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的扭矩开关状态
uint8_t servo_read_torque_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的LED开关状态
uint8_t servo_read_led_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的FLASH开关状态
uint8_t servo_read_flash_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的电流校正值
uint8_t servo_read_current_offset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的中位校正值
uint8_t servo_read_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的控制模式
uint8_t servo_read_control_mode(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的卸载保护条件
uint8_t servo_read_shutdown_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的LED报警条件
uint8_t servo_read_led_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的位置控制D增益
uint8_t servo_read_position_control_d_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的位置控制I增益
uint8_t servo_read_position_control_i_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的位置控制P增益
uint8_t servo_read_position_control_p_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的PWM叠加值
uint8_t servo_read_pwm_punch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的反转死区
uint8_t servo_read_ccw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的正转死区
uint8_t servo_read_cw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的电流保护时间
uint8_t servo_read_current_shutdown_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的电流上限
uint8_t servo_read_max_current_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的PWM上限
uint8_t servo_read_max_pwm_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的电压上限
uint8_t servo_read_max_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的电压下限
uint8_t servo_read_min_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的温度上限
uint8_t servo_read_max_temperature_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的最大位置限制
uint8_t servo_read_max_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的最小位置限制
uint8_t servo_read_min_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的状态返回级别
uint8_t servo_read_return_level(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的应答延迟时间
uint8_t servo_read_return_delay_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的波特率编号
uint8_t servo_read_baud_rate(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的出厂编号
uint8_t servo_read_model_information(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//读取舵机的固件版本号
uint8_t servo_read_firmware_version(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置多个舵机的控速目标位置
uint8_t servo_sync_write_velocity_base_target_position(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置多个舵机的控速目标速度
uint8_t servo_sync_write_velocity_base_target_velocity(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置多个舵机的控速目标加速度
uint8_t servo_sync_write_velocity_base_target_acc(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置多个舵机的控速目标减速度
uint8_t servo_sync_write_velocity_base_target_dec(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置多个舵机的控时目标加速度等级
uint8_t servo_sync_write_time_base_target_acc(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//设置多个舵机的控时目标位置和运行时间
uint8_t servo_sync_write_time_base_target_position_and_moving_time(uint8_t servo_counts, const uint16_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//ping命令包的应答包解析
uint8_t servo_ping_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//恢复出厂设置指令的应答包解析
uint8_t servo_factory_reset_analysis(uint8_t* response_packet);

//参数重置指令的应答包解析
uint8_t servo_parameter_reset_analysis(uint8_t* response_packet);

//校准偏移值指令的应答包解析
uint8_t servo_calibration_analysis(uint8_t* response_packet);

//重启指令的应答包解析
uint8_t servo_reboot_analysis(uint8_t* response_packet);

//设置舵机的应答延迟时间指令的应答包解析
uint8_t servo_set_return_delay_time_analysis(uint8_t* response_packet);

//设置舵机状态返回级别指令的应答包解析
uint8_t servo_set_return_level_analysis(uint8_t* response_packet);

//设置舵机波特率编号指令的应答包解析
uint8_t servo_set_baud_rate_analysis(uint8_t* response_packet);

//设置舵机最小未知限制指令的应答包解析
uint8_t servo_set_min_angle_limit_analysis(uint8_t* response_packet);

//设置舵机最大未知限制指令的应答包解析
uint8_t servo_set_max_angle_limit_analysis(uint8_t* response_packet);

//设置舵机温度上限指令的应答包解析
uint8_t servo_set_max_temperature_limit_analysis(uint8_t* response_packet);

//设置舵机电压上限指令的应答包解析
uint8_t servo_set_max_voltage_limit_analysis(uint8_t* response_packet);

//设置舵机电压下限指令的应答包解析
uint8_t servo_set_min_voltage_limit_analysis(uint8_t* response_packet);

//设置舵机PWM上限指令的应答包解析
uint8_t servo_set_max_pwm_limit_analysis(uint8_t* response_packet);

//设置舵机电流上限指令的应答包解析
uint8_t servo_set_max_current_limit_analysis(uint8_t* response_packet);

//设置舵机电流保护时间指令的应答包解析
uint8_t servo_set_current_shutdown_time_analysis(uint8_t* response_packet);

//设置舵机正转死区指令的应答包解析
uint8_t servo_set_cw_deadband_analysis(uint8_t* response_packet);

//设置舵机反转死区指令的应答包解析
uint8_t servo_set_ccw_deadband_analysis(uint8_t* response_packet);

//设置舵机PWM叠加值指令的应答包解析
uint8_t servo_set_pwm_punch_analysis(uint8_t* response_packet);

//设置舵机位置控制P增益指令的应答包解析
uint8_t servo_set_position_control_p_gain_analysis(uint8_t* response_packet);

//设置舵机位置控制I增益指令的应答包解析
uint8_t servo_set_position_control_i_gain_analysis(uint8_t* response_packet);

//设置舵机位置控制D增益指令的应答包解析
uint8_t servo_set_position_control_d_gain_analysis(uint8_t* response_packet);

//设置舵机LED报警条件指令的应答包解析
uint8_t servo_set_led_condition_analysis(uint8_t* response_packet);

//设置舵机卸载保护条件指令的应答包解析
uint8_t servo_set_shutdown_conditions_analysis(uint8_t* response_packet);

//设置舵机控制模式指令的应答包解析
uint8_t servo_set_control_mode_analysis(uint8_t* response_packet);

//设置舵机FLASH开关状态指令的应答包解析
uint8_t servo_set_flash_switch_analysis(uint8_t* response_packet);

//设置舵机LED开关状态指令的应答包解析
uint8_t servo_set_led_switch_analysis(uint8_t* response_packet);

//设置舵机扭矩开关状态指令的应答包解析
uint8_t servo_set_torque_switch_analysis(uint8_t* response_packet);

//设置舵机目标PWM指令的应答包解析
uint8_t servo_set_target_pwm_analysis(uint8_t* response_packet);

//设置舵机目标电流指令的应答包解析
uint8_t servo_set_target_current_analysis(uint8_t* response_packet);

//设置舵机控速目标位置指令的应答包解析
uint8_t servo_set_velocity_base_target_position_analysis(uint8_t* response_packet);

//设置舵机控速目标速度指令的应答包解析
uint8_t servo_set_velocity_base_target_velocity_analysis(uint8_t* response_packet);

//设置舵机控速目标加速度指令的应答包解析
uint8_t servo_set_velocity_base_target_acc_analysis(uint8_t* response_packet);

//设置舵机控速目标减速度指令的应答包解析
uint8_t servo_set_velocity_base_target_dec_analysis(uint8_t* response_packet);

//设置舵机控时目标加速度指令的应答包解析
uint8_t servo_set_time_base_target_acc_analysis(uint8_t* response_packet);

//设置舵机控时目标位置和运行时间指令的应答包解析
uint8_t servo_set_time_base_target_position_and_moving_time_analysis(uint8_t* response_packet);

//读取舵机的当前电流的指令应答包解析
uint8_t servo_read_present_current_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的当前位置的指令应答包解析
uint8_t servo_read_present_position_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的当前速度的指令应答包解析
uint8_t servo_read_present_velocity_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的当前规划位置的指令应答包解析
uint8_t servo_read_present_profile_position_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的当前规划速度的指令应答包解析
uint8_t servo_read_present_profile_velocity_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的当前PWM的指令应答包解析
uint8_t servo_read_present_pwm_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的当前温度的指令应答包解析
uint8_t servo_read_present_temperature_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的当前输入电压的指令应答包解析
uint8_t servo_read_present_voltage_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的控时目标运行时间的指令应答包解析
uint8_t servo_read_time_base_target_moving_time_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的控时目标位置的指令应答包解析
uint8_t servo_read_time_base_target_position_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的控时目标加速度等级的指令应答包解析
uint8_t servo_read_time_base_target_acc_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的控速目标减速度的指令应答包解析
uint8_t servo_read_velocity_base_target_dec_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的控速目标加速度的指令应答包解析
uint8_t servo_read_velocity_base_target_acc_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的控速目标速度的指令应答包解析
uint8_t servo_read_velocity_base_target_velocity_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的控速目标位置的指令应答包解析
uint8_t servo_read_velocity_base_target_position_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的目标电流的指令应答包解析
uint8_t servo_read_target_current_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的目标PWM的指令应答包解析
uint8_t servo_read_target_pwm_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的扭矩开关状态的指令应答包解析
uint8_t servo_read_torque_switch_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的LED开关状态的指令应答包解析
uint8_t servo_read_led_switch_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的FLASH开关状态的指令应答包解析
uint8_t servo_read_flash_switch_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的电流校正值的指令应答包解析
uint8_t servo_read_current_offset_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的中位校正值的指令应答包解析
uint8_t servo_read_calibration_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的控制模式的指令应答包解析
uint8_t servo_read_control_mode_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的卸载保护条件的指令应答包解析
uint8_t servo_read_shutdown_condition_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的LED报警条件的指令应答包解析
uint8_t servo_read_led_condition_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的位置控制D增益的指令应答包解析
uint8_t servo_read_position_control_d_gain_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的位置控制I增益的指令应答包解析
uint8_t servo_read_position_control_i_gain_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的位置控制P增益的指令应答包解析
uint8_t servo_read_position_control_p_gain_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的PWM叠加值的指令应答包解析
uint8_t servo_read_pwm_punch_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的反转死区的指令应答包解析
uint8_t servo_read_ccw_deadband_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的正转死区的指令应答包解析
uint8_t servo_read_cw_deadband_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的电流保护时间的指令应答包解析
uint8_t servo_read_current_shutdown_time_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的电流上限的指令应答包解析
uint8_t servo_read_max_current_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的PWM上限的指令应答包解析
uint8_t servo_read_max_pwm_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的电压上限的指令应答包解析
uint8_t servo_read_max_voltage_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的电压下限的指令应答包解析
uint8_t servo_read_min_voltage_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的温度上限的指令应答包解析
uint8_t servo_read_max_temperature_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的最大位置限制的指令应答包解析
uint8_t servo_read_max_angle_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的最小位置限制的指令应答包解析
uint8_t servo_read_min_angle_limit_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的状态返回级别的指令应答包解析
uint8_t servo_read_return_level_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的应答延迟时间的指令应答包解析
uint8_t servo_read_return_delay_time_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的波特率编号的指令应答包解析
uint8_t servo_read_baud_rate_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的出厂编号的指令应答包解析
uint8_t servo_read_model_information_analysis(uint8_t* response_packet, uint16_t* analysis_data);

//读取舵机的固件版本号的指令应答包解析
uint8_t servo_read_firmware_version_analysis(uint8_t* response_packet, uint16_t* analysis_data);


#endif

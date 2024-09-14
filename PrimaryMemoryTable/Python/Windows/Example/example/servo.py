class Instruction:
    """
    这个class存放的是舵机指令类型
    """

    # Used to obtain Status Packet.
    PING = 0x01

    # Read data from the memory table.
    READ_DATA = 0x02

    # Write data on the memory table.
    WRITE_DATA = 0x03

    # Simultaneously write instructions on multiple servos.
    SYNC_WRITE = 0X83

    # Resets the servo's value in the memory table to its initial factory default settings.
    FACTORY_RESET = 0x06

    # Resets the values of certain parameters of the servo to initial factory default settings.
    PARAMETER_RESET = 0x10

    # Calibrate the servo's offset value.
    CALIBRATION = 0x15

    # Reboot the servo
    REBOOT = 0x64


class State:
    """
    这个class存放的是舵机错误状态类型
    """

    # No errors occurred
    SUCCESS = 0

    # 舵机的输入电压不在内存表设定的电压限制范围内
    VOLTAGE_ERROR = 0x01

    # 舵机的目标位置不在内存表设定的位置限制范围内
    ANGLE_ERROR = 0x02

    # 舵机内部的温度超过内存表内限定的最高温度
    OVERHEATING_ERROR = 0x04

    # 指令包的长度超过规定的长度
    RANGE_ERROR = 0x08

    # 指令包的校验和解析不正确
    CHECKSUM_ERROR = 0x10

    # 舵机因过流或过载等发生堵转
    STALL_ERROR = 0x20

    # 指令包没按协议规定的格式导致解析错误
    PARSING_ERROR = 0x40

    # 指令包数据帧格式不对
    UNPACK_ERROR = 0x80

class Servo_Sync_Parameter:
    """
    这个class存放的是同步写指令的参数
    """

    def __init__(self):
        self.id_counts = 0                    # 同步写操作舵机的数量
        self.id = [0] * 20                    # 同步写操作舵机的id
        self.torque_switch = [0] * 20         # 舵机扭矩开关状态
        self.control_mode = [0] * 20          # 舵机控制模式
        self.position = [0] * 20              # 舵机位置
        self.time = [0] * 20                  # 舵机运行时间
        self.velocity = [0] * 20              # 舵机运动速度
        self.acc_velocity = [0] * 20          # 舵机运动加速度
        self.dec_velocity = [0] * 20          # 舵机运动减速度
        self.acc_velocity_grade = [0] * 20    # 舵机控时下的加速度等级


class Address:
    """
    这个class存放的是舵机内存表地址
    """

    # Servo Model Number
    MODEL_NUMBER = 0x00

    # Firmware Version Number
    FIRMWARE_VERSION = 0x02

    # Factory Serial Number
    MODEL_INFORMATION = 0x03

    # Servo ID
    SERVO_ID = 0x07

    # Baud Rate fixed at 1Mbps
    BAUD_RATE = 0x08

    # Response Delay Time for servo return to packet
    RETURN_DELAY_TIME = 0x09

    # Returned level of servo status
    RETURN_LEVEL = 0x0A

    # Minimum Angle Limit for servo rotation
    MIN_ANGLE_LIMIT_L = 0x0B

    # Maximum Angle Limit for servo rotation
    MAX_ANGLE_LIMIT_L = 0x0D

    # Maximum Temperature Limit for operating servo
    MAX_TEMPERATURE_LIMIT = 0x0F

    # Maximum Voltage Limit for operating Servo
    MAX_VOLTAGE_LIMIT = 0x10

    # Minimum Voltage Limit for operating Servo
    MIN_VOLTAGE_LIMIT = 0x11

    # Maximum PWM Output Limit of servo
    MAX_PWM_LIMIT_L = 0x12

    # Maximum Current Limit for operating servo
    MAX_CURRENT_LIMIT_L = 0x14

    # Trigger Time for overload protection activation after reaching current limit
    CURRENT_TIME_L = 0x16

    # Dead Band for clockwise direction
    CW_DEADBAND = 0x18

    # Dead Band for counterclockwise direction
    CCW_DEADBAND = 0x19

    # Minimum PWM Value for driving the motor
    PWM_PUNCH = 0x1A

    # Gain Proportion (P) of Servo's PID Control
    POSITION_P_L = 0x1B

    # Gain Integration (I)  of Servo's PID Control
    POSITION_I_L = 0x1D

    # Gain Differential (D)  of Servo's PID Control
    POSITION_D_L = 0x1F

    # Conditions for Alarm LED
    LED_CONDITION = 0x21

    # Conditions for torque unloading
    SHUTDOWN_CONDITION = 0x22

    # Servo Control Mode: 0 Time-based Position Control, 1 Acceleration, 2 Current, 3 PWM
    CONTROL_MODE = 0x23

    # Offset Value for midpoint Calibration of Servo
    CALIBRATION_L = 0x24

    # Offset Value for motor current sampling
    CURRENT_OFFSET = 0x26

    # Flash area write switch: 0 for write disabled, 1 for write enabled.
    FLASH_SW = 0x2E

    # Servo indicator light switch: 0 for off, 1 for on
    LED_SW = 0x2F

    # Servo torque switch: 0 for torque disabled, 1 for torque enabled, 2 for brake mode
    TORQUE_SW = 0x30

    # Direct control of PWM output to the motor
    TARGET_PWM_L = 0x31

    # Aim current for servo operation
    TARGET_CURRENT_L = 0x33

    # Used in Velocity-based Position Control Mode,Plan Profile Position
    VELOCITY_BASE_TARGET_POSITION_L = 0x35

    # Used in Velocity-based Position Control Mode,Plan Profile Velocity
    VELOCITY_BASE_TARGET_VELOCITY_L = 0x37

    # Used in Velocity-based Position Control Mode,Plan Profile Acceleration
    VELOCITY_BASE_TARGET_ACC = 0x39

    # Used in Velocity-based Position Control Mode,Plan Profile Deceleration
    VELOCITY_BASE_TARGET_DEC = 0x3A

    # Acceleration level in Time-based Position Control Mode
    TIME_BASE_TARGET_ACC = 0x3B

    # Plan position and moving time must be written into the data simultaneously
    TIME_BASE_TARGET_POSITION_L = 0x3C

    # Plan position and moving time must be written into the data simultaneously
    TIME_BASE_TARGET_MOVINGTIME_L = 0x3E

    # Actual voltage at which the servo is currently operating
    PRESENT_VOLTAGE = 0x40

    # Actual internal temperature of the servo
    PRESENT_TEMPERATURE = 0x41

    # Present PWM value being output by the servo
    PRESENT_PWM_L = 0x42

    # Present profile velocity of the Profile Planner
    PRESENT_PROFILE_VELOCITY_L = 0x44

    # Present profile position of the Profile Planner
    PRESENT_PROFILE_POSITION_L = 0x46

    # Present actual velocity of the Servo
    PRESENT_VELOCITY_L = 0x48

    # Present actual position of the Servo
    PRESENT_POSITION_L = 0x4A

    # Present actual current of the Servo
    PRESENT_CURRENT_L = 0x4C


class Servo:
    """
    这个class存放所有操作舵机的方法
    """

    @staticmethod
    def get_check(buffer: list, length: int) -> int:
        """
        计算指令包的校验和
        :param buffer: 要计算校验和的缓冲区指针
        :param length: 缓冲区长度
        :return: 计算得到的校验和
        """

        total = 0
        for i in range(length):
            total += buffer[i]
        total = ~total & 0xFF
        return total

    @staticmethod
    def servo_pack(servo_id: int, instruction: int, address: int, byte_length: int, input_buffer: list,
                   output_buffer: list, output_length: list) -> int:
        """
        生成指令数据
        :param servo_id:指令包的ID
        :param instruction: 指令包指令类型
        :param address: 要读写舵机内存表的首地址
        :param byte_length: 字节长度标志
        :param input_buffer: 写入的数据
        :param output_buffer: 用于存储生成的指令数据包的缓冲区指针
        :param output_length: 指令长度
        :return: 成功或者错误类型
        """

        i = 0
        output_buffer[i] = 0xff
        i += 1
        output_buffer[i] = 0xff
        i += 1
        output_buffer[i] = servo_id
        i += 1

        if instruction == Instruction.PING:
            output_buffer[i] = 0x02
            i += 1
            output_buffer[i] = instruction
            i += 1
        elif instruction == Instruction.READ_DATA:
            output_buffer[i] = 0x04
            i += 1
            output_buffer[i] = instruction
            i += 1
            output_buffer[i] = address
            i += 1
            output_buffer[i] = byte_length
            i += 1
        elif instruction == Instruction.WRITE_DATA:
            output_buffer[i] = byte_length + 3
            i += 1
            output_buffer[i] = instruction
            i += 1
            output_buffer[i] = address
            i += 1
            for j in range(byte_length):
                output_buffer[i] = input_buffer[j]
                i += 1
        elif instruction == Instruction.SYNC_WRITE:
            output_buffer[i] = (input_buffer[1] + 1) * byte_length + 4
            i += 1
            output_buffer[i] = instruction
            i += 1
            for j in range((byte_length * input_buffer[1]) + 2 + byte_length):
                output_buffer[i] = input_buffer[j]
                i += 1
        elif instruction in [Instruction.FACTORY_RESET, Instruction.PARAMETER_RESET, Instruction.CALIBRATION,
                             Instruction.REBOOT]:
            output_buffer[i] = 0x04
            i += 1
            output_buffer[i] = instruction
            i += 1
            output_buffer[i] = 0xdf
            i += 1
            output_buffer[i] = 0xdf
            i += 1

        output_buffer[i] = Servo.get_check(output_buffer[2:], i - 2) & 0xff
        output_length[0] = i + 1
        return State.SUCCESS

    @staticmethod
    def servo_unpack(response_packet: list, data_buffer: list) -> int:
        """
        解析应答包
        :param response_packet: 应答包数据
        :param data_buffer: 从应答包中解析出来的参数数据
        :return: 成功或者错误类型
        """

        length = response_packet[3]
        status = response_packet[4]

        checksum = Servo.get_check(response_packet[2:], length + 1)

        if response_packet[0] != 0xff or response_packet[1] != 0xff or checksum != response_packet[length + 3]:
            print("This is not a complete response package!")
            return State.UNPACK_ERROR

        if status != 0x00:
            if status & State.VOLTAGE_ERROR == State.VOLTAGE_ERROR:
                print("电压报错")
            if status & State.ANGLE_ERROR == State.ANGLE_ERROR:
                print("角度报错")
            if status & State.OVERHEATING_ERROR == State.OVERHEATING_ERROR:
                print("过热报错")
            if status & State.RANGE_ERROR == State.RANGE_ERROR:
                print("范围报错")
            if status & State.CHECKSUM_ERROR == State.CHECKSUM_ERROR:
                print("校验报错")
            if status & State.STALL_ERROR == State.STALL_ERROR:
                print("堵转报错")
            if status & State.PARSING_ERROR == State.PARSING_ERROR:
                print("解析报错")
            return status

        if length > 2:
            data_buffer[0] = response_packet[5]
            data_buffer[1] = response_packet[6]

        return State.SUCCESS

    @staticmethod
    def sync_write_data(address: int, servo_counts: int, input_buffer: list, output_buffer: list,
                        output_buffer_len: list) -> int:
        """
        生成同步写指令包
        :param address: 要写入的存储器地址
        :param servo_counts: 要操作的舵机数量
        :param input_buffer: 写入的数据
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_pack(0xfe, Instruction.SYNC_WRITE, address, servo_counts, input_buffer, output_buffer,
                         output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_write(servo_id: int, address: int, write_data_len: int, input_buffer: list, output_buffer: list,
                    output_buffer_len: list) -> int:
        """
        写指令包的生成
        :param servo_id: 舵机ID
        :param address: 要写入的存储器地址
        :param write_data_len: 要写入的数据长度
        :param input_buffer: 写入的数据
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_pack(servo_id, Instruction.WRITE_DATA, address, write_data_len, input_buffer, output_buffer,
                         output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read(servo_id: int, address: int, read_data_len: int, output_buffer: list,
                   output_buffer_len: list) -> int:
        """
        读取指令包的生成
        :param servo_id: 舵机ID
        :param address: 要读取的存储器地址
        :param read_data_len: 要读取的数据长度
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_pack(servo_id, Instruction.READ_DATA, address, read_data_len, None, output_buffer,
                         output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_ping(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        生成PING指令的指令包
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_pack(servo_id, Instruction.PING, 0, 0, None, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_factory_reset(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        生成恢复出厂设置指令
        :param servo_id: 舵机ID 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_pack(servo_id, Instruction.FACTORY_RESET, 0, 0, None, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_parameter_reset(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        生成参数重置指令
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_pack(servo_id, Instruction.PARAMETER_RESET, 0, 0, None, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_calibration(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        生成校正偏移值的指令
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_pack(servo_id, Instruction.CALIBRATION, 0, 0, None, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_reboot(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        生成舵机重启指令
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_pack(servo_id, Instruction.REBOOT, 0, 0, None, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_position_and_present_current(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前位置和当前电流
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_read(servo_id, Address.PRESENT_POSITION_L, 4, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_current(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前电流
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_read(servo_id, Address.PRESENT_CURRENT_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_position(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前位置
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_read(servo_id, Address.PRESENT_POSITION_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_velocity(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前速度
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_read(servo_id, Address.PRESENT_VELOCITY_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_profile_position(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前的规划位置
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_read(servo_id, Address.PRESENT_PROFILE_POSITION_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_profile_velocity(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前规划速度
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_read(servo_id, Address.PRESENT_PROFILE_VELOCITY_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_pwm(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前PWM
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """

        Servo.servo_read(servo_id, Address.PRESENT_PWM_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_temperature(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前温度
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.PRESENT_TEMPERATURE, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_present_voltage(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的当前输入电压
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.PRESENT_VOLTAGE, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_moving_time(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的控时目标运行时间
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.TIME_BASE_TARGET_MOVINGTIME_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_position(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的控时目标位置
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.TIME_BASE_TARGET_POSITION_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_acc(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的控时加速度等级
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.TIME_BASE_TARGET_ACC, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_dec(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的控速目标减速度
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.VELOCITY_BASE_TARGET_DEC, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_acc(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的控速目标加速度
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.VELOCITY_BASE_TARGET_ACC, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_velocity(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的控速目标速度
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.VELOCITY_BASE_TARGET_VELOCITY_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_position(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的控速目标位置
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.VELOCITY_BASE_TARGET_POSITION_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_target_current(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的目标电流
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.TARGET_CURRENT_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_target_pwm(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的目标PWM
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.TARGET_PWM_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_torque_switch(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的扭矩开关
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.TORQUE_SW, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_led_switch(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的LED开关
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.LED_SW, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_flash_switch(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的Flash开关
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.FLASH_SW, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_current_offset(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的电流校正值
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.CURRENT_OFFSET, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_calibration(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的中位校正值
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.CALIBRATION_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_control_mode(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的控制模式
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.CONTROL_MODE, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_shutdown_condition(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的卸载保护条件
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.SHUTDOWN_CONDITION, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_led_condition(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的LED报警条件
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.LED_CONDITION, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_position_control_d_gain(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的位置控制D增益
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.POSITION_D_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_position_control_i_gain(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的位置控制I增益
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.POSITION_I_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_position_control_p_gain(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的位置控制P增益
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.POSITION_P_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_pwm_punch(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的PWM叠加值
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.PWM_PUNCH, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_ccw_deadband(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的反转死区
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.CCW_DEADBAND, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_cw_deadband(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的正转死区
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.CW_DEADBAND, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_current_shutdown_time(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的电流保护时间
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.CURRENT_TIME_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_max_current_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的电流上限
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.MAX_CURRENT_LIMIT_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_max_pwm_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的PWM上限
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.MAX_PWM_LIMIT_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_max_voltage_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的电压上限
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.MAX_VOLTAGE_LIMIT, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_min_voltage_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的电压下限
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.MIN_VOLTAGE_LIMIT, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_max_temperature_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的温度上限
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.MAX_TEMPERATURE_LIMIT, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_max_angle_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的最大位置限制
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.MAX_ANGLE_LIMIT_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_min_angle_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的最小位置限制
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.MIN_ANGLE_LIMIT_L, 2, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_return_level(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的状态返回级别
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.RETURN_LEVEL, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_return_delay_time(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的应答延时时间
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.RETURN_DELAY_TIME, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_baud_rate(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的波特率
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.BAUD_RATE, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_model_information(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的出厂编号
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.MODEL_INFORMATION, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_read_firmware_version(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        读取舵机的固件版本号
        :param servo_id: 舵机ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        Servo.servo_read(servo_id, Address.FIRMWARE_VERSION, 1, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_ping_analysis(response_packet, analysis_data) -> int:
        """
        PING命令应答包的解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_factory_reset_analysis(response_packet: list) -> int:
        """
        恢复出厂设置命令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = None
        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_parameter_reset_analysis(response_packet) -> int:
        """
        参数重置命令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = None
        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_calibration_analysis(response_packet) -> int:
        """
        校正偏移指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = None
        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_read_present_position_and_present_current_analysis(response_packet, position, current) -> int:
        """
        读取舵机的当前位置和当前电流的指令应答包解析
        :param response_packet: 应答包数据应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 4

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            position[0] = (data_buffer[1] << 8) | data_buffer[0]
            current[0] = (data_buffer[3] << 8) | data_buffer[2]
            return State.SUCCESS

    @staticmethod
    def servo_read_present_current_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的当前电流的指令应答包解析
        :param response_packet: 应答包数据应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_present_position_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的当前位置的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_present_velocity_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的当前速度的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_present_profile_position_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的当前规划位置的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_present_profile_velocity_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的当前规划速度的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_present_pwm_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的当前PWM的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)

        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_present_temperature_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的当前温度的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_present_voltage_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的当前输入电压的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_moving_time_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的控时目标运行时间的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_position_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的控时目标位置的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_acc_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的控时目标加速度等级的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_dec_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的控速目标减速度的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_acc_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的控速目标加速度的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_velocity_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的控速目标速度的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_position_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的控速目标位置的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_target_current_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的目标电流的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_target_pwm_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的目标PWM的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_torque_switch_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的扭矩开关状态的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_led_switch_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的LED开关状态的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_flash_switch_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的FLASH开关状态的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_current_offset_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的电流校正值的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_calibration_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的中位校正值的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)

        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_control_mode_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的控制模式的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_shutdown_condition_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的卸载保护条件的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_led_condition_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的LED报警条件的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_position_control_d_gain_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的位置控制D增益的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_position_control_i_gain_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的位置控制I增益的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_position_control_p_gain_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的位置控制P增益的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_pwm_punch_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的PWM叠加值的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_ccw_deadband_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的反转死区的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_cw_deadband_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的正转死区的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_current_shutdown_time_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的电流保护时间的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_max_current_limit_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的电流上限的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_max_pwm_limit_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的PWM上限的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_max_voltage_limit_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的电压上限的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_min_voltage_limit_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的电压下限的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_max_temperature_limit_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的温度上限的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_max_angle_limit_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的最大位置限制的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_min_angle_limit_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的最小位置限制的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)

        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_return_level_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的状态返回级别的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_return_delay_time_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的应答延迟时间的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_baud_rate_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的波特率编号的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_model_information_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的出厂编号的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_read_firmware_version_analysis(response_packet, analysis_data) -> int:
        """
        读取舵机的固件版本号的指令应答包解析
        :param response_packet: 应答包数据
        :param analysis_data: 解析出来的应答包参数
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            analysis_data[0] = data_buffer[0]
            return State.SUCCESS

    @staticmethod
    def servo_modify_known_id(servo_id: int, new_servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        修改目标舵机的ID
        :param servo_id: 舵机ID
        :param new_servo_id: 修改后ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = new_servo_id
        Servo.servo_write(servo_id, Address.SERVO_ID, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_modify_unknown_id(new_servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        修改未知舵机ID的ID
        :param new_servo_id: 修改后ID
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = new_servo_id
        Servo.servo_write(0xfe, Address.SERVO_ID, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_return_delay_time(servo_id: int, response_delay_time: int, output_buffer: list,
                                    output_buffer_len: list) -> int:
        """
        设置舵机的应答延时时间
        :param servo_id: 舵机ID
        :param response_delay_time: 舵机应答返回数据包的延时时间，取值范围0~255，单位是2微妙
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = response_delay_time
        Servo.servo_write(servo_id, Address.RETURN_DELAY_TIME, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_return_level(servo_id: int, return_level: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的状态返回级别
        :param servo_id: 舵机ID
        :param return_level: 舵机应答返回的级别，取值为0 1 2
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = return_level
        Servo.servo_write(servo_id, Address.RETURN_LEVEL, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_baud_rate(servo_id: int, baud_rate_number: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的波特率
        :param servo_id: 舵机ID
        :param baud_rate_number: 舵机波特率编号，取值范围为0~7
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = baud_rate_number
        Servo.servo_write(servo_id, Address.BAUD_RATE, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_min_angle_limit(servo_id: int, min_angle_limit: int, output_buffer: list,
                                  output_buffer_len: list) -> int:
        """
        设置舵机的最小位置限制
        :param servo_id: 舵机ID
        :param min_angle_limit: 舵机转动的最小位置限制，取值范围为0~3000，单位为0.1°
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = min_angle_limit & 0xff
        buffer[1] = (min_angle_limit >> 8) & 0xff

        Servo.servo_write(servo_id, Address.MIN_ANGLE_LIMIT_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_max_angle_limit(servo_id: int, max_angle_limit: int, output_buffer: list,
                                  output_buffer_len: list) -> int:
        """
        设置舵机的最大位置限制
        :param servo_id: 舵机ID
        :param max_angle_limit: 舵机转动的最大位置限制，取值范围为0~3000，单位为0.1°
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = max_angle_limit & 0xff
        buffer[1] = (max_angle_limit >> 8) & 0xff

        Servo.servo_write(servo_id, Address.MAX_ANGLE_LIMIT_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_max_temperature_limit(servo_id: int, max_temperature_limit: int, output_buffer: list,
                                        output_buffer_len: list) -> int:
        """
        设置舵机的温度上限
        :param servo_id: 舵机ID
        :param max_temperature_limit: 舵机工作的温度上限，取值范围为0~127，单位为1℃
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = max_temperature_limit
        Servo.servo_write(servo_id, Address.MAX_TEMPERATURE_LIMIT, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_max_voltage_limit(servo_id: int, max_voltage_limit: int, output_buffer: list,
                                    output_buffer_len: list) -> int:
        """
        设置舵机的电压上限
        :param servo_id: 舵机ID
        :param max_voltage_limit: 舵机工作的电压上限，取值范围为33~90，单位为0.1V
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = max_voltage_limit
        Servo.servo_write(servo_id, Address.MAX_VOLTAGE_LIMIT, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_min_voltage_limit(servo_id: int, min_voltage_limit: int, output_buffer: list,
                                    output_buffer_len: list) -> int:
        """
        设置舵机的电压下限
        :param servo_id: 舵机ID
        :param min_voltage_limit: 舵机工作的电压下限，取值范围为33~90，单位为0.1V
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = min_voltage_limit
        Servo.servo_write(servo_id, Address.MIN_VOLTAGE_LIMIT, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_max_pwm_limit(servo_id: int, max_pwm_limit: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的PWM上限
        :param servo_id: 舵机ID
        :param max_pwm_limit: 舵机输出的PWM上限，取值范围为0~1000，单位为0.1%
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = max_pwm_limit & 0xff
        buffer[1] = (max_pwm_limit >> 8) & 0xff

        Servo.servo_write(servo_id, Address.MAX_PWM_LIMIT_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_max_current_limit(servo_id: int, max_current_limit: int, output_buffer: list,
                                    output_buffer_len: list) -> int:
        """
        设置舵机的电流上限
        :param servo_id: 舵机ID
        :param max_current_limit: 舵机工作的电流上限，取值范围为0~1500，单位为1mA
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = max_current_limit & 0xff
        buffer[1] = (max_current_limit >> 8) & 0xff

        Servo.servo_write(servo_id, Address.MAX_CURRENT_LIMIT_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_current_shutdown_time(servo_id: int, current_shutdown_time: int, output_buffer: list,
                                        output_buffer_len: list) -> int:
        """
        设置舵机的电流保护时间
        :param servo_id: 舵机ID
        :param current_shutdown_time: 舵机达到电流上限后开启过载保护的触发时间，取值范围为0~65536，单位为1ms
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = current_shutdown_time & 0xff
        buffer[1] = (current_shutdown_time >> 8) & 0xff

        Servo.servo_write(servo_id, Address.CURRENT_TIME_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_cw_deadband(servo_id: int, cw_deadband: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的正转死区
        :param servo_id: 舵机ID
        :param cw_deadband: 正转方向的死区，取值范围为0~255，单位为0.1°
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = cw_deadband
        Servo.servo_write(servo_id, Address.CW_DEADBAND, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_ccw_deadband(servo_id: int, ccw_deadband: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的反转死区
        :param servo_id: 舵机ID
        :param ccw_deadband: 反转方向的死区，取值范围为0~255，单位为0.1°
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = ccw_deadband
        Servo.servo_write(servo_id, Address.CCW_DEADBAND, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_pwm_punch(servo_id: int, pwm_punch: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的PWM叠加值
        :param servo_id: 舵机ID
        :param pwm_punch: 舵机输出PWM的叠加值，取值范围为0~255，单位为0.1%
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = pwm_punch
        Servo.servo_write(servo_id, Address.PWM_PUNCH, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_position_control_p_gain(servo_id: int, position_control_p_gain: int, output_buffer: list,
                                          output_buffer_len: list) -> int:
        """
        设置舵机的位置控制P增益
        :param servo_id: 舵机ID
        :param position_control_p_gain: 舵机位置控制PID的比例项，取值范围为0~65535，Kp = 该值/1000
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = position_control_p_gain & 0xff
        buffer[1] = (position_control_p_gain >> 8) & 0xff

        Servo.servo_write(servo_id, Address.POSITION_P_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_position_control_i_gain(servo_id: int, position_control_i_gain: int, output_buffer: list,
                                          output_buffer_len: list) -> int:
        """
        设置舵机的位置控制I增益
        :param servo_id: 舵机ID
        :param position_control_i_gain: 舵机位置控制PID的积分项，取值范围为0~65535，Ki = 该值/10000
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = position_control_i_gain & 0xff
        buffer[1] = (position_control_i_gain >> 8) & 0xff

        Servo.servo_write(servo_id, Address.POSITION_I_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_position_control_d_gain(servo_id: int, position_control_d_gain: int, output_buffer: list,
                                          output_buffer_len: list) -> int:
        """
        设置舵机的位置控制D增益
        :param servo_id: 舵机ID
        :param position_control_d_gain: 舵机位置控制PID的微分项，取值范围为0~65535，Ki = 该值/100
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = position_control_d_gain & 0xff
        buffer[1] = (position_control_d_gain >> 8) & 0xff

        Servo.servo_write(servo_id, Address.POSITION_D_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_led_condition(servo_id: int, led_condition: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的LED报警条件
        :param servo_id: 舵机ID
        :param led_condition: LED报警的条件设置，取值范围为0~255
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = led_condition
        Servo.servo_write(servo_id, Address.LED_CONDITION, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_shutdown_conditions(servo_id: int, shutdown_conditions: int, output_buffer: list,
                                      output_buffer_len: list) -> int:
        """
        设置舵机的卸载保护条件
        :param servo_id: 舵机ID
        :param shutdown_conditions: 卸载扭矩的条件设置，取值范围为0~255
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = shutdown_conditions
        Servo.servo_write(servo_id, Address.SHUTDOWN_CONDITION, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_control_mode(servo_id: int, control_mode: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的控制模式
        :param servo_id: 舵机ID
        :param control_mode: 取值为0控时 1控速 2电流 3PWM
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = control_mode
        Servo.servo_write(servo_id, Address.CONTROL_MODE, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_flash_switch(servo_id: int, flash_switch: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的Flash开关
        :param servo_id: 舵机ID
        :param flash_switch: Flash区域写入开关：0关闭写入 1开启写入
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = flash_switch
        Servo.servo_write(servo_id, Address.FLASH_SW, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_led_switch(servo_id: int, led_switch: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的LED开关
        :param servo_id: 舵机ID
        :param led_switch: 舵机指示灯开关，0关闭 1开启
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = led_switch
        Servo.servo_write(servo_id, Address.LED_SW, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_torque_switch(servo_id: int, torque_switch: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的扭矩开关
        :param servo_id: 舵机ID
        :param torque_switch: 舵机扭矩使能开关：0关闭 1开启 2刹车模式
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = torque_switch
        Servo.servo_write(servo_id, Address.TORQUE_SW, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_target_pwm(servo_id: int, target_pwm: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的目标PWM
        :param servo_id: 舵机ID
        :param target_pwm: 对点击输出的PWM直接控制，取值范围为-1000~1000，单位为0.1%
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = target_pwm & 0xff
        buffer[1] = (target_pwm >> 8) & 0xff

        Servo.servo_write(servo_id, Address.TARGET_PWM_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_target_current(servo_id: int, target_current: int, output_buffer: list,
                                 output_buffer_len: list) -> int:
        """
        设置舵机的目标电流
        :param servo_id: 舵机ID
        :param target_current: 舵机工作的目标电流，取值范围为-1000~1000，单位为1mA
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = target_current & 0xff
        buffer[1] = (target_current >> 8) & 0xff

        Servo.servo_write(servo_id, Address.TARGET_CURRENT_L, 2, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_position(servo_id: int, target_position: int, output_buffer: list,
                                                output_buffer_len: list) -> int:
        """
        设置舵机的控速目标位置
        :param servo_id: 舵机ID
        :param target_position: 取值范围为0~3000，单位为0.1°
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = target_position & 0xff
        buffer[1] = (target_position >> 8) & 0xff

        Servo.servo_write(servo_id, Address.VELOCITY_BASE_TARGET_POSITION_L, 2, buffer, output_buffer,
                          output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_velocity(servo_id: int, target_velocity: int, output_buffer: list,
                                                output_buffer_len: list) -> int:
        """
        设置舵机的控速目标速度
        :param servo_id: 舵机ID
        :param target_velocity: 取值范围为0~65535，单位为0.1°/s
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 2

        buffer[0] = target_velocity & 0xff
        buffer[1] = (target_velocity >> 8) & 0xff

        Servo.servo_write(servo_id, Address.VELOCITY_BASE_TARGET_VELOCITY_L, 2, buffer, output_buffer,
                          output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_acc(servo_id: int, target_acc: int, output_buffer: list,
                                           output_buffer_len: list) -> int:
        """
        设置舵机的控速目标加速度
        :param servo_id: 舵机ID
        :param target_acc: 取值范围为0~255，单位为50°/s²
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = target_acc
        Servo.servo_write(servo_id, Address.VELOCITY_BASE_TARGET_ACC, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_dec(servo_id: int, target_dec: int, output_buffer: list,
                                           output_buffer_len: list) -> int:
        """
        设置舵机的控速目标减速度
        :param servo_id: 舵机ID
        :param target_dec: 取值范围为0~255，单位为单位为50°/s²
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = target_dec
        Servo.servo_write(servo_id, Address.VELOCITY_BASE_TARGET_DEC, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_time_base_target_acc(servo_id: int, target_acc: int, output_buffer: list,
                                       output_buffer_len: list) -> int:
        """
        设置舵机的控时目标加速度等级
        :param servo_id: 舵机ID
        :param target_acc: 取值范围为0~5
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        input_buffer = [0]
        input_buffer[0] = target_acc
        Servo.servo_write(servo_id, Address.TIME_BASE_TARGET_ACC, 1, input_buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_time_base_target_position_and_moving_time(servo_id: int, target_position: int, moving_time: int,
                                                            output_buffer: list, output_buffer_len: list) -> int:
        """
        设置舵机的控时目标位置和目标运行时间
        :param servo_id: 舵机ID
        :param target_position: 运动目标位置，取值范围为0~3000，单位为0.1°
        :param moving_time: 目标运动时间，取值范围为0~65535，单位为1ms
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        buffer = [0] * 4

        buffer[0] = target_position & 0xff
        buffer[1] = (target_position >> 8) & 0xff
        buffer[2] = moving_time & 0xff
        buffer[3] = (moving_time >> 8) & 0xff

        Servo.servo_write(servo_id, Address.TIME_BASE_TARGET_POSITION_L, 4, buffer, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_set_return_delay_time_analysis(response_packet: list) -> int:
        """
        设置应答延时时间命令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)

        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_return_level_analysis(response_packet: list) -> int:
        """
        恢复状态返回级别命令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)

        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_baud_rate_analysis(response_packet: list) -> int:
        """
        恢复波特率命令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_min_angle_limit_analysis(response_packet: list) -> int:
        """
        恢复最小位置限制命令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_max_angle_limit_analysis(response_packet: list) -> int:
        """
        设置舵机最大未知限制指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_max_temperature_limit_analysis(response_packet: list) -> int:
        """
        设置舵机温度上限指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_max_voltage_limit_analysis(response_packet: list) -> int:
        """
        设置舵机电压上限指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_min_voltage_limit_analysis(response_packet: list) -> int:
        """
        设置舵机电压下限指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_max_pwm_limit_analysis(response_packet: list) -> int:
        """
        设置舵机PWM上限指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_max_current_limit_analysis(response_packet: list) -> int:
        """
        设置舵机电流上限指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_current_shutdown_time_analysis(response_packet: list) -> int:
        """
        设置舵机电流保护时间指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_cw_deadband_analysis(response_packet: list) -> int:
        """
        设置舵机正转死区指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_ccw_deadband_analysis(response_packet: list) -> int:
        """
        设置舵机反转死区指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_pwm_punch_analysis(response_packet: list) -> int:
        """
        设置舵机PWM叠加值指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_position_control_p_gain_analysis(response_packet: list) -> int:
        """
        设置舵机位置控制P增益指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_position_control_i_gain_analysis(response_packet: list) -> int:
        """
        设置舵机位置控制I增益指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_position_control_d_gain_analysis(response_packet: list) -> int:
        """
        设置舵机位置控制D增益指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_led_condition_analysis(response_packet: list) -> int:
        """
        设置舵机LED报警条件指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_shutdown_conditions_analysis(response_packet: list) -> int:
        """
        设置舵机卸载保护条件指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_control_mode_analysis(response_packet: list) -> int:
        """
        设置舵机控制模式指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_flash_switch_analysis(response_packet: list) -> int:
        """
        设置舵机FLASH开关状态指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_led_switch_analysis(response_packet: list) -> int:
        """
        设置舵机LED开关状态指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_torque_switch_analysis(response_packet: list) -> int:
        """
        设置舵机扭矩开关状态指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_target_pwm_analysis(response_packet: list) -> int:
        """
        设置舵机目标PWM指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_target_current_analysis(response_packet: list) -> int:
        """
        设置舵机目标电流指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_position_analysis(response_packet: list) -> int:
        """
        设置舵机控速目标位置指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_velocity_analysis(response_packet: list) -> int:
        """
        设置舵机控速目标速度指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_acc_analysis(response_packet: list) -> int:
        """
        设置舵机控速目标加速度指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_dec_analysis(response_packet: list) -> int:
        """
        设置舵机控速目标减速度指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_time_base_target_acc_analysis(response_packet: list) -> int:
        """
        设置舵机控时目标加速度指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_set_time_base_target_position_and_moving_time_analysis(response_packet: list) -> int:
        """
        设置舵机控时目标位置和运行时间指令的应答包解析
        :param response_packet: 应答包数据
        :return: 成功或者错误类型
        """
        data_buffer = [0] * 2

        ret = Servo.servo_unpack(response_packet, data_buffer)
        if ret != State.SUCCESS:
            return ret
        else:
            return State.SUCCESS

    @staticmethod
    def servo_sync_write_torque_switch(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的扭矩开关
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 1 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.TORQUE_SW
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 1] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 1] = servo_sync_parameter.torque_switch[i] & 0xff

        Servo.sync_write_data(Address.TORQUE_SW, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_control_mode(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控制模式
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 1 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.CONTROL_MODE
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 1] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 1] = servo_sync_parameter.control_mode[i] & 0xff

        Servo.sync_write_data(Address.CONTROL_MODE, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_position(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控速目标位置
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 2 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.VELOCITY_BASE_TARGET_POSITION_L
        parameter[1] = 2

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 2] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 2] = servo_sync_parameter.position[i] & 0xff
            parameter[i + 4 + i * 2] = (servo_sync_parameter.position[i] >> 8) & 0xff

        Servo.sync_write_data(Address.VELOCITY_BASE_TARGET_POSITION_L, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_position_and_velocity(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控速目标位置和速度
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 4 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.VELOCITY_BASE_TARGET_POSITION_L
        parameter[1] = 4

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 4] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 4] = servo_sync_parameter.position[i] & 0xff
            parameter[i + 4 + i * 4] = (servo_sync_parameter.position[i] >> 8) & 0xff
            parameter[i + 5 + i * 4] = servo_sync_parameter.velocity[i] & 0xff
            parameter[i + 6 + i * 4] = (servo_sync_parameter.velocity[i] >> 8) & 0xff

        Servo.sync_write_data(Address.VELOCITY_BASE_TARGET_POSITION_L, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控速目标加速度、减速度、速度和位置
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 6 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.VELOCITY_BASE_TARGET_POSITION_L
        parameter[1] = 6

        for i in range(servo_sync_parameter.id_counts):
            parameter[2 + i * 7] = servo_sync_parameter.id[i]
            parameter[3 + i * 7] = servo_sync_parameter.position[i] & 0xff
            parameter[4 + i * 7] = (servo_sync_parameter.position[i] >> 8) & 0xff
            parameter[5 + i * 7] = servo_sync_parameter.velocity[i] & 0xff
            parameter[6 + i * 7] = (servo_sync_parameter.velocity[i] >> 8) & 0xff
            parameter[7 + i * 7] = servo_sync_parameter.acc_velocity[i] & 0xff
            parameter[8 + i * 7] = servo_sync_parameter.dec_velocity[i] & 0xff

        Servo.sync_write_data(Address.VELOCITY_BASE_TARGET_POSITION_L, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_velocity(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控速目标速度
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 2 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.VELOCITY_BASE_TARGET_VELOCITY_L
        parameter[1] = 2

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 2] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 2] = servo_sync_parameter.velocity[i] & 0xff
            parameter[i + 4 + i * 2] = (servo_sync_parameter.velocity[i] >> 8) & 0xff

        Servo.sync_write_data(Address.VELOCITY_BASE_TARGET_VELOCITY_L, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_acc(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控速目标加速度
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.VELOCITY_BASE_TARGET_ACC
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i * 2 + 2] = servo_sync_parameter.id[i]
            parameter[i * 2 + 3] = servo_sync_parameter.acc_velocity[i]

        Servo.sync_write_data(Address.VELOCITY_BASE_TARGET_ACC, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_dec(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控速目标减速度
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.VELOCITY_BASE_TARGET_DEC
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i * 2 + 2] = servo_sync_parameter.id[i]
            parameter[i * 2 + 3] = servo_sync_parameter.dec_velocity[i]

        Servo.sync_write_data(Address.VELOCITY_BASE_TARGET_DEC, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_time_base_target_acc(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控时目标加速度等级
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.TIME_BASE_TARGET_ACC
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i * 2 + 2] = servo_sync_parameter.id[i]
            parameter[i * 2 + 3] = servo_sync_parameter.acc_velocity_grade[i]

        Servo.sync_write_data(Address.TIME_BASE_TARGET_ACC, servo_sync_parameter.id_counts , parameter, output_buffer, output_buffer_len)
        return State.SUCCESS

    @staticmethod
    def servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter: Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        设置多个舵机的控时目标位置和运动时间
        :param servo_sync_parameter: 舵机同步写修改类
        :param output_buffer: 用于存放指令包的输出缓冲区的指针
        :param output_buffer_len: 指令包的长度
        :return: 成功或者错误类型
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 4 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Address.TIME_BASE_TARGET_POSITION_L
        parameter[1] = 4

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 4] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 4] = servo_sync_parameter.position[i] & 0xff
            parameter[i + 4 + i * 4] = (servo_sync_parameter.position[i] >> 8) & 0xff
            parameter[i + 5 + i * 4] = servo_sync_parameter.time[i] & 0xff
            parameter[i + 6 + i * 4] = (servo_sync_parameter.time[i] >> 8) & 0xff

        Servo.sync_write_data(Address.TIME_BASE_TARGET_POSITION_L, servo_sync_parameter.id_counts, parameter, output_buffer,
                              output_buffer_len)
        return State.SUCCESS

#include <stdlib.h>
#include "servo.h"
#include <Arduino.h>

/**
 * @brief 计算指令包的校验和
 * @param buffer 要计算校验和的缓冲区指针
 * @param length 缓冲区长度
 * @return 计算得到的校验和
 */
uint8_t get_check(const uint8_t* buffer, uint8_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += buffer[i];
    }
    sum = ~sum;
    return sum;
}

/**
 * @brief 生成指令数据
 * @param id 指令包的ID
 * @param instruction 指令包指令类型
 * @param address 要读写舵机内存表的首地址
 * @param byte_length 字节长度标志
 * @param input_buffer 指令包参数数据
 * @param output_buffer 用于存储生成的指令数据包的缓冲区指针
 * @param output_length 指令长度
 * @return 成功或者失败
 */
uint8_t servo_pack(uint8_t id, uint8_t instruction, uint8_t address, uint8_t byte_length, uint8_t* input_buffer,uint8_t* output_buffer, uint8_t* output_length)
{
    uint8_t i = 0;

    output_buffer[i++] = 0xff;
    output_buffer[i++] = 0xff;
    output_buffer[i++] = id;

    switch (instruction)
    {
    case PING:
        output_buffer[i++] = 0x02;
        output_buffer[i++] = instruction;
        break;
    case READ_DATA:
        output_buffer[i++] = 4;
        output_buffer[i++] = instruction;
        output_buffer[i++] = address;
        output_buffer[i++] = byte_length;
        break;
    case WRITE_DATA:
        output_buffer[i++] = byte_length + 3;
        output_buffer[i++] = instruction;
        output_buffer[i++] = address;
        for (uint8_t j = 0; j < byte_length; j++)
        {
            output_buffer[i++] = input_buffer[j];
        }
        break;
    case SYNC_WRITE:
        output_buffer[i++] = (input_buffer[1] + 1) * byte_length + 4;
        output_buffer[i++] = instruction;
        for (int j = 0; j < ((byte_length * input_buffer[1]) + 2 + byte_length); j++)
        {
            output_buffer[i++] = input_buffer[j];
        }
        break;
    case FACTORY_RESET:
    case PARAMETER_RESET:
    case CALIBRATION:
    case REBOOT:
        output_buffer[i++] = 0x04;
        output_buffer[i++] = instruction;
        output_buffer[i++] = 0xdf;
        output_buffer[i++] = 0xdf;
        break;
    default:
        return FAILURE;
    }
    output_buffer[i] = get_check(output_buffer + 2, i - 2);
    *output_length = i + 1;
    return SUCCESS;
}

/**
 * @brief 解析应答包
 * @param response_packet 应答包数据的首地址
 * @param data_buffer 应答包解析出来的数据
 * @return 舵机的状态
 */
uint8_t servo_unpack(uint8_t* response_packet, uint8_t** data_buffer)
{
    uint8_t length;
    uint8_t status;
    uint8_t checksum;

    length = response_packet[3];
    status = response_packet[4];

    checksum = get_check(response_packet + 2, length + 1);

    if (response_packet[0] != 0xff || response_packet[1] != 0xff || checksum != response_packet[length + 3])
    {
        PRINTF("This is not a complete response package!");
        return UNPACK_ERROR;
    }

    if (status != 0x00)
    {
        if ((status & VOLTAGE_ERROR) == VOLTAGE_ERROR)
        {
            PRINTF("电压报错");
        }
        if ((status & ANGLE_ERROR) == ANGLE_ERROR)
        {
            PRINTF("角度报错");
        }
        if ((status & OVERHEATING_ERROR) == OVERHEATING_ERROR)
        {
            PRINTF("过热报错");
        }
        if ((status & RANGE_ERROR) == RANGE_ERROR)
        {
            PRINTF("范围报错");
        }
        if ((status & CHECKSUM_ERROR) == CHECKSUM_ERROR)
        {
            PRINTF("校验报错"); 
        }
        if ((status & STALL_ERROR) == STALL_ERROR)
        {
            PRINTF("堵转报错"); 
        }
        if ((status & PARSING_ERROR) == PARSING_ERROR)
        {
            PRINTF("解析报错"); 
        }
        return status;
    }

    if (length > 2) {
        *data_buffer = &response_packet[5];
    }

    return SUCCESS;
}

/**
 * @brief 生成PING指令的指令包
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_ping(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_pack(id, PING, 0, 0, (uint8_t*)nullptr, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取指令包的生成
 * @param id 舵机ID
 * @param address 要读取的存储器地址
 * @param read_data_len 要读取的数据长度
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read(uint8_t id, uint8_t address, uint8_t read_data_len, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_pack(id, READ_DATA, address, read_data_len, (uint8_t *)nullptr, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 写指令包的生成
 * @param id 舵机ID
 * @param address 要写入的存储器地址
 * @param write_data_len 要写的数据长度
 * @param input_buffer 写入的数据
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_write(uint8_t id, uint8_t address, uint8_t write_data_len, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_pack(id, WRITE_DATA, address, write_data_len, input_buffer, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 生成同步写指令包
 * @param address 要写入的存储器地址
 * @param servo_counts 要操作的舵机数量
 * @param input_buffer 写入的数据参数
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t sync_write_data(uint8_t address, uint8_t servo_counts, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_pack(0xfe, SYNC_WRITE, address, servo_counts, input_buffer, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 生成恢复出厂设置指令
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_factory_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_pack(id, FACTORY_RESET, 0, 0, (uint8_t*)nullptr, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 生成参数重置指令
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_parameter_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_pack(id, PARAMETER_RESET, 0, 0, (uint8_t*)nullptr, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 生成校正偏移值的指令
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_pack(id, CALIBRATION, 0, 0, (uint8_t*)nullptr, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 生成舵机重启指令
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_reboot(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_pack(id, REBOOT, 0, 0, (uint8_t*)nullptr, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 修改目标舵机的ID
 * @param id 舵机ID
 * @param new_id 修改后的ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_modify_known_id(uint8_t id, uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_write(id, SERVO_ID, 1, &new_id, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 修改位置舵机ID的ID
 * @param new_id 修改后的ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_modify_unknown_id(uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_write(0xfe, SERVO_ID, 1, &new_id, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 设置舵机的应答延时时间
 * @param id 舵机ID
 * @param response_delay_time 舵机应答返回数据包的延时时间，取值范围0~255，单位是2微妙
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_return_delay_time(uint8_t id, uint8_t response_delay_time, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_write(id, RETURN_DELAY_TIME, 1, &response_delay_time, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 设置舵机的状态返回级别
 * @param id 舵机ID
 * @param return_level 舵机应答返回的级别，取值为0 1 2
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_return_level(uint8_t id, uint8_t return_level, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_write(id, RETURN_LEVEL, 1, &return_level, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 设置舵机的波特率
 * @param id 舵机ID
 * @param baud_rate_number 舵机波特率编号，取值范围为0~7
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_baud_rate(uint8_t id, uint8_t baud_rate_number, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    servo_write(id, BAUD_RATE, 1, &baud_rate_number, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 设置舵机的最小位置限制
 * @param id 舵机ID
 * @param min_angle_limit 舵机转动的最小位置限制，取值范围为0~3000，单位为0.1°
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_min_angle_limit(uint8_t id, uint16_t min_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = min_angle_limit & 0xff;
    buffer[1] = (min_angle_limit >> 8) & 0xff;

    servo_write(id, MIN_ANGLE_LIMIT_L, 2, buffer, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 设置舵机的最大位置限制
 * @param id 舵机ID
 * @param max_angle_limit 舵机转动的最大位置限制，取值范围为0~3000，单位为0.1°
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_angle_limit(uint8_t id, uint16_t max_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = max_angle_limit & 0xff;
    buffer[1] = (max_angle_limit >> 8) & 0xff;


    servo_write(id, MAX_ANGLE_LIMIT_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的温度上限
 * @param id 舵机ID
 * @param max_temperature_limit 舵机工作的温度上限，取值范围为0~127，单位为1℃
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_temperature_limit(uint8_t id, uint8_t max_temperature_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{


    servo_write(id, MAX_TEMPERATURE_LIMIT, 1, &max_temperature_limit, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的电压上限
 * @param id 舵机ID
 * @param max_voltage_limit 舵机工作的电压上限，取值范围为33~90，单位为0.1V
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_voltage_limit(uint8_t id, uint8_t max_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, MAX_VOLTAGE_LIMIT, 1, &max_voltage_limit, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的电压下限
 * @param id 舵机ID
 * @param min_voltage_limit 舵机工作的电压下限，取值范围为33~90，单位为0.1V
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_min_voltage_limit(uint8_t id, uint8_t min_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, MIN_VOLTAGE_LIMIT, 1, &min_voltage_limit, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的PWM上限
 * @param id 舵机ID
 * @param max_pwm_limit 舵机输出的PWM上限，取值范围为0~1000，单位为0.1%
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_pwm_limit(uint8_t id, uint16_t max_pwm_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = max_pwm_limit & 0xff;
    buffer[1] = (max_pwm_limit >> 8) & 0xff;


    servo_write(id, MAX_PWM_LIMIT_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的电流上限
 * @param id 舵机ID
 * @param max_current_limit 舵机工作的电流上限，取值范围为0~1500，单位为1mA
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_current_limit(uint8_t id, uint16_t max_current_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = max_current_limit & 0xff;
    buffer[1] = (max_current_limit >> 8) & 0xff;


    servo_write(id, MAX_CURRENT_LIMIT_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的电流保护时间
 * @param id 舵机ID
 * @param current_shutdown_time 舵机达到电流上限后开启过载保护的触发时间，取值范围为0~65536，单位为1ms
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_current_shutdown_time(uint8_t id, uint16_t current_shutdown_time, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = current_shutdown_time & 0xff;
    buffer[1] = (current_shutdown_time >> 8) & 0xff;


    servo_write(id, CURRENT_TIME_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的正转死区
 * @param id 舵机ID
 * @param cw_deadband 正转方向的死区，取值范围为0~255，单位为0.1°
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_cw_deadband(uint8_t id, uint8_t cw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, CW_DEADBAND, 1, &cw_deadband, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的反转死区
 * @param id 舵机ID
 * @param ccw_deadband 反转方向的死区，取值范围为0~255，单位为0.1°
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_ccw_deadband(uint8_t id, uint8_t ccw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, CCW_DEADBAND, 1, &ccw_deadband, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的PWM叠加值
 * @param id 舵机ID
 * @param pwm_punch 舵机输出PWM的叠加值，取值范围为0~255，单位为0.1%
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_pwm_punch(uint8_t id, uint8_t pwm_punch, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, PWM_PUNCH, 1, &pwm_punch, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的位置控制P增益
 * @param id 舵机ID
 * @param position_control_P_gain 舵机位置控制PID的比例项，取值范围为0~65535，Kp = 该值/1000
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_position_control_p_gain(uint8_t id, uint16_t position_control_P_gain, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = position_control_P_gain & 0xff;
    buffer[1] = (position_control_P_gain >> 8) & 0xff;


    servo_write(id, POSITION_P_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的位置控制I增益
 * @param id 舵机ID
 * @param position_control_I_gain 舵机位置控制PID的积分项，取值范围为0~65535，Ki = 该值/10000
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_position_control_i_gain(uint8_t id, uint16_t position_control_I_gain, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = position_control_I_gain & 0xff;
    buffer[1] = (position_control_I_gain >> 8) & 0xff;


    servo_write(id, POSITION_I_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的位置控制D增益
 * @param id 舵机ID
 * @param position_control_D_gain 舵机位置控制PID的微分项，取值范围为0~65535，Ki = 该值/100
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_position_control_d_gain(uint8_t id, uint16_t position_control_D_gain, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = position_control_D_gain & 0xff;
    buffer[1] = (position_control_D_gain >> 8) & 0xff;


    servo_write(id, POSITION_D_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的LED报警条件
 * @param id 舵机ID
 * @param led_condition LED报警的条件设置，取值范围为0~255
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_led_condition(uint8_t id, uint8_t led_condition, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, LED_CONDITION, 1, &led_condition, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的卸载保护条件
 * @param id 舵机ID
 * @param shutdown_conditions 卸载扭矩的条件设置，取值范围为0~255
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_shutdown_conditions(uint8_t id, uint8_t shutdown_conditions, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, SHUTDOWN_CONDITION, 1, &shutdown_conditions, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的控制模式
 * @param id 舵机ID
 * @param control_mode 取值为0控时 1控速 2电流 3PWM
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_control_mode(uint8_t id, uint8_t control_mode, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, CONTROL_MODE, 1, &control_mode, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的Flash开关
 * @param id 舵机ID
 * @param flash_switch Flash区域写入开关：0关闭写入 1开启写入
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_flash_switch(uint8_t id, uint8_t flash_switch, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, FLASH_SW, 1, &flash_switch, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的LED开关
 * @param id 舵机ID
 * @param led_switch 舵机指示灯开关，0关闭 1开启
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_led_switch(uint8_t id, uint8_t led_switch, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, LED_SW, 1, &led_switch, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的扭矩开关
 * @param id 舵机ID
 * @param torque_switch 舵机扭矩使能开关：0关闭 1开启 2刹车模式
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_torque_switch(uint8_t id, uint8_t torque_switch, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, TORQUE_SW, 1, &torque_switch, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的目标PWM
 * @param id 舵机ID
 * @param target_pwm 对点击输出的PWM直接控制，取值范围为-1000~1000，单位为0.1%
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_target_pwm(uint8_t id, int16_t target_pwm, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = target_pwm & 0xff;
    buffer[1] = (target_pwm >> 8) & 0xff;


    servo_write(id, TARGET_PWM_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的目标电流
 * @param id 舵机ID
 * @param target_current 舵机工作的目标电流，取值范围为-1000~1000，单位为1mA
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_target_current(uint8_t id, int16_t target_current, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = target_current & 0xff;
    buffer[1] = (target_current >> 8) & 0xff;


    servo_write(id, TARGET_CURRENT_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的控速目标位置
 * @param id 舵机ID
 * @param target_position 取值范围为0~3000，单位为0.1°
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_velocity_base_target_position(uint8_t id, uint16_t target_position, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = target_position & 0xff;
    buffer[1] = (target_position >> 8) & 0xff;


    servo_write(id, VELOCITY_BASE_TARGET_POSITION_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的控速目标速度
 * @param id 舵机ID
 * @param target_velocity 取值范围为0~65535，单位为0.1°/s
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_velocity_base_target_velocity(uint8_t id, uint16_t target_velocity, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[2] = { 0 };


    buffer[0] = target_velocity & 0xff;
    buffer[1] = (target_velocity >> 8) & 0xff;


    servo_write(id, VELOCITY_BASE_TARGET_VELOCITY_L, 2, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的控速目标加速度
 * @param id 舵机ID
 * @param target_acc 取值范围为0~255，单位为50°/s²
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_velocity_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, VELOCITY_BASE_TARGET_ACC, 1, &target_acc, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的控速目标减速度
 * @param id 舵机ID
 * @param target_dec 取值范围为0~255，单位为单位为50°/s²
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_velocity_base_target_dec(uint8_t id, uint8_t target_dec, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, VELOCITY_BASE_TARGET_DEC, 1, &target_dec, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的控时目标加速度等级
 * @param id 舵机ID
 * @param target_acc 取值范围为0~5
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_time_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_write(id, TIME_BASE_TARGET_ACC, 1, &target_acc, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 设置舵机的控时目标位置和目标运行时间
 * @param id 舵机ID
 * @param target_position 运动目标位置，取值范围为0~3000，单位为0.1°
 * @param moving_time 目标运动时间，取值范围为0~65535，单位为1ms
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_time_base_target_position_and_moving_time(uint8_t id, uint16_t target_position, uint16_t moving_time, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t buffer[4] = { 0 };


    buffer[0] = target_position & 0xff;
    buffer[1] = (target_position >> 8) & 0xff;
    buffer[2] = moving_time & 0xff;
    buffer[3] = (moving_time >> 8) & 0xff;


    servo_write(id, TIME_BASE_TARGET_POSITION_L, 4, buffer, output_buffer, output_buffer_len);
    return SUCCESS;
}

/**
 * @brief 读取舵机的当前电流
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PRESENT_CURRENT_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的当前位置
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PRESENT_POSITION_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的当前速度
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PRESENT_VELOCITY_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的当前的规划位置
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_profile_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PRESENT_PROFILE_POSITION_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的当前规划速度
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_profile_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PRESENT_PROFILE_VELOCITY_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的当前PWM
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PRESENT_PWM_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的当前温度
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_temperature(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PRESENT_TEMPERATURE, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的当前输入电压
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_voltage(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PRESENT_VOLTAGE, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的控时目标运行时间
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_time_base_target_moving_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, TIME_BASE_TARGET_MOVINGTIME_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的控时目标位置
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_time_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, TIME_BASE_TARGET_POSITION_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的控时加速度等级
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_time_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, TIME_BASE_TARGET_ACC, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的控速目标减速度
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_velocity_base_target_dec(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, VELOCITY_BASE_TARGET_DEC, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的控速目标加速度
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_velocity_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, VELOCITY_BASE_TARGET_ACC, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的控速目标速度
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_velocity_base_target_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, VELOCITY_BASE_TARGET_VELOCITY_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的控速目标位置
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_velocity_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, VELOCITY_BASE_TARGET_POSITION_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的目标电流
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_target_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, TARGET_CURRENT_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的目标PWM
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_target_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, TARGET_PWM_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的扭矩开关
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_torque_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, TORQUE_SW, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的LED开关
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_led_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, LED_SW, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的Flash开关
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_flash_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, FLASH_SW, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的电流校正值
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_current_offset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, CURRENT_OFFSET, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的中位校正值
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, CALIBRATION_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的控制模式
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_control_mode(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, CONTROL_MODE, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的卸载保护条件
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_shutdown_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, SHUTDOWN_CONDITION, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的LED报警条件
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_led_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, LED_CONDITION, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的位置控制D增益
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_position_control_d_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, POSITION_D_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的位置控制I增益
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_position_control_i_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, POSITION_I_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的位置控制P增益
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_position_control_p_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, POSITION_P_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的PWM叠加值
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_pwm_punch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, PWM_PUNCH, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的反转死区
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_ccw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, CCW_DEADBAND, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的正转死区
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_cw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, CW_DEADBAND, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的电流保护时间
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_current_shutdown_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, CURRENT_TIME_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的电流上限
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_current_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, MAX_CURRENT_LIMIT_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的PWM上限
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_pwm_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, MAX_PWM_LIMIT_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的电压上限
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, MAX_VOLTAGE_LIMIT, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的电压下限
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_min_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, MIN_VOLTAGE_LIMIT, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的温度上限
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_temperature_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, MAX_TEMPERATURE_LIMIT, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的最大位置限制
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, MAX_ANGLE_LIMIT_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的最小位置限制
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_min_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, MIN_ANGLE_LIMIT_L, 2, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的状态返回级别
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_return_level(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, RETURN_LEVEL, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的应答延时时间
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_return_delay_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, RETURN_DELAY_TIME, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的波特率
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_baud_rate(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, BAUD_RATE, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的出厂编号
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_model_information(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, MODEL_INFORMATION, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 读取舵机的固件版本号
 * @param id 舵机ID
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_firmware_version(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    servo_read(id, FIRMWARE_VERSION, 1, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 设置多个舵机的控速目标位置
 * @param servo 舵机同步写控制参数结构体
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_sync_write_velocity_base_target_position(struct servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t* parameter = (uint8_t*)malloc((servo.id_counts * 2 + 2 + servo.id_counts) * sizeof(uint8_t));

    parameter[0] = VELOCITY_BASE_TARGET_POSITION_L;
    parameter[1] = 2;
    for (int i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 3] = servo.id[i];
        parameter[3 + i * 3] = servo.position[i] & 0xff;
        parameter[4 + i * 3] = (servo.position[i] >> 8) & 0xff;
    }


    sync_write_data(VELOCITY_BASE_TARGET_POSITION_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    free(parameter);

    return SUCCESS;
}

/**
 * @brief 设置多个舵机的控速目标位置和速度
 * @param servo 舵机同步写控制参数结构体
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_sync_write_velocity_base_target_position_and_velocity(struct servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t* parameter = (uint8_t*)malloc((servo.id_counts * 4 + 2 + servo.id_counts) * sizeof(uint8_t));

    parameter[0] = VELOCITY_BASE_TARGET_POSITION_L;
    parameter[1] = 4;
    for (int i = 0; i < servo.id_counts; i++)
    {
        parameter[i + 2 + i * 4] = servo.id[i];
        parameter[i + 3 + i * 4] = servo.position[i] & 0xff;
        parameter[i + 4 + i * 4] = (servo.position[i] >> 8) & 0xff;
        parameter[i + 5 + i * 4] = servo.velocity[i] & 0xff;
        parameter[i + 6 + i * 4] = (servo.velocity[i] >> 8) & 0xff;
    }


    sync_write_data(VELOCITY_BASE_TARGET_POSITION_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    free(parameter);

    return SUCCESS;
}

/**
 * @brief 设置多个舵机的控速目标加速度、减速度、速度和位置
 * @param servo 舵机同步写控制参数结构体
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(struct servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t* parameter = (uint8_t*)malloc((servo.id_counts * 6 + 2 + servo.id_counts) * sizeof(uint8_t));

    parameter[0] = VELOCITY_BASE_TARGET_POSITION_L;
    parameter[1] = 6;
    for (int i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 7] = servo.id[i];
        parameter[3 + i * 7] = servo.position[i] & 0xff;
        parameter[4 + i * 7] = (servo.position[i] >> 8) & 0xff;
        parameter[5 + i * 7] = servo.velocity[i] & 0xff;
        parameter[6 + i * 7] = (servo.velocity[i] >> 8) & 0xff;
        parameter[7 + i * 7] = servo.acc_velocity[i];
        parameter[8 + i * 7] = servo.dec_velocity[i];
    }

    sync_write_data(VELOCITY_BASE_TARGET_POSITION_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return SUCCESS;
}

/**
 * @brief 设置多个舵机的控速目标速度
 * @param servo 舵机同步写控制参数结构体
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_sync_write_velocity_base_target_velocity(struct servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t* parameter = (uint8_t*)malloc((servo.id_counts * 2 + 2 + servo.id_counts) * sizeof(uint8_t));


    parameter[0] = VELOCITY_BASE_TARGET_VELOCITY_L;
    parameter[1] = 2;
    for (int i = 0; i < servo.id_counts; i++)
    {
        parameter[i + 2 + i * 2] = servo.id[i];
        parameter[i + 3 + i * 2] = servo.velocity[i] & 0xff;
        parameter[i + 4 + i * 2] = (servo.velocity[i] >> 8) & 0xff;
    }

    sync_write_data(VELOCITY_BASE_TARGET_VELOCITY_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    free(parameter);

    return SUCCESS;
}

/**
 * @brief 设置多个舵机的控速目标加速度
 * @param servo 舵机同步写控制参数结构体
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_sync_write_velocity_base_target_acc(struct servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t* parameter = (uint8_t*)malloc((servo.id_counts + 2 + servo.id_counts) * sizeof(uint8_t));


    parameter[0] = VELOCITY_BASE_TARGET_ACC;
    parameter[1] = 1;
    for (int i = 0; i < servo.id_counts; i++)
    {
        parameter[i * 2 + 2] = servo.id[i];
        parameter[i * 2 + 3] = servo.acc_velocity[i];
    }

    sync_write_data(VELOCITY_BASE_TARGET_ACC, servo.id_counts, parameter, output_buffer, output_buffer_len);

    free(parameter);

    return SUCCESS;
}

/**
 * @brief 设置多个舵机的控速目标减速度
 * @param servo 舵机同步写控制参数结构体
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_sync_write_velocity_base_target_dec(struct servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{

    uint8_t* parameter = (uint8_t*)malloc((servo.id_counts + 2 + servo.id_counts) * sizeof(uint8_t));


    parameter[0] = VELOCITY_BASE_TARGET_DEC;
    parameter[1] = 1;
    for (int i = 0; i < servo.id_counts; i++)
    {
        parameter[i * 2 + 2] = servo.id[i];
        parameter[i * 2 + 3] = servo.dec_velocity[i];
    }

    sync_write_data(VELOCITY_BASE_TARGET_DEC, servo.id_counts, parameter, output_buffer, output_buffer_len);

    free(parameter);

    return SUCCESS;
}

/**
 * @brief 设置多个舵机的控时目标加速度等级
 * @param servo 舵机同步写控制参数结构体
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_sync_write_time_base_target_acc(struct servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t* parameter = (uint8_t*)malloc((servo.id_counts + 2 + servo.id_counts) * sizeof(uint8_t));

    parameter[0] = TIME_BASE_TARGET_ACC;
    parameter[1] = 1;
    for (int i = 0; i < servo.id_counts; i++)
    {
        parameter[i * 2 + 2] = servo.id[i];
        parameter[i * 2 + 3] = servo.acc_velocity_grade[i];
    }

    sync_write_data(TIME_BASE_TARGET_ACC, servo.id_counts, parameter, output_buffer, output_buffer_len);

    free(parameter);

    return SUCCESS;
}

/**
 * @brief 设置多个舵机的控时目标位置和运动时间
 * @param servo 舵机同步写控制参数结构体
 * @param output_buffer 用于存放指令包的输出缓冲区的指针
 * @param output_buffer_len 指令包的长度
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_sync_write_time_base_target_position_and_moving_time(struct servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t* parameter = (uint8_t*)malloc((servo.id_counts * 4 + 2 + servo.id_counts) * sizeof(uint8_t));

    parameter[0] = TIME_BASE_TARGET_POSITION_L;
    parameter[1] = 4;
    for (int i = 0; i < servo.id_counts; i++)
    {
        parameter[i + 2 + i * 4] = servo.id[i];
        parameter[i + 3 + i * 4] = servo.position[i] & 0xff;
        parameter[i + 4 + i * 4] = (servo.position[i] >> 8) & 0xff;
        parameter[i + 5 + i * 4] = servo.time[i] & 0xff;
        parameter[i + 6 + i * 4] = (servo.time[i] >> 8) & 0xff;
    }

    sync_write_data(TIME_BASE_TARGET_POSITION_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    free(parameter);

    return SUCCESS;
}

/**
 * @brief PING命令应答包的解析
 * @param response_packet 应答包数据
 * @param data 解析出来的应答包参数
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_ping_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 恢复出厂设置命令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_factory_reset_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 参数重置命令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_parameter_reset_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 校正偏移指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_calibration_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置应答延时时间命令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_return_delay_time_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 恢复状态返回级别命令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_return_level_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 恢复波特率命令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_baud_rate_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 恢复最小位置限制命令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_min_angle_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机最大未知限制指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_angle_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机温度上限指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_temperature_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机电压上限指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_voltage_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机电压下限指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_min_voltage_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机PWM上限指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_pwm_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机电流上限指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_max_current_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机电流保护时间指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_current_shutdown_time_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机正转死区指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_cw_deadband_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机反转死区指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_ccw_deadband_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机PWM叠加值指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_pwm_punch_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机位置控制P增益指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_position_control_p_gain_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机位置控制I增益指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_position_control_i_gain_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机位置控制D增益指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_position_control_d_gain_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机LED报警条件指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_led_condition_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机卸载保护条件指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_shutdown_conditions_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机控制模式指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_control_mode_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机FLASH开关状态指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_flash_switch_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机LED开关状态指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_led_switch_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机扭矩开关状态指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_torque_switch_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机目标PWM指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_target_pwm_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机目标电流指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_target_current_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机控速目标位置指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_velocity_base_target_position_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机控速目标速度指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_velocity_base_target_velocity_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机控速目标加速度指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_velocity_base_target_acc_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机控速目标减速度指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_velocity_base_target_dec_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机控时目标加速度指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_time_base_target_acc_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 设置舵机控时目标位置和运行时间指令的应答包解析
 * @param response_packet 应答包数据
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_set_time_base_target_position_and_moving_time_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的当前电流的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_current_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的当前位置的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_position_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的当前速度的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_velocity_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的当前规划位置的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_profile_position_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的当前规划速度的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_profile_velocity_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的当前PWM的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_pwm_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的当前温度的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_temperature_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的当前输入电压的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_present_voltage_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的控时目标运行时间的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_time_base_target_moving_time_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的控时目标位置的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_time_base_target_position_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的控时目标加速度等级的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_time_base_target_acc_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;


    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的控速目标减速度的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_velocity_base_target_dec_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的控速目标加速度的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_velocity_base_target_acc_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的控速目标速度的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_velocity_base_target_velocity_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的控速目标位置的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_velocity_base_target_position_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的目标电流的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_target_current_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的目标PWM的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_target_pwm_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的扭矩开关状态的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_torque_switch_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的LED开关状态的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_led_switch_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的FLASH开关状态的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_flash_switch_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的电流校正值的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_current_offset_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的中位校正值的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_calibration_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的控制模式的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_control_mode_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的卸载保护条件的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_shutdown_condition_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];
    }

    return SUCCESS;
}

/**
 * @brief 读取舵机的LED报警条件的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_led_condition_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];
    }

    return SUCCESS;
}

/**
 * @brief 读取舵机的位置控制D增益的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_position_control_d_gain_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的位置控制I增益的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_position_control_i_gain_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的位置控制P增益的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_position_control_p_gain_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的PWM叠加值的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_pwm_punch_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的反转死区的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_ccw_deadband_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的正转死区的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_cw_deadband_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的电流保护时间的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_current_shutdown_time_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的电流上限的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_current_limit_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的PWM上限的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_pwm_limit_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的电压上限的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_voltage_limit_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的电压下限的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_min_voltage_limit_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的温度上限的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_temperature_limit_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的最大位置限制的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_max_angle_limit_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的最小位置限制的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_min_angle_limit_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[1];
        *data = *data << 8;
        *data = *data | data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的状态返回级别的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_return_level_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的应答延迟时间的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_return_delay_time_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的波特率编号的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_baud_rate_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的出厂编号的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_model_information_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}

/**
 * @brief 读取舵机的固件版本号的指令应答包解析
 * @param response_packet 应答包数据
 * @param data
 * @return 执行结果，成功或者错误标志
 */
uint8_t servo_read_firmware_version_analysis(uint8_t* response_packet, uint16_t* data)
{
    uint8_t ret;
    uint8_t* data_buffer = nullptr;

    ret = servo_unpack(response_packet, &data_buffer);

    if (ret != SUCCESS) {
        return ret;
    }
    else {
        *data = data_buffer[0];

        return SUCCESS;
    }
}
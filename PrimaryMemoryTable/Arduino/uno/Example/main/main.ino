#include "servo.h"

#define READ_TEST 0             //读取舵机数据测试
#define WRITE_TEST 0            //写入舵机数据测试
#define SYNC_WRITE_TEST 0       //同步写测试
#define PING_TEST 0             //PING命令测试
#define FACTORY_RESET_TEST 0    //恢复出厂设置测试
#define PARAMETER_RESET_TEST 0  //参数重置测试
#define REBOOT_TEST 0           //重启测试
#define CALIBRATION_TEST 0      //校正偏移值测试
#define MODIFY_ID 0             //修改舵机ID测试
#define MODIFY_UNKNOWN_ID 0     //修改未知ID舵机测试

uint8_t ret;                                                                                         //错误检验标志
uint8_t order_buffer[20];                                                                            //存放生成的指令
uint8_t order_len;                                                                                   //指令长度
uint8_t pack[20];                                                                                    //存放接收的应答包
uint8_t pack_len;                                                                                    //应答包长度
uint16_t analysis_data;                                                                              //应答包解析出来的数据
uint16_t sync_write_velocity_base_target_position[5] = { 1, 0, 2, 0 };                               //同步写多个舵机控速目标位置
uint16_t sync_write_velocity_base_target_velocity[5] = { 1, 3600, 2, 3600 };                         //同步写多个舵机控速目标速度
uint16_t sync_write_velocity_base_target_acc[5] = { 1, 150, 2, 150 };                                //同步写多个舵机控速目标加速度
uint16_t sync_write_velocity_base_target_dec[5] = { 1, 150, 2, 150 };                                //同步写多个舵机控速目标减速度
uint16_t sync_write_time_base_target_acc[5] = { 1, 0, 2, 0 };                                        //同步写多个舵机控时目标加速度
uint16_t sync_write_time_base_target_position_and_moving_time[6] = { 1, 1500, 500, 2, 1500, 500 };  //同步写多个舵机控时目标运动位置和运动时间

Servo servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
}

void loop() {
#if PING_TEST
  //向id为1的舵机发送ping指令
  servo.servo_ping(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_ping_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("Ping successfully! The servo model number is ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if CALIBRATION_TEST
  //校正偏移值
  servo.servo_calibration(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_calibration_analysis(pack);
    if (ret == SUCCESS) {
      Serial.print("servo calibration successfully!\r\n");
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

#endif

#if FACTORY_RESET_TEST
  //恢复出厂设置
  servo.servo_factory_reset(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_factory_reset_analysis(pack);
    if (ret == SUCCESS) {
      Serial.print("servo factory reset successfully!\r\n");
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if PARAMETER_RESET_TEST
  //参数重置
  servo.servo_parameter_reset(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_parameter_reset_analysis(pack);
    if (ret == SUCCESS) {
      Serial.print("servo parameter reset successfully!\r\n");
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if REBOOT_TEST
  //重启舵机
  servo.servo_reboot(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Reboot sent successfully.\r\n");
  } else {
    Serial.print("Failed to send Reboot.\r\n");
  }
  delay(1000);
#endif

#if READ_TEST
  //读取ID1舵机的当前电流
  servo.servo_read_present_current(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_present_current_analysis(pack, &analysis_data);
    Serial.print("present current is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的当前位置
  servo.servo_read_present_position(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_present_position_analysis(pack, &analysis_data);
    Serial.print("present position is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的当前速度
  servo.servo_read_present_velocity(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_present_velocity_analysis(pack, &analysis_data);
    Serial.print("present velocity is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的当前的规划位置
  servo.servo_read_present_profile_position(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_present_profile_position_analysis(pack, &analysis_data);
    Serial.print("present profile position is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的当前规划速度
  servo.servo_read_present_profile_velocity(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_present_profile_velocity_analysis(pack, &analysis_data);
    Serial.print("present profile velocity is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的当前PWM
  servo.servo_read_present_pwm(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_present_pwm_analysis(pack, &analysis_data);
    Serial.print("present pwm is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的当前温度
  servo.servo_read_present_temperature(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_present_temperature_analysis(pack, &analysis_data);
    Serial.print("present temperature is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的当前输入电压
  servo.servo_read_present_voltage(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_present_voltage_analysis(pack, &analysis_data);
    Serial.print("present voltage is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的控时目标运行时间
  servo.servo_read_time_base_target_moving_time(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_time_base_target_moving_time_analysis(pack, &analysis_data);
    Serial.print("present time base target moving time is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的控时目标位置
  servo.servo_read_time_base_target_position(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_time_base_target_position_analysis(pack, &analysis_data);
    Serial.print("present time base target position is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的控时加速度等级
  servo.servo_read_time_base_target_acc(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_time_base_target_acc_analysis(pack, &analysis_data);
    Serial.print("present time base target acc is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的控速目标减速度
  servo.servo_read_velocity_base_target_dec(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_velocity_base_target_dec_analysis(pack, &analysis_data);
    Serial.print("present velocity base target dec is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的控速目标加速度
  servo.servo_read_velocity_base_target_acc(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_velocity_base_target_acc_analysis(pack, &analysis_data);
    Serial.print("present velocity base target acc is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的控速目标速度
  servo.servo_read_velocity_base_target_velocity(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_velocity_base_target_velocity_analysis(pack, &analysis_data);
    Serial.print("present velocity base target velocity is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的控速目标位置
  servo.servo_read_velocity_base_target_position(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_velocity_base_target_position_analysis(pack, &analysis_data);
    Serial.print("present velocity base target position is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的目标电流
  servo.servo_read_target_current(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_target_current_analysis(pack, &analysis_data);
    Serial.print("present target current is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的目标PWM
  servo.servo_read_target_pwm(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_target_pwm_analysis(pack, &analysis_data);
    Serial.print("present target pwm is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的扭矩开关
  servo.servo_read_torque_switch(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_torque_switch_analysis(pack, &analysis_data);
    Serial.print("present torque switch is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的LED开关
  servo.servo_read_led_switch(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_led_switch_analysis(pack, &analysis_data);
    Serial.print("present led switch is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的Flash开关
  servo.servo_read_flash_switch(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_flash_switch_analysis(pack, &analysis_data);
    Serial.print("present flash switch is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的电流校正值
  servo.servo_read_current_offset(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    servo.servo_read_current_offset_analysis(pack, &analysis_data);
    Serial.print("present current offset is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的中位校正值
  servo.servo_read_calibration(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_calibration_analysis(pack, &analysis_data);
    Serial.print("present calibration is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的控制模式
  servo.servo_read_control_mode(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_control_mode_analysis(pack, &analysis_data);
    Serial.print("present control mode is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的卸载保护条件
  servo.servo_read_shutdown_condition(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_shutdown_condition_analysis(pack, &analysis_data);
    Serial.print("present shutdown condition is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的LED报警条件
  servo.servo_read_led_condition(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_led_condition_analysis(pack, &analysis_data);
    Serial.print("present led condition is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的位置控制D增益
  servo.servo_read_position_control_d_gain(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_position_control_d_gain_analysis(pack, &analysis_data);
    Serial.print("present position control d gain is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的位置控制I增益
  servo.servo_read_position_control_i_gain(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_position_control_i_gain_analysis(pack, &analysis_data);
    Serial.print("present position control i gain is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的位置控制P增益
  servo.servo_read_position_control_p_gain(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_position_control_p_gain_analysis(pack, &analysis_data);
    Serial.print("present position control p gain is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的PWM叠加值
  servo.servo_read_pwm_punch(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_pwm_punch_analysis(pack, &analysis_data);
    Serial.print("present pwm punch is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的反转死区
  servo.servo_read_ccw_deadband(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_ccw_deadband_analysis(pack, &analysis_data);
    Serial.print("present ccw deadband is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的正转死区
  servo.servo_read_cw_deadband(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_cw_deadband_analysis(pack, &analysis_data);
    Serial.print("present cw deadband is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的电流保护时间
  servo.servo_read_current_shutdown_time(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_current_shutdown_time_analysis(pack, &analysis_data);
    Serial.print("present current shutdown time is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的电流上限
  servo.servo_read_max_current_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_max_current_limit_analysis(pack, &analysis_data);
    Serial.print("present max current limit is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的PWM上限
  servo.servo_read_max_pwm_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_max_pwm_limit_analysis(pack, &analysis_data);
    Serial.print("present max pwm limit is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的电压上限
  servo.servo_read_max_voltage_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_max_voltage_limit_analysis(pack, &analysis_data);
    Serial.print("present max voltage limit is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的电压下限
  servo.servo_read_min_voltage_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_min_voltage_limit_analysis(pack, &analysis_data);
    Serial.print("present min voltage limit is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的温度上限
  servo.servo_read_max_temperature_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_max_temperature_limit_analysis(pack, &analysis_data);
    Serial.print("present max temperature limit is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的最大位置限制
  servo.servo_read_max_angle_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_max_angle_limit_analysis(pack, &analysis_data);
    Serial.print("present max angle limit is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的最小位置限制
  servo.servo_read_min_angle_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_min_angle_limit_analysis(pack, &analysis_data);
    Serial.print("present min angle limit is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的状态返回级别
  servo.servo_read_return_level(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_return_level_analysis(pack, &analysis_data);
    Serial.print("present return level is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的应答延时时间
  servo.servo_read_return_delay_time(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_return_delay_time_analysis(pack, &analysis_data);
    Serial.print("present return delay time is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的波特率
  servo.servo_read_baud_rate(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_baud_rate_analysis(pack, &analysis_data);
    Serial.print("present baud rate is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的出厂编号
  servo.servo_read_model_information(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_model_information_analysis(pack, &analysis_data);
    Serial.print("present model information is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //读取ID1舵机的固件版本号
  servo.servo_read_firmware_version(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    servo.servo_read_firmware_version_analysis(pack, &analysis_data);
    Serial.print("present firmware version is: ");
    Serial.println(analysis_data);
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if WRITE_TEST
  //设置ID1舵机的应答延时时间
  servo.servo_set_return_delay_time(1, 250, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_return_delay_time_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set return delay time successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的状态返回级别
  servo.servo_set_return_level(1, 2, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_return_level_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set return level successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的波特率
  servo.servo_set_baud_rate(1, 7, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_baud_rate_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set baud rate successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的最小位置限制
  servo.servo_set_min_angle_limit(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_min_angle_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set min angle limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的最大位置限制
  servo.servo_set_max_angle_limit(1, 3000, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_max_angle_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max angle limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的温度上限
  servo.servo_set_max_temperature_limit(1, 100, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_max_temperature_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max temperature limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的电压上限
  servo.servo_set_max_voltage_limit(1, 90, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_max_voltage_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max voltage limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的电压下限
  servo.servo_set_min_voltage_limit(1, 33, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_min_voltage_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set min voltage limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的PWM上限
  servo.servo_set_max_pwm_limit(1, 1000, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_max_pwm_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max pwm limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的电流上限
  servo.servo_set_max_current_limit(1, 400, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_max_current_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max current limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的电流保护时间
  servo.servo_set_current_shutdown_time(1, 1000, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_current_shutdown_time_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set current shutdown time successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的正转死区
  servo.servo_set_cw_deadband(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_cw_deadband_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set cw deadband successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的反转死区
  servo.servo_set_ccw_deadband(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_ccw_deadband_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set ccw deadband successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的PWM叠加值
  servo.servo_set_pwm_punch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_pwm_punch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set pwm punch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的位置控制P增益
  servo.servo_set_position_control_p_gain(1, 6000, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_position_control_P_gain_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set position control p gain successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的位置控制I增益
  servo.servo_set_position_control_i_gain(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_position_control_I_gain_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set position control i gain successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的位置控制D增益
  servo.servo_set_position_control_d_gain(1, 151, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_position_control_D_gain_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set position control d gain successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的LED报警条件
  servo.servo_set_led_condition(1, 36, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_led_condition_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set led condition successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的卸载保护条件
  servo.servo_set_shutdown_conditions(1, 36, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_shutdown_conditions_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set shutdown conditions successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的Flash开关
  servo.servo_set_flash_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_flash_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set flash switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的LED开关
  servo.servo_set_led_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_led_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set led switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控制模式
  servo.servo_set_control_mode(1, 3, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的目标PWM
  servo.servo_set_target_pwm(1, 1000, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_target_pwm_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set target pwm successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(3000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控制模式
  servo.servo_set_control_mode(1, 2, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的目标电流
  servo.servo_set_target_current(1, -1000, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_target_current_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set target current successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(3000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控制模式
  servo.servo_set_control_mode(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控速目标速度
  servo.servo_set_velocity_base_target_velocity(1, 3600, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_velocity_base_target_velocity_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set velocity base target velocity successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控速目标加速度
  servo.servo_set_velocity_base_target_acc(1, 150, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_velocity_base_target_acc_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set velocity base target acc successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控速目标减速度
  servo.servo_set_velocity_base_target_dec(1, 150, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_velocity_base_target_dec_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set velocity base target dec successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控速目标位置
  servo.servo_set_velocity_base_target_position(1, 0, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_velocity_base_target_position_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set velocity base target position successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控制模式
  servo.servo_set_control_mode(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控时目标加速度等级
  servo.servo_set_time_base_target_acc(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_time_base_target_acc_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set time base target acc successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控时目标位置和目标运行时间
  servo.servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_time_base_target_position_and_moving_time_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set time base target position and moving time successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if SYNC_WRITE_TEST
  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控制模式
  servo.servo_set_control_mode(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID2舵机的扭矩开关
  servo.servo_set_torque_switch(2, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID2舵机的控制模式
  servo.servo_set_control_mode(2, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID2舵机的扭矩开关
  servo.servo_set_torque_switch(2, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置多个舵机的控速目标速度
  servo.servo_sync_write_velocity_base_target_velocity(2, sync_write_velocity_base_target_velocity, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //设置多个舵机的控速目标加速度
  servo.servo_sync_write_velocity_base_target_acc(2, sync_write_velocity_base_target_acc, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //设置多个舵机的控速目标减速度
  servo.servo_sync_write_velocity_base_target_dec(2, sync_write_velocity_base_target_dec, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //设置多个舵机的控速目标位置
  servo.servo_sync_write_velocity_base_target_position(2, sync_write_velocity_base_target_position, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的控制模式
  servo.servo_set_control_mode(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID1舵机的扭矩开关
  servo.servo_set_torque_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID2舵机的扭矩开关
  servo.servo_set_torque_switch(2, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID2舵机的控制模式
  servo.servo_set_control_mode(2, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置ID2舵机的扭矩开关
  servo.servo_set_torque_switch(2, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo.servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //设置多个舵机的控时目标加速度等级
  servo.servo_sync_write_time_base_target_acc(2, sync_write_time_base_target_acc, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //设置多个舵机的控时目标位置和时间
  servo.servo_sync_write_time_base_target_position_and_moving_time(2, sync_write_time_base_target_position_and_moving_time, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);
#endif

#if MODIFY_ID
  //将id为1的舵机id修改为2
  servo.servo_modify_known_id(1, 2, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Modify successfully.\r\n");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1000);
#endif

#if MODIFY_UNKNOWN_ID
  //将所有舵机id修改为2
  servo.servo_modify_unknown_id(2, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Modify successfully.\r\n");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1000);
#endif
}

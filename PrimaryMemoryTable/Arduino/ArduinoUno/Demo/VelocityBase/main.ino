#include "servo.h"

uint8_t ret;               //错误检验标志
uint8_t order_buffer[20];  //存放生成的指令
uint8_t order_len;         //指令长度
uint8_t pack[20];          //存放接收的应答包
uint8_t pack_len;          //应答包长度
uint16_t analysis_data;    //应答包解析出来的数据

Servo servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
}

void loop() {
  //参数重置
  servo.servo_parameter_reset(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("servo parameter reset!\r\n");
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

  //设置舵机的控速目标位置
  servo.servo_set_velocity_base_target_position(1, 1500, order_buffer, &order_len);

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
    Serial.print("Failed to read data.\r\n");
  }
  delay(1000);

  //设置舵机的控速目标速度
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
    Serial.print("Failed to read data.\r\n");
  }
  delay(1000);

  //设置舵机的控速目标加速度
  servo.servo_set_velocity_base_target_acc(1, 10, order_buffer, &order_len);

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
    Serial.print("Failed to read data.\r\n");
  }
  delay(1000);

  //设置舵机的控速目标减速度
  servo.servo_set_velocity_base_target_dec(1, 1, order_buffer, &order_len);

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
    Serial.print("Failed to read data.\r\n");
  }
  delay(1000);

  //设置舵机的控速目标位置
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
    Serial.print("Failed to read data.\r\n");
  }
  delay(1000);
}

#include "servo.h"

uint8_t ret;                              //Change Unknown Servo ID Test
uint8_t order_buffer[20];                 //Store Generated Instructions
uint8_t order_len;                        //Instruction Length
uint8_t pack[20];                         //Store the received status packet
uint8_t pack_len;                         //Response packet length.
uint16_t analysis_data;                   //Data parsed from the status packet

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
}

void loop() {
  //Change the torque switch of servo ID1 to OFF.
  servo_set_torque_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the control mode of servo ID1 to the PWM control mode.
  servo_set_control_mode(1, 3, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_control_mode_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set control mode successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the torque switch of servo ID1 to ON.
  servo_set_torque_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_torque_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set torque switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the target PWM of servo ID1 to -50%.
  servo_set_target_pwm(1, -500, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_target_pwm_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set target pwm successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(3000);
}

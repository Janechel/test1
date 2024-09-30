#include <Arduino.h>
#include "servo.h"

uint8_t ret;                              //Change Unknown Servo ID Test
uint8_t order_buffer[40];                 //Store Generated Instructions
uint8_t order_len;                        //Instruction Length
uint8_t pack[40];                         //Store the received status packet
uint8_t pack_len;                         //Response packet length.
uint16_t analysis_data;                   //Data parsed from the status packet

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(1000000, SERIAL_8N1, 16, 17);  
}

void loop() {
  //Change the torque switch of servo ID1 to OFF.
  servo_set_torque_switch(1, 0, order_buffer,&order_len);

  if (order_len == Serial2.write(order_buffer, order_len)) 
  {
    PRINTF("Write successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to send data.\r\n");
  }
  delay(1);

  if (Serial2.available()>0) 
  {
    pack_len = Serial2.available();
    Serial2.read(pack, pack_len);
    ret = servo_set_torque_switch_analysis(pack);
    if(ret == SUCCESS)
      PRINTF("servo set torque switch successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to read data.\r\n");
  }
  delay(1000);

  //Change the control mode of servo ID1 to the PWM control mode.
  servo_set_control_mode(1, 3, order_buffer,&order_len);

  if (order_len == Serial2.write(order_buffer, order_len)) 
  {
    PRINTF("Write successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to send data.\r\n");
  }
  delay(1);

  if (Serial2.available()>0) 
  {
    pack_len = Serial2.available();
    Serial2.read(pack, pack_len);
    ret = servo_set_control_mode_analysis(pack);
    if(ret == SUCCESS)
      PRINTF("servo set control mode successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to read data.\r\n");
  }
  delay(1000);

  //Change the torque switch of servo ID1 to ON.
  servo_set_torque_switch(1, 1, order_buffer,&order_len);

  if (order_len == Serial2.write(order_buffer, order_len)) {
    PRINTF("Write successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to send data.\r\n");
  }
  delay(1);

  if (Serial2.available()>0) 
  {
    pack_len = Serial2.available();
    Serial2.read(pack, pack_len);
    ret = servo_set_torque_switch_analysis(pack);
    if(ret == SUCCESS)
      PRINTF("servo set torque switch successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to read data.\r\n");
  }
  delay(1000);

  //Change the target PWM of servo ID1 to -50%.
  servo_set_target_pwm(1, -500, order_buffer,&order_len);

  if (order_len == Serial2.write(order_buffer, order_len)) 
  { 
    PRINTF("Write successfully.\r\n");
  } 
  else 
  { 
    PRINTF("Failed to send data.\r\n");
  }
  delay(1);

  if (Serial2.available()>0) 
  {
    pack_len = Serial2.available();
    Serial2.read(pack, pack_len);
    ret = servo_set_target_pwm_analysis(pack);
    if(ret == SUCCESS)
        PRINTF("servo set target pwm successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to read data.\r\n");
  }
  delay(3000);
}

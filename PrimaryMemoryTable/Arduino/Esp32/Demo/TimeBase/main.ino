#include <Arduino.h>
#include "servo.h"

uint8_t ret;                              //错误检验标志
uint8_t order_buffer[20];                 //存放生成的指令
uint8_t order_len;                        //指令长度
uint8_t pack[20];                         //存放接收的应答包
uint8_t pack_len;                         //应答包长度
uint16_t analysis_data;                   //应答包解析出来的数据

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(1000000, SERIAL_8N1, 16, 17);  
}

void loop() {
  //设置舵机的扭矩开关
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

  //设置舵机的控制模式
  servo_set_control_mode(1, 0, order_buffer,&order_len);

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

  //设置舵机的扭矩开关
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

  //设置舵机的控时目标位置和目标运行时间
  servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer,&order_len);
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
    ret = servo_set_time_base_target_position_and_moving_time_analysis(pack);
    if(ret == SUCCESS)
      PRINTF("servo set time base target position and moving time successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to read data.\r\n");
  }
  delay(1000);

  //设置舵机的控时目标加速度等级
  servo_set_time_base_target_acc(1, 0, order_buffer,&order_len);

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
    ret = servo_set_time_base_target_acc_analysis(pack);
    if(ret == SUCCESS)
        PRINTF("servo set time base target acc successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to read data.\r\n");
  }
  delay(1000);

  //设置舵机的控时目标位置和目标运行时间
  servo_set_time_base_target_position_and_moving_time(1, 0, 1000, order_buffer,&order_len);
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
    ret = servo_set_time_base_target_position_and_moving_time_analysis(pack);
    if(ret == SUCCESS)
      PRINTF("servo set time base target position and moving time successfully.\r\n");
  } 
  else 
  {
    PRINTF("Failed to read data.\r\n");
  }
  delay(1000);
}

#include "servo.h"

#define READ_TEST 0             //Read Servo Data Test
#define WRITE_TEST 0            //Write Servo Data Test
#define SYNC_WRITE_TEST 0       //Sync Write Test
#define PING_TEST 0             //PING Instruction Test
#define FACTORY_RESET_TEST 0    //Factory Reset Test
#define PARAMETER_RESET_TEST 0  //Parameter Reset Test
#define REBOOT_TEST 0           //Reboot Test
#define CALIBRATION_TEST 0      //Calibration Test
#define MODIFY_ID 0             //Change Known Servo ID Test
#define MODIFY_UNKNOWN_ID 0     //Change Unknown Servo ID Test

uint8_t ret;                              //Change Unknown Servo ID Test
uint8_t order_buffer[40];                 //Store Generated Instructions
uint8_t order_len;                        //Instruction Length
uint8_t pack[40];                         //Store the received status packet
uint8_t pack_len;                         //Response packet length.
uint16_t analysis_data;                   //Data parsed from the status packet
uint16_t position;                        //Present position of the servo
uint16_t current;                         //Present current of the servo
uint8_t write_buffer[20];                 //Write data to the memory table

struct servo_sync_parameter servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
}

void loop() {
#if PING_TEST
  //Query the model number of servo ID1.
  servo_ping(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_ping_analysis(pack, &analysis_data);
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
  //Calibrate the midpoint of the servo.
  servo_calibration(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_calibration_analysis(pack);
    if (ret == SUCCESS) {
      Serial.print("servo calibration successfully!\r\n");
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

#endif

#if FACTORY_RESET_TEST
  //Reset the servo to the factory default values.
  servo_factory_reset(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_factory_reset_analysis(pack);
    if (ret == SUCCESS) {
      Serial.print("servo factory reset successfully!\r\n");
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if PARAMETER_RESET_TEST
  //Reset the parameter settings of the servo.
  servo_parameter_reset(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_parameter_reset_analysis(pack);
    if (ret == SUCCESS) {
      Serial.print("servo parameter reset successfully!\r\n");
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if REBOOT_TEST
  //Reboot the servo.
  servo_reboot(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Reboot sent successfully.\r\n");
  } else {
    Serial.print("Failed to send Reboot.\r\n");
  }
  delay(1000);
#endif

#if READ_TEST
  //Read the present current of servo ID1.
  servo_read_present_current(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_current_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present current is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the present position of servo ID1.
  servo_read_present_position(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present position is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the present position and present current of servo ID1.
  servo_read_present_position_and_present_current(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_position_and_present_current_analysis(pack, &position, &current);
    if (ret == SUCCESS) {
      Serial.print("present position is: ");
      Serial.print(position);
      Serial.print("  present current is: ");
      Serial.println(current);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the present velocity of servo ID1.
  servo_read_present_velocity(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present velocity is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the present profile position of servo ID1.
  servo_read_present_profile_position(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_profile_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present profile position is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the present profile velocity of servo ID1.
  servo_read_present_profile_velocity(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_profile_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present profile velocity is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the present PWM of servo ID1.
  servo_read_present_pwm(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_pwm_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present pwm is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the present temperature of servo ID1.
  servo_read_present_temperature(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_temperature_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present temperature is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the present voltage of servo ID1.
  servo_read_present_voltage(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_present_voltage_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present voltage is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the time base target moving time of servo ID1.
  servo_read_time_base_target_moving_time(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_time_base_target_moving_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present time base target moving time is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the time base target position of servo ID1.
  servo_read_time_base_target_position(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_time_base_target_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present time base target position is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the time base target ACC of servo ID1.
  servo_read_time_base_target_acc(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_time_base_target_acc_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present time base target acc is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the velocity base target DEC of servo ID1.
  servo_read_velocity_base_target_dec(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_velocity_base_target_dec_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present velocity base target dec is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the velocity base target ACC of servo ID1.
  servo_read_velocity_base_target_acc(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_velocity_base_target_acc_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present velocity base target acc is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the velocity base target velocity of servo ID1.
  servo_read_velocity_base_target_velocity(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_velocity_base_target_velocity_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present velocity base target velocity is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the velocity base target position of servo ID1.
  servo_read_velocity_base_target_position(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_velocity_base_target_position_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present velocity base target position is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the velocity base target position and velocity of servo ID1.
  servo_read(1, 0x35, 4, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    Serial.print("the velocity base target position and velocity pack is: ");
    for(uint8_t i = 0; i < pack_len; i++)
    {
      Serial.print("0x%x ", pack[i]);
    }
    Serial.println();
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the target current of servo ID1.
  servo_read_target_current(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_target_current_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present target current is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the target PWM of servo ID1.
  servo_read_target_pwm(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_target_pwm_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present target pwm is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the torque switch of servo ID1.
  servo_read_torque_switch(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_torque_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present torque switch is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the LED switch of servo ID1.
  servo_read_led_switch(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_led_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present led switch is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the Flash switch of servo ID1.
  servo_read_flash_switch(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_flash_switch_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present flash switch is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the current offset of servo ID1.
  servo_read_current_offset(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_current_offset_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present current offset is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the calibration of servo ID1.
  servo_read_calibration(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_calibration_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present calibration is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the control mode of servo ID1.
  servo_read_control_mode(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_control_mode_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present control mode is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the shutdown condition of servo ID1.
  servo_read_shutdown_condition(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_shutdown_condition_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present shutdown condition is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the LED condition of servo ID1.
  servo_read_led_condition(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_led_condition_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present led condition is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the position control D gain of servo ID1.
  servo_read_position_control_d_gain(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_position_control_d_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present position control d gain is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the position control I gain of servo ID1.
  servo_read_position_control_i_gain(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_position_control_i_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present position control i gain is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the position control P gain of servo ID1.
  servo_read_position_control_p_gain(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_position_control_p_gain_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present position control p gain is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the position control PID gain of servo ID1.
  servo_read(1, 0x1B, 6, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    Serial.print("position control pid gain pack is: ");
    for(uint8_t i = 0; i < pack_len; i++)
    {
      Serial.print("0x%x ", pack[i]);
    }
    Serial.println();
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the PWM punch of servo ID1.
  servo_read_pwm_punch(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_pwm_punch_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present pwm punch is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the ccw deadband of servo ID1.
  servo_read_ccw_deadband(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_ccw_deadband_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present ccw deadband is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the cw deadband of servo ID1.
  servo_read_cw_deadband(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_cw_deadband_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present cw deadband is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the current shutdown time of servo ID1.
  servo_read_current_shutdown_time(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_current_shutdown_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present current shutdown time is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the max current limit of servo ID1.
  servo_read_max_current_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_max_current_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present max current limit is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the max PWM limit of servo ID1.
  servo_read_max_pwm_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_max_pwm_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present max pwm limit is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the max voltage limit of servo ID1.
  servo_read_max_voltage_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_max_voltage_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present max voltage limit is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the min voltage limit of servo ID1.
  servo_read_min_voltage_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_min_voltage_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present min voltage limit is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the voltage limit of servo ID1.
  servo_read(1, 0x10, 2, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_min_voltage_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present voltage limit pack is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the max temperature limit of servo ID1.
  servo_read_max_temperature_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_max_temperature_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present max temperature limit is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the max angle limit of servo ID1.
  servo_read_max_angle_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_max_angle_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present max angle limit is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the min angle limit of servo ID1.
  servo_read_min_angle_limit(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_min_angle_limit_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present min angle limit is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the angle limit of servo ID1.
  servo_read(1, 0x0B, 4, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    Serial.print("the angle limit pack is: ");
    for(uint8_t i = 0; i < pack_len; i++)
    {
      Serial.print("0x%x ", pack[i]);
    }
    Serial.print("\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the return level of servo ID1.
  servo_read_return_level(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_return_level_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present return level is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the return delay time of servo ID1.
  servo_read_return_delay_time(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_return_delay_time_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present return delay time is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the baud rate of servo ID1.
  servo_read_baud_rate(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_baud_rate_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present baud rate is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the model information of servo ID1.
  servo_read_model_information(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_model_information_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present model information is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Read the firmware version of servo ID1.
  servo_read_firmware_version(1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Data sent successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_read_firmware_version_analysis(pack, &analysis_data);
    if (ret == SUCCESS) {
      Serial.print("present firmware version is: ");
      Serial.println(analysis_data);
    }
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if WRITE_TEST
  //Change the return delay time of servo ID1 to 500us.
  servo_set_return_delay_time(1, 250, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_return_delay_time_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set return delay time successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the return level of servo ID1 to respond to all instruction.
  servo_set_return_level(1, 2, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_return_level_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set return level successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the baud rate of servo ID1 to 1000000.
  servo_set_baud_rate(1, 7, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_baud_rate_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set baud rate successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the min angle limit of servo ID1 to 0°.
  servo_set_min_angle_limit(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_min_angle_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set min angle limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the max angle limit of servo ID1 to 300°.
  servo_set_max_angle_limit(1, 3000, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_max_angle_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max angle limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the angle limit of servo ID1 to 0°~300°.
  write_buffer[0] = 0 & 0xff;;
  write_buffer[1] = (0 >> 8) & 0xff;
  write_buffer[2] = 3000 & 0xff;
  write_buffer[3] = (3000 >> 8) & 0xff;

  servo_write(1, 0x0B, 4, write_buffer, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    Serial.print("servo set angle limit pack is: ");
    for(uint8_t i = 0; i < pack_len; i++)
    {
      Serial.print("0x%x ", pack[i]);
    }
    Serial.print("\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the max temperature limit of servo ID1 to 65℃.
  servo_set_max_temperature_limit(1, 65, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_max_temperature_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max temperature limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the max voltage limit of servo ID1 to 8.4V.
  servo_set_max_voltage_limit(1, 84, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_max_voltage_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max voltage limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the min voltage limit of servo ID1 to 3.5V.
  servo_set_min_voltage_limit(1, 35, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_min_voltage_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set min voltage limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the voltage limit of servo ID1 to 3.5~8.4V.
  write_buffer[0] = 84 & 0xff;
  write_buffer[1] = 35 & 0xff;
  
  servo_write(1, 0x10, 2, write_buffer, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    Serial.print("the voltage limit pack is: ");
    for(uint8_t i = 0; i < pack_len; i++)
    {
      Serial.print("0x%x ", pack[i]);
    }
    Serial.print("\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the max PWM limit of servo ID1 to 90%.
  servo_set_max_pwm_limit(1, 900, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_max_pwm_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max pwm limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the max current limit of servo ID1 to 900mA.
  servo_set_max_current_limit(1, 900, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_max_current_limit_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set max current limit successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the current shutdown time of servo ID1 to 500ms.
  servo_set_current_shutdown_time(1, 500, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_current_shutdown_time_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set current shutdown time successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the CW deadband of servo ID1 to 0.2°.
  servo_set_cw_deadband(1, 2, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_cw_deadband_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set cw deadband successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the CCW deadband of servo ID1 to 0.2°.
  servo_set_ccw_deadband(1, 2, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_ccw_deadband_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set ccw deadband successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the CW and CCW deadband of servo ID1 to 0.2°.
  write_buffer[0] = 2 & 0xff;
  write_buffer[1] = 2 & 0xff;

  servo_write(1, 0x18, 2, write_buffer, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    Serial.print("servo set cw deadband and ccw deadband pack is: ");
    for(uint8_t i = 0; i < pack_len; i++)
    {
      Serial.print("0x%x ", pack[i]);
    }
    Serial.print("\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the PWM punch of servo ID1 to 1%.
  servo_set_pwm_punch(1, 10, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_pwm_punch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set pwm punch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the position control P gain of servo ID1 to 5995.
  servo_set_position_control_p_gain(1, 5995, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_position_control_p_gain_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set position control p gain successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the position control D gain of servo ID1 to 5.
  servo_set_position_control_i_gain(1, 5, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_position_control_i_gain_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set position control i gain successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the position control D gain of servo ID1 to 145.
  servo_set_position_control_d_gain(1, 145, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_position_control_d_gain_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set position control d gain successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the position control PID gain of servo ID1 to 5995, 5, and 145 respectively.
  write_buffer[0] = 5995 & 0xff;
  write_buffer[1] = (5995 >> 8) & 0xff;
  write_buffer[2] = 5 & 0xff;
  write_buffer[3] = (5 >> 8) & 0xff;
  write_buffer[4] = 145 & 0xff;
  write_buffer[5] = (145 >> 8) & 0xff;

  servo_write(1, 0x1B, 6, write_buffer, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    Serial.print("servo set position control pid gain pack is: ");
    for(uint8_t i = 0; i < pack_len; i++)
    {
      Serial.print("0x%x ", pack[i]);
    }
    Serial.print("\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the LED condition of servo ID1 to turn on stall error, overheating error, and angle error.
  servo_set_led_condition(1, 38, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_led_condition_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set led condition successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the shutdown condition of servo ID1 to turn on stall error, overheating error, voltage error, and angle error.
  servo_set_shutdown_conditions(1, 39, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_shutdown_conditions_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set shutdown conditions successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the Flash switch of servo ID1 to ON.
  servo_set_flash_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_flash_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set flash switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the Flash switch of servo ID1 to OFF.
  servo_set_flash_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_flash_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set flash switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the LED switch of servo ID1 to ON.
  servo_set_led_switch(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_led_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set led switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the LED switch of servo ID1 to OFF.
  servo_set_led_switch(1, 0, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_led_switch_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set led switch successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

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

  //Change the control mode of servo ID1 to the current control mode.
  servo_set_control_mode(1, 2, order_buffer, &order_len);

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

  //Change the target current of servo ID1 to -400mA.
  servo_set_target_current(1, -400, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_target_current_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set target current successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(3000);

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

  //Change the control mode of servo ID1 to the velocity base position control mode.
  servo_set_control_mode(1, 1, order_buffer, &order_len);

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

  //Change the velocity base target velocity of servo ID1 to 360°/s.
  servo_set_velocity_base_target_velocity(1, 3600, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_velocity_base_target_velocity_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set velocity base target velocity successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the velocity base target ACC of servo ID1 to 500°/s².
  servo_set_velocity_base_target_acc(1, 10, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_velocity_base_target_acc_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set velocity base target acc successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the velocity base target DEC of servo ID1 to 50°/s².
  servo_set_velocity_base_target_dec(1, 1, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_velocity_base_target_dec_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set velocity base target dec successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the velocity base target position of servo ID1 to 150°.
  servo_set_velocity_base_target_position(1, 1500, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_velocity_base_target_position_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set velocity base target position successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

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

  //Change the control mode of servo ID1 to the time base position control mode.
  servo_set_control_mode(1, 0, order_buffer, &order_len);

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

  //Change the time base target ACC of servo ID1 to 5.
  servo_set_time_base_target_acc(1, 5, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_time_base_target_acc_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set time base target acc successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);

  //Change the time base target position and moving time of servo ID1 to 300°, 500ms respectively.
  servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1);

  if (Serial.available() > 0) {
    pack_len = Serial.available();
    Serial.readBytes(pack, pack_len);
    ret = servo_set_time_base_target_position_and_moving_time_analysis(pack);
    if (ret == SUCCESS)
      Serial.print("servo set time base target position and moving time successfully.\r\n");
  } else {
    Serial.print("Failed to readBytes data.\r\n");
  }
  delay(1000);
#endif

#if SYNC_WRITE_TEST
  servo.id_counts = 2;  //Sync write two servos
  servo.id[0] = 1;      //Set the ID of the first servo to 1
  servo.id[1] = 2;      //Set the ID of the second servo to 2

  //Change the torque switch of the servo ID1, ID2 to OFF respectively.
  servo.torque_switch[0] = 0;
  servo.torque_switch[1] = 0;

  servo_sync_write_torque_switch(servo, order_buffer, &order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1000);

  //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
  servo.control_mode[0] = 1;
  servo.control_mode[1] = 1;

  servo_sync_write_control_mode(servo, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1000);

  //Change the velocity base target velocity of the servo ID1, ID2 to 360°/s² and 720°/s², respectively.
  servo.velocity[0] = 3600;
  servo.velocity[1] = 7200;

  servo_sync_write_velocity_base_target_velocity(servo, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //Change the velocity base target ACC of servo ID1, ID2 to 500°/s² and 50°/s², respectively.
  servo.acc_velocity[0] = 10;
  servo.acc_velocity[1] = 1;

  servo_sync_write_velocity_base_target_acc(servo, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //Change the velocity base target DEC of servo ID1, ID2 to 50°/s² and 500°/s², respectively.
  servo.dec_velocity[0] = 1;
  servo.dec_velocity[1] = 10;

  servo_sync_write_velocity_base_target_dec(servo, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //Change the velocity base target velocity of the servo ID1, ID2 to 150° midpoint and 0° position, respectively.
  servo.position[0] = 1500;
  servo.position[1] = 0;

  servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //Change the velocity base target velocity of servo ID1 ,ID2 to 1800 and 3600, and the position to 3000 and 3000, respectively
  servo.velocity[0] = 1800;
  servo.velocity[1] = 3600;
  servo.position[0] = 3000;
  servo.position[1] = 3000;

  servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //SChange the velocity base target velocity of servo ID1 ,ID2 to 3600 and 3600, position to 0,0, acceleration to 500°/s², 500°/s², deceleration to 500°/s², 500°/s², respectively
  servo.velocity[0] = 3600;
  servo.velocity[1] = 3600;
  servo.position[0] = 0;
  servo.position[1] = 0;
  servo.acc_velocity[0] = 10;
  servo.acc_velocity[1] = 10;
  servo.dec_velocity[0] = 10;
  servo.dec_velocity[1] = 10;

  servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //Change the torque switch of the servo ID1, ID2 to OFF respectively.
  servo.torque_switch[0] = 0;
  servo.torque_switch[1] = 0;
  servo_sync_write_torque_switch(servo, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1000);

  //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
  servo.control_mode[0] = 0;
  servo.control_mode[1] = 0;
  servo_sync_write_control_mode(servo, order_buffer,&order_len);

  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Write successfully.");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1000);

  //Change the time base target ACC of servo ID1 to 1 and 5 respectively.
  servo.acc_velocity_grade[0] = 1;
  servo.acc_velocity_grade[1] = 5;

  servo_sync_write_time_base_target_acc(servo, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);

  //Change the time base target position and moving time of servo ID1 to 150° midpoint and 1s, 0° and 500ms respectively.
  servo.position[0] = 1500;
  servo.position[1] = 0;
  servo.time[0] = 1000;
  servo.time[1] = 500;

  servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Sync Write successfully.\r\n");
  } else {
    Serial.print("Failed to send data.\r\n");
  }
  delay(1000);
#endif

#if MODIFY_ID
  //Change the servo ID of servo ID1 to 2.
  servo_modify_known_id(1, 2, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Modify successfully.\r\n");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1000);
#endif

#if MODIFY_UNKNOWN_ID
  //Change the servo ID of the servo with an unknown ID to 1.
  servo_modify_unknown_id(1, order_buffer, &order_len);
  if (order_len == Serial.write(order_buffer, order_len)) {
    Serial.print("Modify successfully.\r\n");
  } else {
    Serial.print("Failed to send data.");
  }
  delay(1000);
#endif
}

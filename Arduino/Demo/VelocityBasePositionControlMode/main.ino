#include <Arduino.h>
#include "servo.h"

uint8_t ret;                              //Change Unknown Servo ID Test
uint8_t order_buffer[40];                 //Store Generated Instructions
uint8_t order_len;                        //Instruction Length
uint8_t pack[40];                         //Store the received status packet
uint8_t pack_len;                         //Response packet length.
uint16_t analysis_data;                   //Data parsed from the status packet
uint8_t write_buffer[20];                 //Write data to the memory table

struct servo_sync_parameter servo;

servo.id_counts = 2;            //Sync write two servos
servo.id[0] = 1;                //Set the ID of the first servo to 1
servo.id[1] = 2;                //Set the ID of the second servo to 2

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(1000000, SERIAL_8N1, 16, 17);  
}

void loop() {
    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;

    servo_sync_write_torque_switch(servo, order_buffer,&order_len);

    if (order_len == Serial2.write(order_buffer, order_len))
    {
        PRINTF("Sync Write torque witch successfully.\r\n");
    }
    else
    {
        PRINTF("Failed to send data.\r\n");
    }
    delay(1000);

    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    servo_sync_write_control_mode(servo, order_buffer,&order_len);

    if (order_len == Serial2.write(order_buffer, order_len))
    {
        PRINTF("Sync Write control mode successfully.\r\n");
    }
    else
    {
        PRINTF("Failed to send data.\r\n");
    }
    delay(1);

    //Change the velocity base target position of servo ID1 to 150°.
    servo_set_velocity_base_target_position(1, 1500, order_buffer,&order_len);
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
        ret = servo_set_velocity_base_target_position_analysis(pack);
        if(ret == SUCCESS)
            PRINTF("servo set velocity base target position successfully.\r\n");
    }
    else
    {
        PRINTF("Failed to read data.\r\n");
    }
    delay(1000);

    //In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s.
    write_buffer[0] = 3000 & 0xff;
    write_buffer[1] = (3000 >> 8) & 0xff;
    write_buffer[2] = 3600 & 0xff;
    write_buffer[3] = (3600 >> 8) & 0xff;

    servo_write(1, 0x35, 4, write_buffer, order_buffer, &order_len);
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
        PRINTF("servo pack is: ");
        for(uint8_t i = 0; i < pack_len; i++)
        {
            PRINTF("0x%x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("Failed to read data.\r\n");
    }
    delay(1000);

    //Change the velocity base target position, velocity base target velocity, velocity base target ACC,
    //and velocity base target DEC of servo ID1 to 0° position, 360°/s, 500°/s², and 50°/s², respectively.
    write_buffer[0] = 0 & 0xff;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3600 & 0xff;
    write_buffer[3] = (3600 >> 8) & 0xff;
    write_buffer[4] = 10 & 0xff;
    write_buffer[5] = 1 & 0xff;

    servo_write(1, 0x35, 6, write_buffer, order_buffer, &order_len);
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
        PRINTF("servo pack is: ");
        for(uint8_t i = 0; i < pack_len; i++)
        {
            PRINTF("0x%x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("Failed to read data.\r\n");
    }
    delay(1000);

    //In velocity base position control mode, let servo ID1 move to the 150° midpoint and let servo ID2 move to the 0° position.
    servo.position[0] = 1500;
    servo.position[1] = 0;

    servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
    if (order_len == Serial2.write(order_buffer, order_len))
    {
        PRINTF("Sync Write velocity base target position successfully.\r\n");
    }
    else
    {
        PRINTF("Failed to send data.\r\n");
    }
    delay(1000);

    //In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s,
    //and let servo ID2 move to the 150° position at a velocity base target velocity of 720°/s.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;
    servo.position[0] = 3000;
    servo.position[1] = 1500;

    servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
    if (order_len == Serial2.write(order_buffer, order_len))
    {
        PRINTF("Sync Write velocity base target position and velocity successfully.\r\n");
    }
    else
    {
        PRINTF("Failed to send data.\r\n");
    }
    delay(1000);

    //let servo ID1 move to the 0° position at a velocity base target velocity of 720°/s, a velocity base target ACC of 500°/s², and a velocity base target DEC of 50°/s².
    //Let servo ID2 move to the 300° position at a velocity base target velocity of 360°/s, a velocity base target ACC of 50°/s², and a velocity base target DEC of 500°/s².
    servo.velocity[0] = 7200;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 3000;
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 1;
    servo.dec_velocity[0] = 1;
    servo.dec_velocity[1] = 10;

    servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);
    if (order_len == Serial2.write(order_buffer, order_len))
    {
        PRINTF("Sync Write velocity base target acc,dec,velocity and position successfully.\r\n");
    }
    else
    {
        PRINTF("Failed to send data.\r\n");
    }
    delay(1000);
}

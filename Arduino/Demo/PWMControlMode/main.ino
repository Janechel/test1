#include <Arduino.h>
#include "PrimaryServo.h"

#define ESP32_BOARD

#if defined(ESP32_BOARD)
#define SERVO_SERIAL Serial2
void readFunction(uint8_t* pack, uint8_t pack_len) {
    SERVO_SERIAL.read(pack, pack_len);
}
#else
#define SERVO_SERIAL Serial
void readFunction(uint8_t* pack, uint8_t pack_len) {
    SERVO_SERIAL.readBytes(pack, pack_len);
}
#endif

uint8_t ret;                              //Change Unknown Servo ID Test
uint8_t order_buffer[40];                 //Store Generated Instructions
uint8_t order_len;                        //Instruction Length
uint8_t pack[40];                         //Store the received status packet
uint8_t pack_len;                         //Response packet length.
uint32_t analysis_data;                   //Data parsed from the status packet

void setup() {
    // put your setup code here, to run once:
    SERVO_SERIAL.begin(1000000, SERIAL_8N1, 16, 17);
}

void loop() {
    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer,&order_len);

    SERVO_SERIAL.write(order_buffer, order_len);
    delay(1);

    if (SERVO_SERIAL.available()>0)
    {
        pack_len = SERVO_SERIAL.available();
        readFunction(pack, pack_len);
        ret = primary_servo_set_torque_switch_analysis(pack);
        if(ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    delay(1000);

    //Change the control mode of servo ID1 to the PWM control mode.
    primary_servo_set_control_mode(1, 3, order_buffer,&order_len);

    SERVO_SERIAL.write(order_buffer, order_len);
    delay(1);

    if (SERVO_SERIAL.available()>0)
    {
        pack_len = SERVO_SERIAL.available();
        readFunction(pack, pack_len);
        ret = primary_servo_set_control_mode_analysis(pack);
        if(ret == PRIMARY_SUCCESS)
            PRINTF("write control mode complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    delay(1000);

    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer,&order_len);

    SERVO_SERIAL.write(order_buffer, order_len);
    delay(1);

    if (SERVO_SERIAL.available()>0)
    {
        pack_len = SERVO_SERIAL.available();
        readFunction(pack, pack_len);
        ret = primary_servo_set_torque_switch_analysis(pack);
        if(ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    delay(1000);

    //Change the target PWM of servo ID1 to -50%.
    primary_servo_set_target_pwm(1, -500, order_buffer,&order_len);

    SERVO_SERIAL.write(order_buffer, order_len);
    delay(1);

    if (SERVO_SERIAL.available()>0)
    {
        pack_len = SERVO_SERIAL.available();
        readFunction(pack, pack_len);
        ret = primary_servo_set_target_pwm_analysis(pack);
        if(ret == PRIMARY_SUCCESS)
            PRINTF("write target pwm complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    delay(3000);
}

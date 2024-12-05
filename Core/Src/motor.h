//
// Created by h on 2024/12/6.
//

#ifndef MOTOR_H
#define MOTOR_H

#endif //MOTOR_H
#include <stdint.h>

class Motor {
public:
    // 电机基本属性
    float ratio_;
    float delta_angle_;
    float angle_;
    float last_ecd_angle_;
    float delta_ecd_angle_;
    float rotate_speed_;
    float current_;
    float temp_;

    Motor(float ratio);
    virtual void updateMotorStatus(uint8_t rx_data[8]);
    static float linearMapping(int value, int in_min, int in_max, float out_min, float out_max);
};

// 子类：具体型号电机
class M3508Motor : public Motor {
public:
    M3508Motor();
    void updateMotorStatus(uint8_t rx_data[8]) override;
};

class M2006Motor : public Motor {
public:
    M2006Motor();
    void updateMotorStatus(uint8_t rx_data[8]) override;
};

class GM6020Motor : public Motor {
public:
    GM6020Motor();
    void updateMotorStatus(uint8_t rx_data[8]) override;
};


#define CAN_MOTOR_PIT_ID 0x205
#define CAN_MOTOR_YAW_ID 0x207

extern GM6020Motor pit_motor;
extern GM6020Motor yaw_motor;

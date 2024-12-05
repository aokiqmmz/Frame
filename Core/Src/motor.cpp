//
// Created by h on 2024/12/6.
//

#include "motor.h"
#include <cstdint>

// 基类构造函数
Motor::Motor(float ratio)
    : ratio_(ratio), delta_angle_(0.0f), angle_(0.0f), last_ecd_angle_(0.0f),
      delta_ecd_angle_(0.0f), rotate_speed_(0.0f), current_(0.0f), temp_(0.0f) {}

// 线性映射函数
float Motor::linearMapping(int value, int in_min, int in_max, float out_min, float out_max) {
    float out = (value - in_min) *1.0/ (in_max - in_min) * (out_max - out_min) + out_min;
    return out;
}

// 更新电机状态（虚函数，子类可以重写）
//这是基于GM6020手册的
void Motor::updateMotorStatus(uint8_t rx_data[8]) {
    int16_t ecd_angle_raw = (rx_data[0] << 8 | rx_data[1]);
    float ecd_angle_ = linearMapping(ecd_angle_raw, 0, 8191, 0.0, 360.0);

    int16_t rotate_speed_raw = (rx_data[2] << 8 | rx_data[3]);
    rotate_speed_ = linearMapping(rotate_speed_raw, 0, 65535, 0.0, 132.0);

    int16_t current_raw = (rx_data[4] << 8 | rx_data[5]);
    current_ = linearMapping(current_raw, -16384, 16364, -3.0, 3.0);

    //motor->temp_ = linearMapping(rx_data[6], 0, 255, 0.0, 125.0);

    angle_ = ecd_angle_/ratio_;
}




// 子类构造函数
M3508Motor::M3508Motor() : Motor(3591.0f / 187.0f) {}
void M3508Motor::updateMotorStatus(uint8_t rx_data[8]) {
    Motor::updateMotorStatus(rx_data);
}

M2006Motor::M2006Motor() : Motor(36.0f) {}
void M2006Motor::updateMotorStatus(uint8_t rx_data[8]) {
    Motor::updateMotorStatus(rx_data);
}

GM6020Motor::GM6020Motor() : Motor(1.0f) {}
void GM6020Motor::updateMotorStatus(uint8_t rx_data[8]) {
    Motor::updateMotorStatus(rx_data);

}

// 实例化电机对象
GM6020Motor pit_motor;
GM6020Motor yaw_motor;

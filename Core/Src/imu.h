//
// Created by h on 2024/12/6.
//

#ifndef IMU_H
#define IMU_H

#endif //IMU_H

#include <stdint.h>
#include <math.h>

class IMU {
public:
    struct Gyro {
        int16_t RATE_X;
        int16_t RATE_Y;
        int16_t RATE_Z;
        float angular_rate_x;
        float angular_rate_y;
        float angular_rate_z;
    };

    struct Accel {
        int16_t RATE_X;
        int16_t RATE_Y;
        int16_t RATE_Z;
        float accel_x;
        float accel_y;
        float accel_z;
    };

    struct EulerAngles {
        float p_accel;
        float r_accel;
        float y_accel;
        float p_gyro;
        float r_gyro;
        float y_gyro;
    };

    IMU();
    void init();
    void readGyroRate();
    void readAccelRate();
    void dataProcess(float k);
    float linearMapping(int value, int in_min, int in_max, float out_min, float out_max);
    float p, r, y;
    float p_degree;

private:
    void BMI088_ACCEL_NS_L();
    void BMI088_ACCEL_NS_H();
    void BMI088_GYRO_NS_L();
    void BMI088_GYRO_NS_H();
    void BMI088_WriteReg(uint8_t reg, uint8_t write_data);
    void BMI088_read_gyro_single_reg(uint8_t reg, uint8_t* return_data, uint8_t length);
    void BMI088_read_acc_single_reg(uint8_t reg, uint8_t* return_data, uint8_t length);
    void BMI088_write_gyro_single_reg(uint8_t reg, uint8_t data, uint8_t length);
    void BMI088_write_acc_single_reg(const uint8_t reg, const uint8_t data, uint8_t length);

    void dataProcess(float dt, float k);
    EulerAngles getEulerAngles() const;

    Gyro gyro_c;
    Accel accel_c;
    EulerAngles euler_imu;

    float dt;
    float k;
};

extern IMU imu;

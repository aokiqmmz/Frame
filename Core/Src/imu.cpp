//
// Created by h on 2024/12/6.
//

#include "imu.h"
#include "spi.h"
#include "main.h"

#define PI 3.1415926

IMU::IMU(float k) : p(0), r(0), y(0), p_degree(0), dt(0.001f) {
    this->k = k;
    gyro_c = {0, 0, 0, 0.0f, 0.0f, 0.0f};
    accel_c = {0, 0, 0, 0.0f, 0.0f, 0.0f};
    euler_imu = {0, 0, 0, 0.0f, 0.0f, 0.0f};
}

// SPI 接口函数
void IMU::BMI088_ACCEL_NS_L() {
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
}

void IMU::BMI088_ACCEL_NS_H() {
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
}

void IMU::BMI088_GYRO_NS_L() {
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);
}

void IMU::BMI088_GYRO_NS_H() {
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);
}

void IMU::BMI088_WriteReg(uint8_t reg, uint8_t write_data) {
    reg = reg & 0x7F;
    BMI088_ACCEL_NS_L();
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Transmit(&hspi1, &write_data, 1, HAL_MAX_DELAY);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    BMI088_ACCEL_NS_H();
}

// 初始化 BMI088 加速度计和陀螺仪
void IMU::init() {
    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7E, 0xB6);
    BMI088_ACCEL_NS_H();

    BMI088_GYRO_NS_L();
    BMI088_WriteReg(0x14, 0xB6);
    BMI088_GYRO_NS_H();

    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7D, 0x04);
    BMI088_ACCEL_NS_H();

    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x41, 0x02);
    BMI088_ACCEL_NS_H();
}

// 读取陀螺仪数据
void IMU::BMI088_read_gyro_single_reg(uint8_t reg, uint8_t* return_data, uint8_t length) {
    BMI088_GYRO_NS_L();
    uint8_t reg_send = reg | 0x80;
    HAL_SPI_Transmit(&hspi1, &reg_send, 1, 100);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Receive(&hspi1, return_data, length, 100);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    BMI088_GYRO_NS_H();
}

// 读取加速度计数据
void IMU::BMI088_read_acc_single_reg(uint8_t reg, uint8_t* return_data, uint8_t length) {
    BMI088_ACCEL_NS_L();
    uint8_t reg_send = reg | 0x80;
    HAL_SPI_Transmit(&hspi1, &reg_send, 1, 100);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Receive(&hspi1, return_data, 1, 100);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Receive(&hspi1, return_data, length, 100);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    BMI088_ACCEL_NS_H();
}

// 读取陀螺仪数据并转换
void IMU::readGyroRate() {
    uint8_t rx_data_gyro[6];
    BMI088_read_gyro_single_reg(0x02, &rx_data_gyro[0], 6);
    gyro_c.RATE_X = (int16_t)((uint16_t)rx_data_gyro[1] << 8 | (uint16_t)rx_data_gyro[0]);
    gyro_c.RATE_Y = (int16_t)((uint16_t)rx_data_gyro[3] << 8 | (uint16_t)rx_data_gyro[2]);
    gyro_c.RATE_Z = (int16_t)((uint16_t)rx_data_gyro[5] << 8 | (uint16_t)rx_data_gyro[4]);

    gyro_c.angular_rate_x = linearMapping(gyro_c.RATE_X, -32767, 32767, -2000, 2000) / 180 * 3.1415926; // rad/s
    gyro_c.angular_rate_y = linearMapping(gyro_c.RATE_Y, -32767, 32767, -2000, 2000) / 180 * 3.1415926;
    gyro_c.angular_rate_z = linearMapping(gyro_c.RATE_Z, -32767, 32767, -2000, 2000) / 180 * 3.1415926;
}

// 读取加速度计数据并转换
void IMU::readAccelRate() {
    uint8_t rx_data_accel[6];
    BMI088_read_acc_single_reg(0x12, &rx_data_accel[0], 6);
    accel_c.RATE_X = (int16_t)((uint16_t)rx_data_accel[1] << 8 | (uint16_t)rx_data_accel[0]);
    accel_c.RATE_Y = (int16_t)((uint16_t)rx_data_accel[3] << 8 | (uint16_t)rx_data_accel[2]);
    accel_c.RATE_Z = (int16_t)((uint16_t)rx_data_accel[5] << 8 | (uint16_t)rx_data_accel[4]);

    accel_c.accel_x = accel_c.RATE_X / 32768.0 * 1000.0 * pow(2, (2 + 1)) * 1.5; // mg
    accel_c.accel_y = accel_c.RATE_Y / 32768.0 * 1000.0 * pow(2, (2 + 1)) * 1.5;
    accel_c.accel_z = accel_c.RATE_Z / 32768.0 * 1000.0 * pow(2, (2 + 1)) * 1.5;
}

void IMU::dataProcess() {
    euler_imu.p_accel = -atan2(accel_c.accel_x, sqrt(accel_c.accel_y * accel_c.accel_y + accel_c.accel_z * accel_c.accel_z));
    euler_imu.r_accel = atan2(accel_c.accel_y, accel_c.accel_z);
    euler_imu.r_gyro = r + (gyro_c.angular_rate_x + gyro_c.angular_rate_y * sin(p) * sin(r) / cos(p) + gyro_c.angular_rate_z * cos(r) * sin(p) / cos(p)) * dt;
    euler_imu.p_gyro = p + (gyro_c.angular_rate_y * cos(r) - gyro_c.angular_rate_z * sin(r)) * dt;
    euler_imu.y_gyro = y + (gyro_c.angular_rate_y * sin(r) / cos(p) + gyro_c.angular_rate_z * cos(r) / cos(p)) * dt;

    p = euler_imu.p_gyro * (1 - k) + euler_imu.p_accel * k;
    r = euler_imu.r_gyro * (1 - k) + euler_imu.r_accel * k;
    y = euler_imu.y_gyro;

    p_degree = p * 180 / PI;
    y_degree = y * 180 / PI;
}

IMU::EulerAngles IMU::getEulerAngles() const {
    return euler_imu;
}

// 线性映射函数实现
float IMU::linearMapping(int value, int in_min, int in_max, float out_min, float out_max) {
    float out = (value - in_min) *1.0/ (in_max - in_min) * (out_max - out_min) + out_min;
    return out;
}

IMU imu(0.0004);
//
// Created by h on 2024/12/6.
//

#include "pid.h"

PID::PID(float kp, float ki, float kd, float i_max, float out_max) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->i_max = i_max;
    this->out_max = out_max;
}

float PID::PID_Calc(float ref, float fdb) {
    this->ref = ref;
    this->fdb = fdb;
    this->err = this->ref - this->fdb;

    // Proportional term
    this->pout = this->kp * this->err;

    // Integral term with limit
    this->err_sum += this->err;
    if (this->err_sum > this->i_max) this->err_sum = this->i_max;
    if (this->err_sum < -this->i_max) this->err_sum = -this->i_max;
    this->iout = this->ki * this->err_sum;

    // Derivative term
    this->dout = this->kd * (this->err - this->last_err);
    this->last_err = this->err;

    // Total output with limit
    this->output = this->pout + this->iout + this->dout;
    if (this->output > this->out_max) this->output = this->out_max;
    if (this->output < -this->out_max) this->output = -this->out_max;

    return this->output;
}

PID pid_pitch_angle(0.029999997, 7.00e-06, 0.5, 10000, 10000);
PID pid_pitch_speed(120000, 0.0029999998, 0.5, 10000, 10000);
PID pid_yaw_angle(0.03,0,0.25,10000,10000);
PID pid_yaw_speed(160000,0,100,10000,10000);
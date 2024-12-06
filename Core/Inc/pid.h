//
// Created by h on 2024/12/6.
//

#ifndef PID_H
#define PID_H

#endif //PID_H

class PID{
public:
  PID(float kp, float ki, float kd, float i_max, float out_max);
  float PID_Calc(float ref, float fdb);
  float output;

  float kp, ki, kd;
  float i_max, out_max;
  float ref, fdb;
  float err, err_sum, last_err;
  float pout, iout, dout;
};

extern PID pid_pitch_angle;
extern PID pid_pitch_speed;
extern PID pid_yaw_angle;
extern PID pid_yaw_speed;
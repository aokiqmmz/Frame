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

private:
  float kp, ki, kd;
  float i_max, out_max;
  float ref, fdb;
  float err, err_sum, last_err;
  float pout, iout, dout;
};


inline PID pid_pitch_angle(0.029999997, 7.00e-06, 0.5, 10000, 10000);
inline PID pid_pitch_speed(120000, 0.0029999998, 0.5, 10000, 10000);
inline PID pid_yaw_angle(0,0,0,10000,10000);
inline PID pid_yaw_speed(0,0,0,10000,10000);




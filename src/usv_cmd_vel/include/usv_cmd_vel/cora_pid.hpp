#ifndef _PID_WAMV_H
#define _PID_WAMV_H
#include <math.h>

namespace control {
class pid_wamv {
private:
double err, last_err, integral_error = 0, delta_err;
double ut;
int count = 0;

public:
double pid_control(double desire, double actual, double kp, double kd, double ki);
double pid_control_velocity(double desire, double actual, double kp, double kd, double ki);
double position_pid_control(double desire_pos, double actual_pos, double kp, double kd, double ki);
};

// 位置环PID控制
double pid_wamv::position_pid_control(double desire_pos, double actual_pos, double kp, double kd, double ki) {
err = desire_pos - actual_pos;
integral_error += err;
delta_err = err - last_err;

ut = kp * err + ki * integral_error + kd * delta_err;
if (count % 3 == 0) {
last_err = err;
}
count++;
return ut;
}

// 速度环PID控制
double pid_wamv::pid_control(double desire, double actual, double kp, double kd, double ki) {
err = desire - actual;
integral_error += err;
delta_err = err - last_err;



ut = kp * err + ki * integral_error + kd * delta_err;
if (count % 3 == 0) {
last_err = err;
}
count++;
return ut;
}

// 速度控制（备用）
double pid_wamv::pid_control_velocity(double desire, double actual, double kp, double kd, double ki) {
return pid_control(desire, actual, kp, kd, ki);
}
}

#endif
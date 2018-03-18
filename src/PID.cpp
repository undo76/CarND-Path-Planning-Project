#include "PID.h"

PID::PID(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

double PID::update(double cte) {
  i_error_ += cte;
  d_error_ = (cte - p_error_);
  p_error_ = cte;
  return totalError();
}

double PID::totalError() {
  return -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;
}

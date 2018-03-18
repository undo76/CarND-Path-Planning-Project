#pragma once

class PID {
public:
  PID(double Kp, double Ki, double Kd);

  double update(double cte);

private:
  double p_error_;
  double i_error_;
  double d_error_;

  double Kp_;
  double Ki_;
  double Kd_;

  double totalError();
};

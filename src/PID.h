#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID(double Kp, double Ki, double Kd, double KiMax, double KLpfAlpha);
  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  double run(double error);

 private:
  static double lpf(double newval, double oldval, double alpha);
  /**
   * PID Coefficients
   */
  const double Kp_;
  const double Ki_;
  const double Kd_;
  const double KiMax_;
  const double KLpfAlpha_;

  double integrator_{0.0};
  double last_error_{0.0f};
  double delta_error_{0.0};
  bool first_run_{true};
};

#endif  // PID_H

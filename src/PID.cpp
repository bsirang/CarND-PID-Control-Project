#include "PID.h"

#include <iostream>

PID::PID(double Kp, double Ki, double Kd, double KiMax, double KLpfAlpha) : Kp_{Kp}, Ki_{Ki}, Kd_{Kd}, KiMax_{KiMax}, KLpfAlpha_{KLpfAlpha} {}


double PID::lpf(double newval, double oldval, double alpha) {
  return (newval * alpha) + (oldval * (1.0 - alpha));
}

double PID::run(double error) {
    if (first_run_) {
      last_error_ = error;
      first_run_ = false;
    }

    // low pass filter noisy d term
    delta_error_ = lpf((error - last_error_), delta_error_, KLpfAlpha_);

    integrator_ += (Ki_ * error);
    // prevent integrator wind up
    integrator_ = std::min(KiMax_, std::max(-KiMax_, integrator_));

    double p = Kp_ * error;
    double i = integrator_;
    double d = Kd_ * delta_error_;

    std::cout << "p = " << p << " i = " << i << " d = " << d << std::endl;

    last_error_ = error;

    return p + i + d;
}

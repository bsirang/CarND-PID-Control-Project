#include "PID.h"

#include <iostream>
#include <limits>
#include <cmath>

PID::PID(double Kp, double Ki, double Kd, double KiMax, double KLpfAlpha) : Kp_{Kp}, Ki_{Ki}, Kd_{Kd}, KiMax_{KiMax}, KLpfAlpha_{KLpfAlpha} {}


double PID::lpf(double newval, double oldval, double alpha) {
  return (newval * alpha) + (oldval * (1.0 - alpha));
}

void PID::twiddle(double error_avg) {
  static constexpr unsigned num_params = 2;
  static double dp[num_params] = {0.05, 0.5};
  static double best_error = std::numeric_limits<double>::max();
  static unsigned current_idx = 0;
  static bool second_attempt = false;

  std::cout << "twiddle on param " << current_idx << " with error = " << error_avg << " and best = " << best_error << std::endl;
  std::cout << "before " << Kp_ << " " << Kd_ << " " << dp[0] << " " << dp[1] << std::endl;
  double p[num_params] = {Kp_, Kd_};

  if (error_avg < best_error) {
    best_error = error_avg;
    p[current_idx] += dp[current_idx];
    dp[current_idx] *= 1.1;
    current_idx = (current_idx + 1) % num_params;
    second_attempt = false;
    std::cout << "New best error " << best_error << std::endl;
  } else {
    if (!second_attempt) {
      p[current_idx] -= dp[current_idx];
      second_attempt = true;
    } else {
      // we didn't get a new best on our second second_attempt
      // add dp to go back to original value and then reduce dp
      p[current_idx] += dp[current_idx];
      dp[current_idx] *= 0.9;

      // Let's update the next parameter right away
      current_idx = (current_idx + 1) % num_params;
      p[current_idx] -= dp[current_idx];
    }
  }

  if (Kp_ != p[0]) {
    Kp_ = p[0];
    std::cout << "New Kp = " << Kp_ << std::endl;
  }
  if (Kd_ != p[1]) {
    Kd_ = p[1];
    std::cout << "New Kd = " << Kd_ << std::endl;
  }
  std::cout << "after " << Kp_ << " " << Kd_ << " " << dp[0] << " " << dp[1] << std::endl;

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

    // std::cout << "p = " << p << " i = " << i << " d = " << d << std::endl;

    last_error_ = error;
    error_accum_ += std::fabs(error);

    static bool first_twiddle = true;
    if (++sample_num_ >= kNumTwiddleSamples) {
      if (first_twiddle) {
        first_twiddle = false;
      } else {
        twiddle(error_accum_ / kNumTwiddleSamples);
      }
      sample_num_ = 0;
      error_accum_ = 0.0;
    }

    return p + i + d;
}

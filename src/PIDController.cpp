#include "motion/PIDController.h"

#include <algorithm>

namespace motion {

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd) {}

void PIDController::setSetpoint(double setpoint) noexcept {
    setpoint_ = setpoint;
}

double PIDController::setpoint() const noexcept {
    return setpoint_;
}

double PIDController::compute(double measurement, std::chrono::duration<double> dt) {
    const double error = setpoint_ - measurement;
    const double dtSeconds = dt.count();

    integral_ += error * dtSeconds;
    integral_ = std::clamp(integral_, -1e3, 1e3);

    double derivative = 0.0;
    if (hasPrevious_ && dtSeconds > 0.0) {
        derivative = (error - previousError_) / dtSeconds;
    }

    previousError_ = error;
    hasPrevious_ = true;

    return (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
}

void PIDController::reset() noexcept {
    integral_ = 0.0;
    previousError_ = 0.0;
    hasPrevious_ = false;
}

} // namespace motion

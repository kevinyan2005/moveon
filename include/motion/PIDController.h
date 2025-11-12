#pragma once

#include <chrono>

namespace motion {

// Basic PID loop that drives actuator effort toward the target position.
class PIDController {
public:
    PIDController(double kp, double ki, double kd);

    void setSetpoint(double setpoint) noexcept;
    double setpoint() const noexcept;

    double compute(double measurement, std::chrono::duration<double> dt);
    void reset() noexcept;

private:
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};

    double setpoint_{0.0};
    double integral_{0.0};
    double previousError_{0.0};
    bool hasPrevious_{false};
};

} // namespace motion

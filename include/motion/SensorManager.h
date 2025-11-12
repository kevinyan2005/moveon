#pragma once

#include <random>

namespace motion {

// Maintains probe readings and reports tolerance checks for the motion controller.
class SensorManager {
public:
    SensorManager();

    double readPosition() const noexcept;
    bool isWithinTolerance(double sample, double target, double tolerance) const noexcept;

    void applyActuatorFeedback(double effort);
    void setSimulatedPosition(double position) noexcept;

private:
    double simulatedPosition_{0.0};
    double noiseAmplitude_{0.001};
    mutable std::mt19937 rng_;
    mutable std::uniform_real_distribution<double> noise_;
};

} // namespace motion

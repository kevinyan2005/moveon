#include "motion/SensorManager.h"

#include <cmath>

namespace motion {

SensorManager::SensorManager()
    : rng_(std::random_device{}()), noise_(-noiseAmplitude_, noiseAmplitude_) {}

double SensorManager::readPosition() const noexcept {
    return simulatedPosition_ + noise_(rng_);
}

bool SensorManager::isWithinTolerance(double sample, double target, double tolerance) const noexcept {
    return std::abs(sample - target) <= tolerance;
}

void SensorManager::applyActuatorFeedback(double effort) {
    simulatedPosition_ += effort * 0.1;
}

void SensorManager::setSimulatedPosition(double position) noexcept {
    simulatedPosition_ = position;
}

} // namespace motion

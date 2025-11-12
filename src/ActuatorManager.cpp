#include "motion/ActuatorManager.h"

#include "motion/SensorManager.h"

#include <algorithm>

namespace motion {

void ActuatorManager::attachSensor(const std::shared_ptr<SensorManager>& sensor) {
    sensor_ = sensor;
}

void ActuatorManager::applyEffort(double effort) {
    lastEffort_ = clampEffort(effort);
    if (auto sensor = sensor_.lock()) {
        sensor->applyActuatorFeedback(lastEffort_);
    }
}

void ActuatorManager::holdPosition() {
    lastEffort_ = 0.0;
}

void ActuatorManager::stop() {
    lastEffort_ = 0.0;
}

double ActuatorManager::lastEffort() const noexcept {
    return lastEffort_;
}

double ActuatorManager::clampEffort(double effort) const noexcept {
    return std::clamp(effort, -maxEffort_, maxEffort_);
}

} // namespace motion

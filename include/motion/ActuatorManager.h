#pragma once

#include <memory>

namespace motion {

class SensorManager;

// Issues drive commands and informs sensors of simulated motion feedback.
class ActuatorManager {
public:
    ActuatorManager() = default;

    void attachSensor(const std::shared_ptr<SensorManager>& sensor);

    void applyEffort(double effort);
    void holdPosition();
    void stop();

    double lastEffort() const noexcept;

private:
    double clampEffort(double effort) const noexcept;

    double lastEffort_{0.0};
    std::weak_ptr<SensorManager> sensor_{};
    double maxEffort_{1.0};
};

} // namespace motion

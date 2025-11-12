#pragma once

#include <chrono>
#include <memory>
#include <optional>

namespace motion {

class PIDController;
class SensorManager;
class ActuatorManager;

// Coordinates PID, sensing, and actuation while enforcing settle/timeout rules from the motion-control design.
class MotionController {
public:
    enum class MoveStatus {
        Idle,
        Moving,
        Completed,
        TargetTimeout,
        SettleTimeout,
        Cancelled,
        Stopped
    };

    struct MoveConfig {
        double tolerance{0.05};
        std::chrono::milliseconds settleDuration{std::chrono::seconds(2)};
        std::chrono::milliseconds settleTimeout{std::chrono::seconds(10)};
        std::chrono::milliseconds targetTimeout{std::chrono::seconds(20)};
    };

    MotionController(std::unique_ptr<PIDController> pid,
                     std::shared_ptr<SensorManager> sensors,
                     std::shared_ptr<ActuatorManager> actuators);

    void setConfig(const MoveConfig& config);
    const MoveConfig& config() const noexcept;

    void startMove(double targetPosition);
    void update();
    void cancelMove();
    void stopMove();

    MoveStatus getStatus() const noexcept;
    bool isMoveComplete() const noexcept;
    bool hasFault() const noexcept;
    double target() const noexcept;

private:
    using Clock = std::chrono::steady_clock;

    void evaluateTimers(Clock::time_point now, bool inTolerance);
    void transitionToFault(MoveStatus status);

    MoveConfig config_{};
    std::unique_ptr<PIDController> pid_;
    std::shared_ptr<SensorManager> sensors_;
    std::shared_ptr<ActuatorManager> actuators_;

    MoveStatus status_{MoveStatus::Idle};
    double target_{0.0};

    Clock::time_point moveStart_{};
    std::optional<Clock::time_point> firstToleranceEntry_{};
    std::optional<Clock::time_point> settleStart_{};
    std::optional<Clock::time_point> lastUpdate_{};
};

} // namespace motion

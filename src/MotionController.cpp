#include "motion/MotionController.h"

#include "motion/ActuatorManager.h"
#include "motion/PIDController.h"
#include "motion/SensorManager.h"

#include <cmath>
#include <stdexcept>

namespace motion {

MotionController::MotionController(std::unique_ptr<PIDController> pid,
                                   std::shared_ptr<SensorManager> sensors,
                                   std::shared_ptr<ActuatorManager> actuators)
    : pid_(std::move(pid)), sensors_(std::move(sensors)), actuators_(std::move(actuators)) {
    if (!pid_ || !sensors_ || !actuators_) {
        throw std::invalid_argument("MotionController requires PID, sensor, and actuator instances");
    }
}

void MotionController::setConfig(const MoveConfig& config) {
    config_ = config;
}

const MotionController::MoveConfig& MotionController::config() const noexcept {
    return config_;
}

void MotionController::startMove(double targetPosition) {
    target_ = targetPosition;
    moveStart_ = Clock::now();
    firstToleranceEntry_.reset();
    settleStart_.reset();
    lastUpdate_ = moveStart_;
    status_ = MoveStatus::Moving;

    pid_->reset();
    pid_->setSetpoint(targetPosition);
}

void MotionController::update() {
    if (status_ != MoveStatus::Moving) {
        return;
    }

    const auto now = Clock::now();
    const double position = sensors_->readPosition();
    const std::chrono::duration<double> dt =
        lastUpdate_.has_value() ? now - *lastUpdate_ : std::chrono::duration<double>::zero();

    const double controlEffort = pid_->compute(position, dt);
    actuators_->applyEffort(controlEffort);
    lastUpdate_ = now;

    const bool inTolerance = sensors_->isWithinTolerance(position, target_, config_.tolerance);
    evaluateTimers(now, inTolerance);
}

void MotionController::cancelMove() {
    if (status_ == MoveStatus::Moving) {
        status_ = MoveStatus::Cancelled;
        actuators_->stop();
    }
}

void MotionController::stopMove() {
    if (status_ == MoveStatus::Moving) {
        status_ = MoveStatus::Stopped;
        actuators_->stop();
    }
}

MotionController::MoveStatus MotionController::getStatus() const noexcept {
    return status_;
}

bool MotionController::isMoveComplete() const noexcept {
    return status_ == MoveStatus::Completed;
}

bool MotionController::hasFault() const noexcept {
    return status_ == MoveStatus::TargetTimeout || status_ == MoveStatus::SettleTimeout;
}

double MotionController::target() const noexcept {
    return target_;
}

void MotionController::evaluateTimers(Clock::time_point now, bool inTolerance) {
    if (!firstToleranceEntry_ && (now - moveStart_) >= config_.targetTimeout) {
        transitionToFault(MoveStatus::TargetTimeout);
        return;
    }

    if (inTolerance) {
        if (!firstToleranceEntry_) {
            firstToleranceEntry_ = now;
        }
        if (!settleStart_) {
            settleStart_ = now;
        }

        if (firstToleranceEntry_ && (now - *firstToleranceEntry_) >= config_.settleTimeout) {
            transitionToFault(MoveStatus::SettleTimeout);
            return;
        }

        if (settleStart_ && (now - *settleStart_) >= config_.settleDuration) {
            status_ = MoveStatus::Completed;
            actuators_->holdPosition();
        }
    } else {
        settleStart_.reset();
        if (firstToleranceEntry_ && (now - *firstToleranceEntry_) >= config_.settleTimeout) {
            transitionToFault(MoveStatus::SettleTimeout);
        }
    }
}

void MotionController::transitionToFault(MoveStatus status) {
    status_ = status;
    actuators_->stop();
}

} // namespace motion

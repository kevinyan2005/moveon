#include "motion/ActuatorManager.h"
#include "motion/MotionController.h"
#include "motion/PIDController.h"
#include "motion/SensorManager.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

int main() {
    using namespace std::chrono_literals;

    auto sensors = std::make_shared<motion::SensorManager>();
    auto actuators = std::make_shared<motion::ActuatorManager>();
    actuators->attachSensor(sensors);

    auto pid = std::make_unique<motion::PIDController>(0.9, 0.05, 0.02);
    motion::MotionController controller(std::move(pid), sensors, actuators);

    motion::MotionController::MoveConfig config;
    config.tolerance = 0.01;
    config.settleDuration = 2s;
    config.settleTimeout = 10s;
    config.targetTimeout = 5s;
    controller.setConfig(config);

    auto statusToString = [](motion::MotionController::MoveStatus status) {
        switch (status) {
            case motion::MotionController::MoveStatus::Idle:
                return "Idle";
            case motion::MotionController::MoveStatus::Moving:
                return "Moving";
            case motion::MotionController::MoveStatus::Completed:
                return "Completed";
            case motion::MotionController::MoveStatus::TargetTimeout:
                return "TargetTimeout";
            case motion::MotionController::MoveStatus::SettleTimeout:
                return "SettleTimeout";
            case motion::MotionController::MoveStatus::Cancelled:
                return "Cancelled";
            case motion::MotionController::MoveStatus::Stopped:
                return "Stopped";
        }
        return "Unknown";
    };

    // Move #1: precision move to 10 mm that should complete normally.
    std::cout << "Starting precision move to 10 mm...\n";
    controller.startMove(10.0);
    while (controller.getStatus() == motion::MotionController::MoveStatus::Moving) {
        controller.update();
        std::this_thread::sleep_for(50ms);
    }
    std::cout << "First move status: " << statusToString(controller.getStatus()) << "\n";

    // Move #2: rotate to 180 degrees but cancel midway.
    std::cout << "Starting rotation move to 180 degrees (will cancel)...\n";
    controller.startMove(180.0);
    for (int i = 0; i < 40 && controller.getStatus() == motion::MotionController::MoveStatus::Moving; ++i) {
        if (i == 15) {
            std::cout << "Cancelling rotation move...\n";
            controller.cancelMove();
            break;
        }
        controller.update();
        std::this_thread::sleep_for(50ms);
    }
    std::cout << "Second move status: " << statusToString(controller.getStatus()) << "\n";

    // Move #3: short move that is stopped via stop command.
    std::cout << "Starting short move to 2 mm (will stop)...\n";
    controller.startMove(2.0);
    for (int i = 0; i < 40 && controller.getStatus() == motion::MotionController::MoveStatus::Moving; ++i) {
        if (i == 10) {
            std::cout << "Issuing stop command...\n";
            controller.stopMove();
            break;
        }
        controller.update();
        std::this_thread::sleep_for(50ms);
    }
    std::cout << "Third move status: " << statusToString(controller.getStatus()) << "\n";

    return 0;
}

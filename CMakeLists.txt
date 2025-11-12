cmake_minimum_required(VERSION 3.16)
project(MotionControl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(motion_app
    src/main.cpp
    src/MotionController.cpp
    src/PIDController.cpp
    src/SensorManager.cpp
    src/ActuatorManager.cpp
)

target_include_directories(motion_app PRIVATE include)

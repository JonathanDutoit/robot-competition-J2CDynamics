#include <main/differential_drive_controller.hpp>
#include <common/robot_config.hpp>

DifferentialDriveController::DifferentialDriveController(
    ISpeedControllable* leftController, 
    ISpeedControllable* rightController
) : leftController(leftController), rightController(rightController) {}

void DifferentialDriveController::setWheelVelocities(
    float leftWheelRadPerSec, float rightWheelRadPerSec
) {
    leftController->setVelocity(GEAR_RATIO * leftWheelRadPerSec);
    rightController->setVelocity(GEAR_RATIO * (-rightWheelRadPerSec));
}

void DifferentialDriveController::getWheelVelocities(
    float& leftWheelRadPerSec, float& rightWheelRadPerSec
) {
    leftWheelRadPerSec = leftController->getVelocity() / GEAR_RATIO;
    rightWheelRadPerSec = rightController->getVelocity() / GEAR_RATIO;
}
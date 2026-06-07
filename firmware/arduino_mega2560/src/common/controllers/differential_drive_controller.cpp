#include <common/controllers/differential_drive_controller.hpp>
#include <common_config.hpp>

DifferentialDriveController::DifferentialDriveController(
    ISpeedControllable* leftController, 
    ISpeedControllable* rightController
): leftController(leftController), rightController(rightController) {}

void DifferentialDriveController::init() {
    // Initialize both controllers to zero velocity
    leftController->setVelocity(0);
    rightController->setVelocity(0);
}

void DifferentialDriveController::update() {
    leftController->setVelocity(_targetLeftVel);
    rightController->setVelocity(_targetRightVel);
}

void DifferentialDriveController::setVelocities(
    float leftRadPerSec, float rightRadPerSec
) {
    _targetLeftVel = leftRadPerSec;
    _targetRightVel = rightRadPerSec;
}

void DifferentialDriveController::getVelocities(
    float& leftRadPerSec, float& rightRadPerSec
) {
    leftRadPerSec = leftController->getVelocity();
    rightRadPerSec = -rightController->getVelocity();
}
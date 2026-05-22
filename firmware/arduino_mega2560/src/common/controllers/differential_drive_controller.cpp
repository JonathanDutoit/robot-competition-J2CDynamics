#include <common/controllers/differential_drive_controller.hpp>
#include <common_config.hpp>

DifferentialDriveController::DifferentialDriveController(
    ISpeedControllable* leftController, 
    ISpeedControllable* rightController
): leftController(leftController), rightController(rightController) {}

void DifferentialDriveController::setVelocities(
    float leftRadPerSec, float rightRadPerSec
) {
    leftController->setVelocity(leftRadPerSec);
    rightController->setVelocity(-rightRadPerSec);
}

void DifferentialDriveController::getVelocities(
    float& leftRadPerSec, float& rightRadPerSec
) {
    leftRadPerSec = leftController->getVelocity();
    rightRadPerSec = -rightController->getVelocity();
}
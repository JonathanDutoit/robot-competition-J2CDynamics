#include <common/controllers/differential_drive_controller.hpp>
#include <common_config.hpp>

DifferentialDriveController::DifferentialDriveController(
    ISpeedControllable* leftController, ISpeedControllable* rightController,
    RobotCommand& cmd, RobotState& state
): 
_leftController(leftController), _rightController(rightController), 
_cmd(cmd), _state(state) {}

void DifferentialDriveController::init() {
    // Initialize both controllers to zero velocity
    _leftController->setVelocity(0);
    _rightController->setVelocity(0);
}

void DifferentialDriveController::update() {
    _leftController->setVelocity(_cmd.leftWheelVelocity);
    _rightController->setVelocity(_cmd.rightWheelVelocity);
}

void DifferentialDriveController::getVelocities() {
    _state.leftWheelVelocity = _leftController->getVelocity();
    _state.rightWheelVelocity = _rightController->getVelocity();
}
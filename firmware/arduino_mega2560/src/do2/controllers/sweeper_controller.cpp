#include <do2/controllers/sweeper_controller.hpp>
#include <do2/robot_config.hpp>

SweeperController::SweeperController(
    DRI0018DriverChannel* leftController, DRI0018DriverChannel* rightController,
    SweeperState &sweeperState
):
_leftController(leftController), _rightController(rightController), 
_sweeperState(sweeperState) {}

void SweeperController::init() {
    _stopBrushes();
}

void SweeperController::update() {
    switch (_sweeperState.mode) {
        case SweeperMode::Idle:
            _stopBrushes();
            break;

        case SweeperMode::Collect:
            _startCollecting();
            break;

        case SweeperMode::Dropoff:
            _startDropoff();
            break;

        default:
            _stopBrushes();
            break;
    }
}

void SweeperController::_startCollecting() {
    _leftController->setVelocity(DC_MOTOR_MAX_VELOCITY_RAD_SEC);
    _rightController->setVelocity(-DC_MOTOR_MAX_VELOCITY_RAD_SEC);
}

void SweeperController::_startDropoff() {
    _leftController->setVelocity(-DC_MOTOR_MAX_VELOCITY_RAD_SEC);
    _rightController->setVelocity(DC_MOTOR_MAX_VELOCITY_RAD_SEC);
}

void SweeperController::_stopBrushes() {
    _leftController->setVelocity(0.0f);
    _rightController->setVelocity(0.0f);
}
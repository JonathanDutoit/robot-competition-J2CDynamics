#include <do2/controllers/sweeper_controller.hpp>
#include <do2/robot_config.hpp>

SweeperController::SweeperController(
    DRI0018DriverChannel* leftController, DRI0018DriverChannel* rightController,
    Do2Command& cmd, Do2State& state
) : DifferentialDriveController(leftController, rightController, cmd, state), 
_cmd(cmd), _state(state) {}

void SweeperController::update() {
    switch (_sweeperState) {
        case SweeperControlState::Ready:
            if (_cmd.mode == SweeperMode::Collect) {
                _startCollecting();
            }
            else if (_cmd.mode == SweeperMode::Dropoff) {
                _startDropoff();
            } else {
                _stopBrushes();
            }
            break;

        case SweeperControlState::Collecting:
            if (_cmd.mode == SweeperMode::Idle) {
                _stopBrushes();
                _sweeperState = SweeperControlState::Ready;
            }

            if (_cmd.mode == SweeperMode::Dropoff) {
                _sweeperState = SweeperControlState::Dropoff;
            }
            break;

        case SweeperControlState::Dropoff:
            if (_cmd.mode == SweeperMode::Idle) {
                _stopBrushes();
                _sweeperState = SweeperControlState::Ready;
            }

            if (_cmd.mode == SweeperMode::Collect) {
                _sweeperState = SweeperControlState::Collecting;
            }
            break;
    }
}

void SweeperController::_startCollecting() {
    _leftController->setVelocity(DC_MOTOR_MAX_VELOCITY_RAD_SEC);
    _rightController->setVelocity(-DC_MOTOR_MAX_VELOCITY_RAD_SEC);
    _sweeperState = SweeperControlState::Collecting;
}

void SweeperController::_startDropoff() {
    _leftController->setVelocity(-DC_MOTOR_MAX_VELOCITY_RAD_SEC);
    _rightController->setVelocity(DC_MOTOR_MAX_VELOCITY_RAD_SEC);
    _sweeperState = SweeperControlState::Dropoff;
}

void SweeperController::_stopBrushes() {
    _leftController->setVelocity(0.0f);
    _rightController->setVelocity(0.0f);
}
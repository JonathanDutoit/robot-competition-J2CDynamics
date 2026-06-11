#include <do2/controllers/sweeper_controller.hpp>
#include <do2/robot_config.hpp>
#include <Arduino.h>

SweeperController::SweeperController(
    DRI0018DriverChannel* leftController,
    DRI0018DriverChannel* rightController,
    SweeperState &sweeperState,
    DuploCounter &duploCounter
):
_leftController(leftController),
_rightController(rightController),
_sweeperState(sweeperState),
_duploCounter(duploCounter)
{
    _unjamStep = 0;
    _unjamAttempts = 0;
}

void SweeperController::init() {
    _stopBrushes();
}

void SweeperController::update()
{
    switch (_sweeperState.mode)
    {
        case SweeperMode::Idle:
            _resetUnjamState();
            _stopBrushes();
            break;

        case SweeperMode::Collect:
            _handleCollect();
            break;

        case SweeperMode::Dropoff:
            _handleDropoff();
            break;

        case SweeperMode::Fault:
        default:
            _stopBrushes();
            break;
    }
}

void SweeperController::_handleCollect()
{
    if (_duploCounter.isBlocked())
    {
        _startUnjam();
        return;
    }

    _startCollecting();
}

void SweeperController::_handleDropoff()
{
    if (_duploCounter.isBlocked())
    {
        _startUnjam();
        return;
    }

    _startDropoff();
}

void SweeperController::_startUnjam()
{
    const uint32_t now = millis();

    switch (_unjamStep)
    {
        case 0:
            _leftController->setVelocity( DC_MOTOR_MAX_VELOCITY_RAD_SEC);
            _rightController->setVelocity(-DC_MOTOR_MAX_VELOCITY_RAD_SEC);
            _stepStartTime = now;
            _unjamStep = 1;
            break;

        case 1:
            if (now - _stepStartTime > UNJAM_STEP_DELAY_MS)
            {
                _stepStartTime = now;
                _unjamStep = 2;
            }
            break;

        case 2:
            _leftController->setVelocity(-DC_MOTOR_MAX_VELOCITY_RAD_SEC);
            _rightController->setVelocity( DC_MOTOR_MAX_VELOCITY_RAD_SEC);
            _stepStartTime = now;
            _unjamStep = 3;
            break;

        case 3:
            if (now - _stepStartTime > UNJAM_STEP_DELAY_MS)
            {
                _unjamAttempts++;

                if (_unjamAttempts >= UNJAM_MAX_ATTEMPTS)
                {
                    _sweeperState.mode = SweeperMode::Fault;
                    _stopBrushes();
                    _resetUnjamState();
                }
                else
                {
                    _unjamStep = 0; // retry cycle
                }
            }
            break;
    }
}

void SweeperController::_resetUnjamState()
{
    _unjamStep = 0;
    _unjamAttempts = 0;
}

void SweeperController::_startCollecting()
{
    _leftController->setVelocity(DC_MOTOR_MAX_VELOCITY_RAD_SEC);
    _rightController->setVelocity(-DC_MOTOR_MAX_VELOCITY_RAD_SEC);
}

void SweeperController::_startDropoff()
{
    _leftController->setVelocity(-DC_MOTOR_MAX_VELOCITY_RAD_SEC);
    _rightController->setVelocity(DC_MOTOR_MAX_VELOCITY_RAD_SEC);
}

void SweeperController::_stopBrushes()
{
    _leftController->setVelocity(0.0f);
    _rightController->setVelocity(0.0f);
}
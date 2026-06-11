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
    _lastDropoffTime = 0;
    _previousCount = 0;
    _unjamRequested = false;
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
    uint32_t now = millis();

    uint8_t count = _duploCounter.getCount();

    // Detect output event
    if (count > _previousCount)
    {
        _lastDropoffTime = now;
        _previousCount = count;
        _unjamRequested = false;   // reset recovery trigger
    }

    // Jam condition: nothing exiting for too long
    if ((now - _lastDropoffTime) > DROP_OFF_JAM_THRESHOLD_MS)
    {
        _unjamRequested = true;
    }

    if (_unjamRequested)
    {
        _startUnjam();
    }
    else
    {
        _startDropoff();
    }
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
            if (now - _stepStartTime > 300)
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
            if (now - _stepStartTime > 300)
            {
                _unjamAttempts++;

                if (_unjamAttempts >= 3)
                {
                    _sweeperState.mode = SweeperMode::Fault;
                    _stopBrushes();
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
    _unjamRequested = false;
    _unjamStep = 0;
    _unjamAttempts = 0;
    _previousCount = 0;
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
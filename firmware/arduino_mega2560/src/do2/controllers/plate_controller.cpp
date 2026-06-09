#include <do2/controllers/plate_controller.hpp>
#include <do2/robot_config.hpp>

PlateController::PlateController(SweeperState& sweeperState, RobotState& robotState) : 
_stepper(AccelStepper::DRIVER, PIN_STEPPER_STEP, PIN_STEPPER_DIR), 
_sweeperState(sweeperState), _robotState(robotState) {}

void PlateController::init()
{
    _stepper.setEnablePin(PIN_STEPPER_ENABLE);
    _stepper.setPinsInverted(false, false, true);

    _stepper.setMaxSpeed(STEPPER_MAX_SPEED_STEPS_PER_SEC);
    _stepper.setAcceleration(STEPPER_ACCELERATION_STEPS_PER_SEC2);

    _stepper.enableOutputs();
}

void PlateController::update()
{
    switch (_sweeperState.mode)
    {
        case SweeperMode::Idle:
            _previousMode = SweeperMode::Idle;
            _stepper.stop();
            break;

        case SweeperMode::Collect:
            _previousMode = SweeperMode::Collect;
            _stepper.run();
            break;

        case SweeperMode::Dropoff:
            if (_previousMode != SweeperMode::Dropoff) {
                rotateContinuous();
                _previousMode = SweeperMode::Dropoff;
            }
            _stepper.runSpeed();
            break;
    }
}

void PlateController::rotateQuarterTurn()
{
    if (_stepper.distanceToGo() != 0 || _sweeperState.mode != SweeperMode::Collect)
        return;
    _stepper.move(-QUARTER_TURN_STEPS);
}

void PlateController::rotateContinuous()
{
    if (_sweeperState.mode != SweeperMode::Dropoff)
        return;
    _stepper.setSpeed(DROP_OFF_SPEED);
}

void PlateController::stop()
{
    _stepper.stop();
}
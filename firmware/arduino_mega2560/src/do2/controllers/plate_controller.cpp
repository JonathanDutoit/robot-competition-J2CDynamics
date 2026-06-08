#include <do2/controllers/plate_controller.hpp>
#include <do2/robot_config.hpp>

PlateController::PlateController()
: _stepper(AccelStepper::DRIVER, PIN_STEPPER_STEP, PIN_STEPPER_DIR) {}

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
    _stepper.run();
}

void PlateController::rotateQuarterTurn()
{
    if (_stepper.distanceToGo() != 0)
        return;

    _targetPosition += QUARTER_TURN_STEPS;
    _stepper.moveTo(_targetPosition);
}
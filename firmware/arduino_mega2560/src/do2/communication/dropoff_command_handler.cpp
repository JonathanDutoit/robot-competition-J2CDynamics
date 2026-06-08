#include <do2/robot_config.hpp>
#include <do2/communication/dropoff_command_handler.hpp>

DropoffCommandHandler::DropoffCommandHandler(Do2Command& cmd) : _cmd(cmd) {}

bool DropoffCommandHandler::handleCommand(const char* command, const float* args) {
    if (strcmp(command, "DROPOFF") == 0) {
        _cmd.leftSweeperVelocity = -DC_MOTOR_CRUISE_VEL_RAD_SEC; // Cruise speed for dropoff
        _cmd.rightSweeperVelocity = -DC_MOTOR_CRUISE_VEL_RAD_SEC; // Cruise speed for dropoff
        _cmd.stepperVelocity = STEPPER_DROPOFF_VEL_STEPS_SEC; // Move stepper to dropoff velocity
        return true;
    }
    return false;
}
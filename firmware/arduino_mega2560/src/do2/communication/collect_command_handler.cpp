#include <do2/robot_config.hpp>
#include <do2/communication/collect_command_handler.hpp>

CollectCommandHandler::CollectCommandHandler(Do2Command& cmd) : _cmd(cmd) {}

bool CollectCommandHandler::handleCommand(const char* command, const float* args) {
    if (strcmp(command, "COLLECT") == 0) {
        _cmd.leftSweeperVelocity = DC_MOTOR_CRUISE_VEL_RAD_SEC; // Cruise speed for collection
        _cmd.rightSweeperVelocity = DC_MOTOR_CRUISE_VEL_RAD_SEC; // Cruise speed for collection
        return true;
    }
    return false;
}
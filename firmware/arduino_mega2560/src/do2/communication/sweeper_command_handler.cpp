#include <do2/communication/sweeper_command_handler.hpp>

SweeperCommandHandler::SweeperCommandHandler(Do2Command& cmd) : _cmd(cmd) {}

bool SweeperCommandHandler::handleCommand(const char* command, const float* args) {
    if (strcmp(command, "COLLECT") == 0) {
        _cmd.mode = SweeperMode::Collect;
        return true;
    }

    if (strcmp(command, "DROPOFF") == 0) {
        _cmd.mode = SweeperMode::Dropoff;
        return true;
    }

    if (strcmp(command, "IDLE") == 0) {
        _cmd.mode = SweeperMode::Idle;
        return true;
    }

    return false;
}
#include <do2/communication/sweeper_command_handler.hpp>

SweeperCommandHandler::SweeperCommandHandler(SweeperState& state) : _state(state) {}

HandlerResponse SweeperCommandHandler::handleCommand(const char* command, const float* args) {
    if (strcmp(command, "COLLECT") == 0) {
        _state.mode = SweeperMode::Collect;
        return { true, "" };
    }

    if (strcmp(command, "DROPOFF") == 0) {
        _state.mode = SweeperMode::Dropoff;
        return { true, "" };
    }

    if (strcmp(command, "IDLE") == 0) {
        _state.mode = SweeperMode::Idle;
        return { true, "" };
    }

    if (strcmp(command, "MODE") == 0) {
        return _sendSweeperMode();
    }

    return { false, "Unknown mode requested." };
}

HandlerResponse SweeperCommandHandler::_sendSweeperMode() {
    switch (_state.mode) {
        case SweeperMode::Idle:
            return { true, "MODE: IDLE" };
            break;
        case SweeperMode::Collect:
            return { true, "MODE: COLLECT" };
            break;
        case SweeperMode::Dropoff:
            return { true, "MODE: DROPOFF" };
            break;
        case SweeperMode::Fault:
            return { true, "MODE: FAULT" };
            break;
        default:
            return { false, "Unknown state." };
    }
}
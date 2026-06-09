#include <do2/communication/sweeper_command_handler.hpp>

SweeperCommandHandler::SweeperCommandHandler(SweeperState& state) : _state(state) {}

HandlerResponse SweeperCommandHandler::handleCommand(const char* command, const float* args) {
    if (strcmp(command, "COLLECT") == 0) {
        _state.mode = SweeperMode::Collect;
        return { true, "COLLECT command received" };
    }

    if (strcmp(command, "DROPOFF") == 0) {
        _state.mode = SweeperMode::Dropoff;
        return { true, "DROPOFF command received" };
    }

    if (strcmp(command, "IDLE") == 0) {
        _state.mode = SweeperMode::Idle;
        return { true, "IDLE command received" };
    }

    if (strcmp(command, "MODE") == 0) {
        return _sendSweeperMode();
    }

    return { false, "Unknown command" };
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
            return { false, "MODE: UNKNOWN" };
    }
}
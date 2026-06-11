#include <do2/communication/reset_command_handler.hpp>

ResetCommandHandler::ResetCommandHandler() {}

HandlerResponse ResetCommandHandler::handleCommand(const char* command, const float* args) {
    if (strcmp(command, "RESET") == 0) {
        resetFunc();
        return { true, "" };
    }
    return { false, "Unknown command requested." };
}
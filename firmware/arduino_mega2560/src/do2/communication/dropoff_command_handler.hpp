#pragma once

#include <common/communication/serial_bridge.hpp>
#include <do2/data/do2_command.hpp>

class DropoffCommandHandler : public ICommandHandler
{
    public:
        DropoffCommandHandler(Do2Command& cmd);
        bool handleCommand(const char* command, const float* args) override;

    private:
        Do2Command& _cmd;
};
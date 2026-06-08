#pragma once

#include <common/communication/serial_bridge.hpp>
#include <do2/data/do2_command.hpp>

class CollectCommandHandler : public ICommandHandler
{
    public:
        CollectCommandHandler(Do2Command& cmd);
        bool handleCommand(const char* command, const float* args) override;

    private:
        Do2Command& _cmd;
};
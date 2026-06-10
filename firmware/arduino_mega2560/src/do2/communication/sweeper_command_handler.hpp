#pragma once

#include <common/ICommandHandler.hpp>
#include <do2/data/do2_command.hpp>

class SweeperCommandHandler : public ICommandHandler
{
    public:
        SweeperCommandHandler(Do2Command& cmd);
        bool handleCommand(const char* command, const float* args) override;

    private:
        Do2Command& _cmd;
};
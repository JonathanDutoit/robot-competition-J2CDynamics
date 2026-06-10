#pragma once

#include <common/ICommandHandler.hpp>
#include <do2/data/sweeper_state.hpp>

class SweeperCommandHandler : public ICommandHandler
{
    public:
        SweeperCommandHandler(SweeperState& state);
        HandlerResponse handleCommand(const char* command, const float* args) override;

    private:
        HandlerResponse _sendSweeperMode();
        SweeperState& _state;
};
#pragma once

#include <common/ICommandHandler.hpp>

class ResetCommandHandler : public ICommandHandler
{
    public:
        ResetCommandHandler();
        HandlerResponse handleCommand(const char* command, const float* args) override;

    private:
        void(* resetFunc) (void) = 0; //declare reset function @ address 0
};
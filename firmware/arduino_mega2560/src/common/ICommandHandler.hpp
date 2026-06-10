#pragma once

class ICommandHandler
{
public:
    virtual bool handleCommand(const char* command, const float* args) = 0;
};
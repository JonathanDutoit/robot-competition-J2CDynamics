#pragma once

#include <Arduino.h>

struct HandlerResponse
{
    bool success;
    String message;
};

class ICommandHandler
{
public:
    virtual HandlerResponse handleCommand(const char* command, const float* args) = 0;
};
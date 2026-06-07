#pragma once

#include <Arduino.h>

struct RobotCommand
{
    float leftWheelVelocity = 0.0f;
    float rightWheelVelocity = 0.0f;
};
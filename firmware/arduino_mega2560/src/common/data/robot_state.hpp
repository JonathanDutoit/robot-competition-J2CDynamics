#pragma once

#include <Arduino.h>

struct RobotState
{
    float leftWheelVelocity = 0.0f;
    float rightWheelVelocity = 0.0f;

    uint8_t duploCount = 0;
};
#pragma once

#include <Arduino.h>
#include <common/data/robot_command.hpp>

struct Do2Command : public RobotCommand
{
    float leftSweeperVelocity = 0.0f;
    float rightSweeperVelocity = 0.0f;
    
    float stepperVelocity = 0.0f; // in rad/s
};
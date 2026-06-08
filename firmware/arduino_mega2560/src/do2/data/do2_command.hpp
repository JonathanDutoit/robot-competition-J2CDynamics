#pragma once

#include <Arduino.h>
#include <common/data/robot_command.hpp>

enum class SweeperMode
{
    Idle,
    Collect,
    Dropoff
};

struct Do2Command : public RobotCommand
{
    SweeperMode mode = SweeperMode::Idle;
    
    float stepperVelocity = 0.0f; // in rad/s
};
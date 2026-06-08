#pragma once

#include <Arduino.h>
#include <common/data/robot_state.hpp>

struct Do2State: public RobotState
{
    float leftSweeperVelocity = 0.0f;
    float rightSweeperVelocity = 0.0f;
};
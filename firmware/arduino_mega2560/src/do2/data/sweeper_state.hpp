#pragma once

#include <Arduino.h>
#include <common/data/robot_state.hpp>

enum class SweeperMode
{
    Idle,
    Collect,
    Dropoff,
    Fault
};

struct SweeperState: public RobotState
{
    SweeperMode mode = SweeperMode::Idle;
};
#pragma once

#include <Arduino.h>
#include <common/data/robot_command.hpp>
#include <do2/data/sweeper_mode.hpp>

struct Do2Command : public RobotCommand
{
    SweeperMode mode = SweeperMode::Idle;
};
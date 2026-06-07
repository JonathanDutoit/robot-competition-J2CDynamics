#pragma once

#include <Arduino.h>
#include <common/IUpdatable.hpp>
#include <common/data/robot_command.hpp>
#include <common/data/robot_state.hpp>

class SerialBridge: public IUpdatable {
    public:
        SerialBridge(RobotCommand&, RobotState&);
        void init();
        void update();
    
    private:
        bool _parseInput(String input, char* command, int& parsedCount);
        void _handleSpeedCommand(float leftVel, float rightVel);
        void _handleOdometryCommand();
        void _handleDuploCountCommand();
        
        RobotCommand& _cmd;
        RobotState& _state;
        char _buffer[64];
        uint8_t _index = 0;
};
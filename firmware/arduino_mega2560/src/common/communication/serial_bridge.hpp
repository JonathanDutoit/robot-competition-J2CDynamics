#pragma once

#include <Arduino.h>
#include <common/IUpdatable.hpp>
#include <common/data/robot_command.hpp>
#include <common/data/robot_state.hpp>

class SerialBridge: public IUpdatable {
    public:
        SerialBridge(RobotCommand&, RobotState&);
        void init() override;
        void update() override;
    
    private:
        bool _parseInput(
            String input, char* command, float& arg_left, 
            float& arg_right, int& parsedCount
        );
        void _setNextSpeedCommand(float leftVel, float rightVel);
        void _sendOdometry();
        void _sendDuploCountCommand();
        
        RobotCommand& _cmd;
        RobotState& _state;
        char _buffer[64];
        uint8_t _index = 0;
};
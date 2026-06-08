#pragma once

#include <Arduino.h>
#include <common_config.hpp>
#include <common/IUpdatable.hpp>
#include <common/data/robot_command.hpp>
#include <common/data/robot_state.hpp>
#include <common/ICommandHandler.hpp>

class SerialBridge: public IUpdatable {
    public:
        SerialBridge(RobotCommand&, RobotState&);
        void init() override;
        void update() override;
        bool registerHandler(ICommandHandler* handler);
    
    private:
        bool _parseInput(
            String input, char* command, float* args, int& parsedCount
        );
        bool _processCommand(
            const char* command, const float* args, const int& parsedCount
        );
        void _setNextSpeedCommand(float leftVel, float rightVel);
        void _sendOdometry();
        void _sendDuploCountCommand();

        RobotCommand& _cmd;
        RobotState& _state;

        char _buffer[64];
        uint8_t _index = 0;

        ICommandHandler* _handlers[MAX_COMMAND_HANDLERS];
        uint8_t _handlerCount = 0;
};
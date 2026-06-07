#pragma once

#include <common/ISpeedControllable.hpp>
#include <common/IUpdatable.hpp>
#include <common/data/robot_command.hpp>
#include <common/data/robot_state.hpp>

class DifferentialDriveController: public IUpdatable {
    public:
        DifferentialDriveController(
            ISpeedControllable* leftController, ISpeedControllable* rightController,
            RobotCommand& cmd, RobotState& state
        );
        void init() override;
        void update() override;
        void getVelocities();
    private:
        void _applyVelocities(const RobotCommand& cmds);
        ISpeedControllable* _leftController;
        ISpeedControllable* _rightController;
        RobotCommand& _cmd;
        RobotState& _state;
};
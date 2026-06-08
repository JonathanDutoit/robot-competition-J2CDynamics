#pragma once

#include <common/controllers/differential_drive_controller.hpp>
#include <do2/drivers/dri0018_driver_channel.hpp>
#include <do2/data/do2_command.hpp>
#include <do2/data/do2_state.hpp>

enum class SweeperControlState {
    Ready,
    Collecting,
    Dropoff
};

class SweeperController : public DifferentialDriveController
{
    public:
        SweeperController(
            DRI0018DriverChannel* leftController, DRI0018DriverChannel* rightController,
            Do2Command& cmd, Do2State& state
        );
        void update() override;

    private:
        void _startCollecting();
        void _startDropoff();
        void _stopBrushes();
        Do2Command& _cmd;
        Do2State& _state;
        SweeperControlState _sweeperState = SweeperControlState::Ready;
};
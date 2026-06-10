#pragma once

#include <common/IUpdatable.hpp>
#include <do2/drivers/dri0018_driver_channel.hpp>
#include <do2/data/sweeper_state.hpp>

class SweeperController: public IUpdatable
{
    public:
        SweeperController(
            DRI0018DriverChannel* leftController, 
            DRI0018DriverChannel* rightController,
            SweeperState& sweeperState
        );
        void init() override;
        void update() override;

    private:
        void _startCollecting();
        void _startDropoff();
        void _stopBrushes();
        DRI0018DriverChannel* _leftController;
        DRI0018DriverChannel* _rightController;
        SweeperState& _sweeperState;
};
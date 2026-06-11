#pragma once

#include <common/IUpdatable.hpp>
#include <do2/drivers/dri0018_driver_channel.hpp>
#include <do2/data/sweeper_state.hpp>
#include <common/sensors/duplo_counter.hpp>

class SweeperController: public IUpdatable
{
    public:
        SweeperController(
            DRI0018DriverChannel* leftController, 
            DRI0018DriverChannel* rightController,
            SweeperState& sweeperState,
            DuploCounter& duploCounter
        );
        void init() override;
        void update() override;

    private:
        void _startCollecting();
        void _startDropoff();
        void _stopBrushes();

        void _handleCollect();
        void _handleDropoff();
        void _startUnjam();
        void _resetUnjamState();

        DRI0018DriverChannel* _leftController;
        DRI0018DriverChannel* _rightController;
        
        SweeperState& _sweeperState;
        DuploCounter& _duploCounter;

        uint32_t _lastDropoffTime;
        uint8_t  _previousCount;

        bool     _unjamRequested;
        uint8_t  _unjamStep;
        uint8_t  _unjamAttempts;

        uint32_t _stepStartTime;
};
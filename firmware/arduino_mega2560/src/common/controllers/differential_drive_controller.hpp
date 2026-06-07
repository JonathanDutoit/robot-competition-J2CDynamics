#pragma once

#include <common/ISpeedControllable.hpp>
#include <common/IUpdatable.hpp>

class DifferentialDriveController: public IUpdatable {
    public:
        DifferentialDriveController(ISpeedControllable* leftController, 
                                    ISpeedControllable* rightController);
        void init();
        void update();
        void setVelocities(float leftRadPerSec, float rightRadPerSec);
        void getVelocities(float& leftRadPerSec, float& rightRadPerSec);
    private:
        ISpeedControllable* leftController;
        ISpeedControllable* rightController;
        float _targetLeftVel = 0;
        float _targetRightVel = 0;
};
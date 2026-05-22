#ifndef DIFFERENTIAL_DRIVE_CONTROLLER_HPP
#define DIFFERENTIAL_DRIVE_CONTROLLER_HPP

#include <common/ISpeedControllable.hpp>

class DifferentialDriveController {
    public:
        DifferentialDriveController(ISpeedControllable* leftController, 
                                    ISpeedControllable* rightController);
        void setVelocities(float leftRadPerSec, float rightRadPerSec);
        void getVelocities(float& leftRadPerSec, float& rightRadPerSec);
    private:
        ISpeedControllable* leftController;
        ISpeedControllable* rightController;
};

#endif
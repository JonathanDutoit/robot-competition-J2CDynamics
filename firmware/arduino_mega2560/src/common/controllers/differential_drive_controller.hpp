#ifndef DIFFERENTIAL_DRIVE_CONTROLLER_HPP
#define DIFFERENTIAL_DRIVE_CONTROLLER_HPP

#include <common/ISpeedControllable.hpp>

class DifferentialDriveController {
    public:
        DifferentialDriveController(ISpeedControllable* leftController, 
                                    ISpeedControllable* rightController);
        void setWheelVelocities(float leftWheelRadPerSec, float rightWheelRadPerSec);
        void getWheelVelocities(float& leftWheelRadPerSec, float& rightWheelRadPerSec);
    private:
        ISpeedControllable* leftController;
        ISpeedControllable* rightController;
};

#endif
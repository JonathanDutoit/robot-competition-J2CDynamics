#pragma once

#include <AccelStepper.h>
#include <common/IUpdatable.hpp>
#include <do2/data/sweeper_state.hpp>
#include <common/data/robot_state.hpp>

class PlateController: public IUpdatable {
public:
    PlateController(SweeperState& sweeperState, RobotState& robotState);
    void init() override;
    void update() override;
    void rotateTurnStep();
    void rotateContinuous();
    void stop();

private:
    AccelStepper _stepper;
    SweeperMode _previousMode = SweeperMode::Idle;
    SweeperState& _sweeperState;
    RobotState& _robotState;
};
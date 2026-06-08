#pragma once

#include <AccelStepper.h>
#include <common/IUpdatable.hpp>
#include <do2/data/do2_command.hpp>

class PlateController: public IUpdatable {
public:
    PlateController(Do2Command& cmd);
    void init() override;
    void update() override;
    void rotateQuarterTurn();
    void rotateContinuous();
    void stop();

private:
    AccelStepper _stepper;
    SweeperMode _previousMode = SweeperMode::Idle;
    Do2Command& _cmd;
};
#pragma once

#include <AccelStepper.h>
#include <common/IUpdatable.hpp>

class PlateController: public IUpdatable {
public:
    PlateController();
    void init() override;
    void update() override;
    void rotateQuarterTurn();

private:
    AccelStepper _stepper;
    long _targetPosition = 0;
};
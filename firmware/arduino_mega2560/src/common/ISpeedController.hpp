#ifndef ISPEEDCONTROLLER_HPP
#define ISPEEDCONTROLLER_HPP

#include <stdint.h>

struct ISpeedController
{
    virtual void setSpeed(int16_t targetRpm) = 0;
    virtual int16_t getSpeed() = 0;
};

#endif

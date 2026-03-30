#ifndef ISPEEDCONTROLLER_HPP
#define ISPEEDCONTROLLER_HPP

#include <stdint.h>

struct ISpeedController
{
    virtual void setVelocity(float rad_per_sec) = 0;
    virtual float getVelocity() = 0;
};

#endif

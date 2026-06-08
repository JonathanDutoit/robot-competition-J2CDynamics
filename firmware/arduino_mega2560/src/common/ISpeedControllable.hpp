#ifndef ISPEEDCONTROLLER_HPP
#define ISPEEDCONTROLLER_HPP

#include <stdint.h>

struct ISpeedControllable
{
    virtual void setVelocity(float rad_per_sec) = 0;
    virtual float getVelocity() const = 0;
    virtual uint8_t isReady() const = 0;
};

#endif

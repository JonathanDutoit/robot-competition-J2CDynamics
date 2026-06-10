#pragma once

class IUpdatable
{
public:
    virtual void init() = 0;
    virtual void update() = 0;
};
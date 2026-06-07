#pragma once

#include <Arduino.h>

class PeriodicTask {
    public:
        PeriodicTask(uint32_t periodMs);
        bool ready();
    private:
        uint32_t _period;
        uint32_t _lastExecution = 0;
};
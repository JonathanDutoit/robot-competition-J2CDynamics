#include <common/periodic_task.hpp>

PeriodicTask::PeriodicTask(uint32_t periodMs): _period(periodMs){}

bool PeriodicTask::ready() {
    uint32_t now = millis();

    if (now - _lastExecution >= _period) {
        _lastExecution = now;
        return true;
    }

    return false;
}
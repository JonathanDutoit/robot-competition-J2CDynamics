#pragma once

#include <Arduino.h>
#include <common/IUpdatable.hpp>

enum class DuploCounterState {
    NO_DUPLO,
    DUPLO_PRESENT,
    BLOCKED
};

class DuploCounter: public IUpdatable {
    public:
        DuploCounter(uint8_t sensorPin);
        void init() override;
        void update() override;
        uint8_t getCount() const;
        void reset();
        bool isBlocked() const;
    private:
        uint8_t _sensorPin;
        float _baselineAdc;
        uint8_t _count = 0;
        DuploCounterState _state = DuploCounterState::NO_DUPLO;
        uint32_t _ignoreUntil = 0;
        uint32_t _firstDetectTime = 0;
        uint32_t _lastDetectTime = 0;
        bool _wasDetected = false;
};
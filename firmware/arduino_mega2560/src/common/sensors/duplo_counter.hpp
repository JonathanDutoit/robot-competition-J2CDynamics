#pragma once

#include <Arduino.h>

enum class State {
    NO_DUPLO,
    DUPLO_PRESENT
};

class DuploCounter {
    public:
        DuploCounter(uint8_t sensorPin);
        void init();
        void update();
        uint8_t getCount() const;
        void reset();
    private:
        uint8_t _sensorPin;
        float _baselineAdc;
        uint8_t _count = 0;
        State _state = State::NO_DUPLO;
};
#include <sensors/duplo_counter.hpp>
#include <common_config.hpp>

DuploCounter::DuploCounter(uint8_t sensorPin): _sensorPin(sensorPin){}

void DuploCounter::init() {
    pinMode(_sensorPin, INPUT);

    // Read the sensor multiple times to get a stable baseline ADC value
    float sum = 0;
    for (int i = 0; i < BASELINE_MEASUREMENT_COUNT; ++i) {
        sum += analogRead(_sensorPin);
        delay(5);
    }
    _baselineAdc = sum / BASELINE_MEASUREMENT_COUNT;
}

void DuploCounter::update() {

    uint16_t raw = analogRead(_sensorPin);
    float delta = raw - _baselineAdc;

    switch (_state) {

        case State::NO_DUPLO:
            if (delta > DUPLO_DETECTION_DELTA) {
                _state = State::DUPLO_PRESENT;
            }
            break;

        case State::DUPLO_PRESENT:
            if (delta < DUPLO_RELEASE_DETECTION_DELTA) {
                _count++;
                _state = State::NO_DUPLO;
            }
            break;
    }
}

uint8_t DuploCounter::getCount() const {
    return _count;
}

void DuploCounter::reset() {
    _count = 0;
}


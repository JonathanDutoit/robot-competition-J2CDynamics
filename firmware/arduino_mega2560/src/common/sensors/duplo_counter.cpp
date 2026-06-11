#include <do2/robot_config.hpp>
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
    uint32_t now = millis();
    if (now < _ignoreUntil) {
        return;
    }

    uint16_t raw = analogRead(_sensorPin);
    float delta = raw - _baselineAdc;
    bool detected = delta > DUPLO_DETECTION_DELTA;
    bool released = delta < DUPLO_RELEASE_DETECTION_DELTA;

    switch (_state) {

        case DuploCounterState::NO_DUPLO:
            if (detected) {
                _state = DuploCounterState::DUPLO_PRESENT;
                _firstDetectTime = now;
                _lastDetectTime = now;
                _wasDetected = true;
            }
            break;

        case DuploCounterState::DUPLO_PRESENT:
            if (detected) {
                _lastDetectTime = now;

                if (now - _firstDetectTime > COLLECTING_JAM_THRESHOLD_MS) {
                    _state = DuploCounterState::BLOCKED;
                }
            } else if (released) {
                 _state = DuploCounterState::NO_DUPLO;

                if (_wasDetected) {
                    _count++;
                    _ignoreUntil = now + COLLECTING_JAM_THRESHOLD_MS;
                }

                _wasDetected = false;
            }
            break;
        
        case DuploCounterState::BLOCKED:
            // stay blocked until object disappears
            if (!detected) {
                _state = DuploCounterState::NO_DUPLO;
                _wasDetected = false;
            }
            break;
    }
}

uint8_t DuploCounter::getCount() const {
    return _count;
}

bool DuploCounter::isBlocked() const
{
    return _state == DuploCounterState::BLOCKED;
}

void DuploCounter::reset() {
    _count = 0;
}


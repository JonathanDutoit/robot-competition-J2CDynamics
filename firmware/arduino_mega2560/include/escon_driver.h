#ifndef MAXON_DRIVER_H
#define MAXON_DRIVER_H

#include <stdint.h>

class EsconDriver {
    public:
        EsconDriver(uint8_t pwmDigitalInputPin, uint8_t enableDigitalInputPin, 
                    uint8_t directionDigitalInputPin, uint8_t readyDigitalInputPin, 
                    uint8_t speedAnalogOutputPin, uint8_t currentAnalogOutputPin);
        void init();
        uint8_t isReady();
        void setSpeed(int16_t targetRpm);
        int16_t getAveragedSpeed();
        int16_t getAveragedCurrent();
    private:
        uint8_t _pwmPin, _enPin, _dirPin, _readyPin, _speedPin, _currPin;
        void configurePWM();
        uint8_t rpmToDuty(int16_t rpm);
};

#endif
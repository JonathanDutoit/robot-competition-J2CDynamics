#ifndef MAXON_DRIVER_H
#define MAXON_DRIVER_H

#include <stdint.h>

class EsconDriver {
    public:
        EsconDriver(uint8_t pwmDigitalInputPin, uint8_t enableDigitalInputPin, 
                    uint8_t directionDigitalInputPin, uint8_t readyDigitalInputPin, 
                    uint8_t speedAnalogOutputPin, uint8_t currentAnalogOutputPin);
        void init();
        bool isReady();
        void setSpeed(int speed);
        void enableMotion();
        void disableMotion();
        float getAveragedSpeed();
        float getAveragedCurrent();
    private:
        uint8_t _pwmPin, _enPin, _dirPin, _readyPin, _speedPin, _currPin;
        void configurePWM();
        int16_t rpmToDuty(int16_t rpm);
};

#endif
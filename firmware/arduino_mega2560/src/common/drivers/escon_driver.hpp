#ifndef MAXON_DRIVER_H
#define MAXON_DRIVER_H

#include <stdint.h>
#include <common/ISpeedController.hpp>

class EsconDriver : public ISpeedController {
    public:
        EsconDriver(uint8_t pwmDigitalInputPin, uint8_t enableDigitalInputPin, 
                    uint8_t directionDigitalInputPin, uint8_t readyDigitalInputPin, 
                    uint8_t speedAnalogOutputPin, uint8_t currentAnalogOutputPin);
        void init();
        void setVelocity(float rad_per_sec) override;
        float getVelocity() override;
        float getCurrent();
    private:
        uint8_t _pwmPin, _enPin, _dirPin, _readyPin, _speedPin, _currPin;
        uint8_t isReady();
        void configurePWM();
};

#endif
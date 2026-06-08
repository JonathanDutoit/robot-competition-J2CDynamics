#ifndef ESCON_DRIVER_H
#define ESCON_DRIVER_H

#include <stdint.h>
#include <common/ISpeedControllable.hpp>

class EsconDriver : public ISpeedControllable {
    public:
        EsconDriver(uint8_t pwmDigitalInputPin, uint8_t enableDigitalInputPin, 
                    uint8_t directionDigitalInputPin, uint8_t readyDigitalInputPin, 
                    uint8_t speedAnalogOutputPin, uint8_t currentAnalogOutputPin);
        void init();
        void setVelocity(float rad_per_sec) override;
        float getVelocity() const override;
        uint8_t isReady() const override;
        float getCurrent() const;
    private:
        uint8_t _pwmPin, _enPin, _dirPin, _readyPin, _speedPin, _currPin;
        void configurePWM();
};

#endif
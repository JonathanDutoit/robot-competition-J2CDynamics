#ifndef ESCON_DRIVER_H
#define ESCON_DRIVER_H

#include <stdint.h>
#include <common/ISpeedControllable.hpp>
#include <common_config.hpp>

class EsconDriver : public ISpeedControllable {
    public:
        EsconDriver(uint8_t pwmDigitalInputPin, uint8_t enableDigitalInputPin, 
                    uint8_t directionDigitalInputPin, uint8_t readyDigitalInputPin, 
                    uint8_t speedAnalogOutputPin, uint8_t currentAnalogOutputPin);
        void init();
        void setVelocity(float rad_per_sec) override;
        float getVelocity() const override;
        uint8_t isReady() const;
        float getCurrent() const;
        float EsconDriver::measureZeroVoltage(int samples) const; 
        void setZeroVoltage(float v) { _zeroVoltage = v; }
    private:
        uint8_t _pwmPin, _enPin, _dirPin, _readyPin, _speedPin, _currPin;
        void configurePWM();
        float _zeroVoltage{ESCON_VELOCITY_ZERO_VOLTAGE};  // default, overridable
};

#endif
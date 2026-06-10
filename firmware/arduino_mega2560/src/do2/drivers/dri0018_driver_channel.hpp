#ifndef DRI0018_DRIVER_CHANNEL_H
#define DRI0018_DRIVER_CHANNEL_H

#include <stdint.h>
#include <common/ISpeedControllable.hpp>

class DRI0018DriverChannel : public ISpeedControllable {
    public:
        DRI0018DriverChannel(
            uint8_t pwmDigitalInputPin, uint8_t directionDigitalInputPin
        );
        void init();
        void setVelocity(float rad_per_sec) override;
        float getVelocity() const override {return 0.0f;} // Not implemented for DRI0018
    private:
        void configurePWM();
        uint8_t _pwmPin, _dirPin;
};

#endif
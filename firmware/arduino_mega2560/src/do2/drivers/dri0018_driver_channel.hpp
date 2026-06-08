#ifndef DRI0018_DRIVER_CHANNEL_H
#define DRI0018_DRIVER_CHANNEL_H

#include <stdint.h>
#include <common/ISpeedControllable.hpp>

class DRI0018DriverChannel : public ISpeedControllable {
    public:
        static constexpr uint8_t INVALID_PIN = 255;
        DRI0018DriverChannel(
            uint8_t pwmDigitalInputPin, uint8_t directionDigitalInputPin,
            uint8_t currentSenseDigitalInputPin = INVALID_PIN
        );
        void init();
        void setVelocity(float rad_per_sec) override;
        float getVelocity() const override {return 0.0f;} // Not implemented for DRI0018
        uint8_t isReady() const override;
    private:
        uint8_t _pwmPin, _dirPin, _currPin;
        void configurePWM();
};

#endif
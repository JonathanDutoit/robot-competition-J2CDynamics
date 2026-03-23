#include <escon_driver.h>
#include <Arduino.h>

EsconDriver::EsconDriver(uint8_t pwmDigitalInputPin, uint8_t enableDigitalInputPin, 
                        uint8_t directionDigitalInputPin, uint8_t readyDigitalInputPin, 
                        uint8_t speedAnalogOutputPin, uint8_t currentAnalogOutputPin):
    _pwmPin(pwmDigitalInputPin), _dirPin(directionDigitalInputPin), 
    _enPin(enableDigitalInputPin), _readyPin(readyDigitalInputPin), 
    _speedPin(speedAnalogOutputPin), _currPin(currentAnalogOutputPin) {}

void EsconDriver::init() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_enPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    // Configure ready pin as input with pull-up resistor
    // 0 = ready, 1 = not ready (assuming Ready is set to active HIGH in Motion Studio)
    pinMode(_readyPin, INPUT_PULLUP);

    configurePWM();

    disableMotion(); // Ensure motor is stopped on init
    digitalWrite(_dirPin, LOW); // Default direction (e.g. CCW)
    analogWrite(_pwmPin, 0); // Start with 0% duty cycle (stopped)
}
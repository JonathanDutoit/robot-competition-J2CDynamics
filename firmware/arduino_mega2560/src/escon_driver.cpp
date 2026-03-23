#include <escon_driver.h>

#include <Arduino.h>
#include "robot_config.h"

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

    configurePWM(); // Set PWM frequency for motor control

    disableMotion(); // Ensure motor is stopped on init
    digitalWrite(_dirPin, LOW); // Default direction (e.g. CCW)
    analogWrite(_pwmPin, 0); // Start with 0% duty cycle (stopped)
}

void EsconDriver::configurePWM() {
    // For Arduino Mega 2560, we can use the default PWM frequency at 490 Hz or change
    // one of the timers' prescaler to achieve 4 kHz.
    // The Arduino Mega 2560 has several timers:
    // - Timer0: Pins 4, 13 (used for millis(), delay(), etc. - avoid changing this)
    // - Timer1: Pins 11, 12 (used for Servo library - avoid changing this if using Servo)
    // - Timer2: Pins 9, 10 (used for tone() - avoid changing this if using tone)
    // - Timer3: Pins 2, 3, 5 (available for PWM frequency change)
    // - Timer4: Pins 6, 7, 8 (available for PWM frequency change)
    if (_pwmPin == 6 || _pwmPin == 7 || _pwmPin == 8) {
        TCCR4B &= ~0b111; // Clear prescaler bits
        TCCR4B |= ARDUINO_PWM_MOTOR_PRESCALER; // Set prescaler for Timer4 to achieve ~4 kHz PWM frequency
    } else if (_pwmPin == 2 || _pwmPin == 3 || _pwmPin == 5) {
        TCCR3B &= ~0b111; // Clear prescaler bits
        TCCR3B |= ARDUINO_PWM_MOTOR_PRESCALER; // Set prescaler for Timer3 to achieve ~4 kHz PWM frequency
    }
}
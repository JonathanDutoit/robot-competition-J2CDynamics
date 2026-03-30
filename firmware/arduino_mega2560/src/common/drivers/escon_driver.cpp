#include <common/drivers/escon_driver.hpp>

#include <Arduino.h>
#include <common/robot_config.hpp>

EsconDriver::EsconDriver(uint8_t pwmDigitalInputPin, uint8_t enableDigitalInputPin, 
                        uint8_t directionDigitalInputPin, uint8_t readyDigitalInputPin, 
                        uint8_t speedAnalogOutputPin, uint8_t currentAnalogOutputPin):
    _pwmPin(pwmDigitalInputPin), _enPin(enableDigitalInputPin), 
    _dirPin(directionDigitalInputPin), _readyPin(readyDigitalInputPin), 
    _speedPin(speedAnalogOutputPin), _currPin(currentAnalogOutputPin) {}

void EsconDriver::init() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_enPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_readyPin, INPUT_PULLUP);

    configurePWM(); // Set PWM frequency for motor control

    digitalWrite(_enPin, LOW); // Ensure motor is disabled at startup
    digitalWrite(_dirPin, MOTOR_CCW_DIRECTION); // Default direction

    setVelocity(0); // Start with 0 speed but ensure PWM is set at 10% duty cycle to avoid ESCON error
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
    if (_pwmPin == 2 || _pwmPin == 5) {
        TCCR3B &= ~0b111; // Clear prescaler bits
        TCCR3B |= ARDUINO_PWM_MOTOR_PRESCALER; // Set prescaler for Timer3 to achieve ~4 kHz PWM frequency
    }
}

uint8_t EsconDriver::isReady() {
    // Ready pin is active HIGH in Motion Studio -> 0 = ready, 1 = not ready
    // see https://support.maxongroup.com/hc/en-us/articles/360008666420-ESCON-Digital-Output-Wiring
    return digitalRead(_readyPin) == LOW;
}

void EsconDriver::setVelocity(float rad_per_sec) {
    if (!isReady()) {
        digitalWrite(_enPin, LOW);
        analogWrite(_pwmPin, 0);
        return;
    } else {
        digitalWrite(_enPin, HIGH);
    }

    // Direction Logic
    if (rad_per_sec >= 0) {
        digitalWrite(_dirPin, MOTOR_CCW_DIRECTION);
    } else {
        digitalWrite(_dirPin, MOTOR_CW_DIRECTION);
        rad_per_sec = -rad_per_sec;
    }

    // Saturation
    if (rad_per_sec > MOTOR_MAX_VELOCITY_RAD_SEC) {
        rad_per_sec = MOTOR_MAX_VELOCITY_RAD_SEC;
    }

    // Normalize to [0, 1]
    rad_per_sec = rad_per_sec / MOTOR_MAX_VELOCITY_RAD_SEC;

    // Convert RPM to Duty Cycle (0-255) and write to PWM pin
    uint8_t duty = static_cast<uint8_t>(
        rad_per_sec * (ESCON_PWM_DUTY_CYCLE_MAX - ESCON_PWM_DUTY_CYCLE_MIN) 
        + ESCON_PWM_DUTY_CYCLE_MIN
    );
    analogWrite(_pwmPin, duty);
}

float EsconDriver::getVelocity() {
    if (!isReady()) {
        return 0; // If not ready, return 0 speed
    }
    float adcValue = static_cast<float>(analogRead(_speedPin));
    float voltage = (adcValue / ARDUINO_ADC_MAX_COUNT) * ARDUINO_ADC_VOLTAGE_REF; // Convert ADC value to voltage (0-5V)
    voltage = voltage - ESCON_VELOCITY_ZERO_VOLTAGE; // Center around 0V (2V is 0 speed)
    return voltage * 2 * MOTOR_MAX_VELOCITY_RAD_SEC / ESCON_MAX_OUTPUT_VOLTAGE; // in radians per second
}

float EsconDriver::getCurrent() {
    if (!isReady()) {
        return 0; // If not ready, return 0 current
    }
    float adcValue = static_cast<float>(analogRead(_currPin));
    float voltage = (adcValue / ARDUINO_ADC_MAX_COUNT) * ARDUINO_ADC_VOLTAGE_REF; // Convert ADC value to voltage (0-5V)
    voltage = voltage - ESCON_CURRENT_ZERO_VOLTAGE; // Center around 0V (2V is 0 current)
    return voltage * 2 * MOTOR_MAX_CURRENT_A / ESCON_MAX_OUTPUT_VOLTAGE; // in amps
}
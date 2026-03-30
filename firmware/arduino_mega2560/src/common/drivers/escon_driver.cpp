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

    setSpeed(0); // Start with 0 speed but ensure PWM is set at 10% duty cycle to avoid ESCON error
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

void EsconDriver::setSpeed(int16_t targetCmd) {
    if (!isReady()) {
        digitalWrite(_enPin, LOW);
        analogWrite(_pwmPin, 0);
        return;
    } else {
        digitalWrite(_enPin, HIGH);
    }

    // Direction Logic
    if (targetCmd >= 0) {
        digitalWrite(_dirPin, MOTOR_CCW_DIRECTION);
    } else {
        digitalWrite(_dirPin, MOTOR_CW_DIRECTION);
        targetCmd = -targetCmd;
    }

    // Saturation
    int8_t cmd = constrain(targetCmd, ESCON_PWM_DUTY_CYCLE_MIN, 
                            ESCON_PWM_DUTY_CYCLE_MAX);

    analogWrite(_pwmPin, cmd);
}

int16_t EsconDriver::getSpeed() {
    if (!isReady()) {
        return 0; // If not ready, return 0 speed
    }
    int adcValue = analogRead(_speedPin);
    // Map voltage back to RPM
    return static_cast<int16_t>(
        map(adcValue, 
            ESCON_ADC_MIN_VALUE, ESCON_ADC_MAX_VALUE, 
            -MOTOR_MAX_PERMISSIBLE_RPM, MOTOR_MAX_PERMISSIBLE_RPM
        )
    );
}

int16_t EsconDriver::getCurrent() {
    int adcValue = analogRead(_currPin);
    // Map ADC value back to current
    return static_cast<int16_t>(
        map(adcValue, 
            ESCON_ADC_MIN_VALUE, ESCON_ADC_MAX_VALUE, 
            -MOTOR_MAX_PERMISSIBLE_CURRENT_MA, MOTOR_MAX_PERMISSIBLE_CURRENT_MA
        )
    );
}
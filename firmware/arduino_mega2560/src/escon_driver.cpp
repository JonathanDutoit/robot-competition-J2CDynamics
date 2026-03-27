#include <escon_driver.h>

#include <Arduino.h>
#include <robot_config.h>

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

void EsconDriver::setSpeed(int16_t targetRpm) {
    if (!isReady()) {
        digitalWrite(_enPin, LOW);
        analogWrite(_pwmPin, 0);
        return;
    } else {
        digitalWrite(_enPin, HIGH);
    }

    // Direction Logic
    if (targetRpm >= 0) {
        digitalWrite(_dirPin, MOTOR_CCW_DIRECTION);
    } else {
        digitalWrite(_dirPin, MOTOR_CW_DIRECTION);
        targetRpm = -targetRpm;
    }

    // Saturation
    if (targetRpm > MOTOR_MAX_PERMISSIBLE_RPM) targetRpm = MOTOR_MAX_PERMISSIBLE_RPM;

    // Convert RPM to Duty Cycle (0-255) and write to PWM pin
    uint8_t duty = rpmToDuty(targetRpm);
    analogWrite(_pwmPin, duty);
}

uint8_t EsconDriver::rpmToDuty(int16_t rpm) {
    // Map RPM to duty cycle percentage
    float dutyCyclePercent = map(rpm, ESCON_PWM_SPEED_RPM_MIN, ESCON_PWM_SPEED_RPM_MAX, 
                                ESCON_PWM_DUTY_CYCLE_MIN, ESCON_PWM_DUTY_CYCLE_MAX);
    // Convert percentage to 0-255 range for analogWrite
    return static_cast<uint8_t>(map(dutyCyclePercent, 0.0f, 100.0f, 0, 255));
}

int16_t EsconDriver::getAveragedSpeed() {
    int adcValue = analogRead(_speedPin);
    float voltage = (adcValue / static_cast<float>(ARDUINO_ADC_MAX_VALUE)) 
                    * ARDUINO_ADC_VOLTAGE_REF;
    // Map voltage back to RPM
    return static_cast<int16_t>(
        map(voltage, 
            ESCON_ANALOG_VOLTAGE_MIN, ESCON_ANALOG_VOLTAGE_MAX, 
            ESCON_RPM_AT_VOLTAGE_MIN, ESCON_RPM_AT_VOLTAGE_MAX
        )
    );
}

int16_t EsconDriver::getAveragedCurrent() {
    int adcValue = analogRead(_currPin);
    float voltage = (adcValue / static_cast<float>(ARDUINO_ADC_MAX_VALUE)) 
                    * ARDUINO_ADC_VOLTAGE_REF;
    // Map voltage back to current
    return static_cast<int16_t>(
        map(voltage, 
            ESCON_ANALOG_VOLTAGE_MIN, ESCON_ANALOG_VOLTAGE_MAX, 
            ESCON_CURRENT_AT_VOLTAGE_MIN, ESCON_CURRENT_AT_VOLTAGE_MAX
        )
    );
}
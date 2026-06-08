#include <do2/drivers/dri0018_driver_channel.hpp>

#include <Arduino.h>
#include <do2/robot_config.hpp>
#include <common_config.hpp>

DRI0018DriverChannel::DRI0018DriverChannel(uint8_t pwmDigitalInputPin, 
                                           uint8_t directionDigitalInputPin,
                                           uint8_t currentSenseDigitalInputPin):
    _pwmPin(pwmDigitalInputPin), _dirPin(directionDigitalInputPin), 
    _currPin(currentSenseDigitalInputPin) {}

void DRI0018DriverChannel::init() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_currPin, INPUT);

    // Read the sensor multiple times to get a stable baseline ADC value
    float sum = 0;
    for (int i = 0; i < BASELINE_MEASUREMENT_COUNT; ++i) {
        sum += analogRead(_currPin);
        delay(5);
    }
    _baselineAdc = sum / BASELINE_MEASUREMENT_COUNT;

    configurePWM(); // Set PWM frequency for motor control
    digitalWrite(_dirPin, MOTOR_CCW_DIRECTION); // Default direction
    setVelocity(0); // Start with 0 speed
}

void DRI0018DriverChannel::configurePWM() {
    // For Arduino Mega 2560, we can use the default PWM frequency at 490 Hz or change
    // one of the timers' prescaler to achieve 4 kHz.
    // The Arduino Mega 2560 has several timers:
    // - Timer0: Pins 4, 13 (used for millis(), delay(), etc. - avoid changing this)
    // - Timer1: Pins 11, 12 (used for Servo library - avoid changing this if using Servo)
    // - Timer2: Pins 9, 10 (used for tone() - avoid changing this if using tone)
    // - Timer3: Pins 2, 3, 5 (available for PWM frequency change)
    // - Timer4: Pins 6, 7, 8 (available for PWM frequency change)
    if (_pwmPin == 9 || _pwmPin == 10) {
        TCCR2B &= ~0b111; // Clear prescaler bits
        TCCR2B |= ARDUINO_PWM_MOTOR_PRESCALER; // Set prescaler for Timer2 to achieve ~4 kHz PWM frequency
    }
}

uint8_t DRI0018DriverChannel::isReady() const {
    // Current sense pin outputs HIGH when a fault condition is detected (overcurrent, overtemperature, etc.)
    const int adcValue = analogRead(_currPin);
    float voltage = (adcValue / ARDUINO_ADC_MAX_COUNT) * ARDUINO_ADC_VOLTAGE_REF; // Convert ADC value to voltage (0-5V)

    // Check if the current reading is significantly above the baseline, indicating a potential fault
    if (static_cast<float>(adcValue) > 1.2f * static_cast<float>(_baselineAdc)) {
        return 0; // Fault detected
    }
    return 1; // No fault detected
}

void DRI0018DriverChannel::setVelocity(float rad_per_sec) {
    if (!isReady()) {
        analogWrite(_pwmPin, 0);
        return;
    }

    // Direction Logic
    if (rad_per_sec >= 0) {
        digitalWrite(_dirPin, MOTOR_CCW_DIRECTION);
    } else {
        digitalWrite(_dirPin, MOTOR_CW_DIRECTION);
        rad_per_sec = -rad_per_sec;
    }

    // Saturation
    if (rad_per_sec > DC_MOTOR_MAX_VELOCITY_RAD_SEC) {
        rad_per_sec = DC_MOTOR_MAX_VELOCITY_RAD_SEC;
    }

    // Normalize to [0, 1]
    rad_per_sec = rad_per_sec / DC_MOTOR_MAX_VELOCITY_RAD_SEC;

    // Convert RPM to Duty Cycle (0-255) and write to PWM pin
    uint8_t duty = static_cast<uint8_t>(rad_per_sec * DC_MOTOR_MAX_PWM_DUTY_CYCLE);
    analogWrite(_pwmPin, duty);
}

float DRI0018DriverChannel::getCurrent() {
    int adcValue = analogRead(_currPin);
    float voltage = (adcValue / ARDUINO_ADC_MAX_COUNT) * ARDUINO_ADC_VOLTAGE_REF; // Convert ADC value to voltage (0-5V)
    return voltage * 2 * DRI0018_MAX_CURRENT_A / DRI0018_MAX_OUTPUT_VOLTAGE; // in amps
}
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// -----------------------
// --- HARDWARE PINOUT ---
// -----------------------

// --- Left Motor ---
#define PIN_LEFT_MOTOR_PWM            2
#define PIN_LEFT_MOTOR_EN             3
#define PIN_LEFT_MOTOR_DIR            4
#define PIN_LEFT_MOTOR_READY          A0
#define PIN_LEFT_MOTOR_SPEED_ANA      A1
#define PIN_LEFT_MOTOR_CURR_ANA       A2

// --- Right Motor ---
#define PIN_RIGHT_MOTOR_PWM           5
#define PIN_RIGHT_MOTOR_EN            6
#define PIN_RIGHT_MOTOR_DIR           7
#define PIN_RIGHT_MOTOR_READY         A3
#define PIN_RIGHT_MOTOR_SPEED_ANA     A4
#define PIN_RIGHT_MOTOR_CURR_ANA      A5


// ------------------------------
// ---------- ARDUINO -----------
// ------------------------------

// --- ADC properties ---
#define ARDUINO_ADC_MAX_COUNT           1023.0f // 1023 for 10-bit ADC
#define ARDUINO_ADC_VOLTAGE_REF         5.0f // in volts (Arduino ADC reference voltage)

// --- PWM properties ---
#define ARDUINO_PWM_MAX_COUNT           255.0f // 255 for 8-bit PWM resolution

// --- PWM configuration ---
#define ARDUINO_PWM_MOTOR_PRESCALER     0b010 // For ~4 kHz PWM frequency on Arduino Mega 2560 (Timer4)


// --------------------------------
// --- MOTOR & ESCON PARAMETERS ---
// --------------------------------

// --- Motor limits ---
#define MOTOR_MAX_RPM                   4092
#define MOTOR_MAX_VELOCITY_RAD_SEC      (float)(MOTOR_MAX_RPM * 2.0f * PI / 60.0f) // in radians per second
#define MOTOR_MAX_CURRENT_A             3.06f // in amps

// --- Motor directions ---
#define MOTOR_CCW_DIRECTION             0
#define MOTOR_CW_DIRECTION              1

// --- PWM parameters ---
#define ESCON_PWM_DUTY_CYCLE_MIN       (int)(10.0f / 100.0f * ARDUINO_PWM_MAX_COUNT) // Minimum duty cycle to overcome motor deadzone (10% of 255)
#define ESCON_PWM_DUTY_CYCLE_MAX       (int)(90.0f / 100.0f * ARDUINO_PWM_MAX_COUNT) // Maximum duty cycle to avoid overdriving the motor (90% of 255)

// --- Analog output configuration ---
// Arduino ADC read 0-5V, so ESCON MUST output 0-4V only
#define ESCON_MAX_OUTPUT_VOLTAGE       4.0f // in volts (4V max output from ESCON)
#define ESCON_CURRENT_ZERO_VOLTAGE     (ESCON_MAX_OUTPUT_VOLTAGE) / 2.0f // in volts (2V offset for centering around 0 current/speed)
#define ESCON_VELOCITY_ZERO_VOLTAGE    (ESCON_MAX_OUTPUT_VOLTAGE) / 2.0f // in volts (2V offset for centering around 0 current/speed)

#endif
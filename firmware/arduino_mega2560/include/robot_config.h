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
#define ARDUINO_ADC_RESOLUTION_BITS     10
#define ARDUINO_ADC_MAX_VALUE           ((1 << ARDUINO_ADC_RESOLUTION_BITS) - 1) // 1023 for 10-bit ADC
#define ARDUINO_ADC_VOLTAGE_REF         5000 // in millivolts (5V reference voltage)

// --- PWM configuration ---
#define ARDUINO_PWM_MOTOR_PRESCALER     0b010 // For ~4 kHz PWM frequency on Arduino Mega 2560 (Timer4)


// --------------------------------
// --- MOTOR & ESCON PARAMETERS ---
// --------------------------------

// --- Motor limits ---
#define MOTOR_MAX_PERMISSIBLE_RPM       4092
#define MOTOR_MIN_DEADZONE_RPM          200

// --- Motor directions ---
#define MOTOR_CCW_DIRECTION             0
#define MOTOR_CW_DIRECTION              1

// --- PWM parameters ---
#define ESCON_PWM_DUTY_CYCLE_MIN       10.0f
#define ESCON_PWM_DUTY_CYCLE_MAX       90.0f
#define ESCON_PWM_SPEED_RPM_MIN         0.0f
#define ESCON_PWM_SPEED_RPM_MAX        MOTOR_MAX_PERMISSIBLE_RPM

// --- Analog output configuration ---
// Arduino ADC read 0-5V, so ESCON MUST output 0-4V only
#define ESCON_ANALOG_VOLTAGE_MAX        4000 // in millivolts (4V max output from ESCON)
#define ESCON_ANALOG_VOLTAGE_MIN        0 // in millivolts (0V min output from ESCON)

#define ESCON_CURRENT_AT_VOLTAGE_MAX    3060 // in milliamps (3.06A from ESCON)
#define ESCON_CURRENT_AT_VOLTAGE_MIN    -3060 // in milliamps (-3.06A from ESCON)

#define ESCON_RPM_AT_VOLTAGE_MAX        4092
#define ESCON_RPM_AT_VOLTAGE_MIN        -4092

// Maximum and minimum Arduino ADC values corresponding to ESCON's voltage outputs
#define ESCON_ADC_MAX_VALUE   (int)(ARDUINO_ADC_MAX_VALUE * (ESCON_ANALOG_VOLTAGE_MAX / ARDUINO_ADC_VOLTAGE_REF))
#define ESCON_ADC_MIN_VALUE   (int)(ARDUINO_ADC_MAX_VALUE * (ESCON_ANALOG_VOLTAGE_MIN / ARDUINO_ADC_VOLTAGE_REF))


// ---------------------
// --- COMMUNICATION ---
// ---------------------

#define SERIAL_BAUD_RATE        921600
#define CMD_TIMEOUT_MS          200
#define TELEMETRY_FREQ_HZ       50

// -------------------------
// --- ODOMETRY GEOMETRY ---
// -------------------------

#define WHEEL_DIAMETER_METERS   0.065f  // 65mm DUPLO wheel approx
#define WHEEL_BASE_METERS       0.150f  // Distance between wheel centers

#endif
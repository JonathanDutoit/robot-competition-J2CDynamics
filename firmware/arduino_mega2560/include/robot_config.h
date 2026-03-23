#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// -----------------------
// --- HARDWARE PINOUT ---
// -----------------------

// --- Left Motor ---
#define PIN_LEFT_PWM            4
#define PIN_LEFT_EN       
#define PIN_LEFT_DIR      
#define PIN_LEFT_READY    
#define PIN_LEFT_SPEED_ANA 
#define PIN_LEFT_CURR_ANA 

// --- Right Motor ---
#define PIN_RIGHT_PWM           13
#define PIN_RIGHT_EN      
#define PIN_RIGHT_DIR     
#define PIN_RIGHT_READY   
#define PIN_RIGHT_SPEED_ANA 
#define PIN_RIGHT_CURR_ANA


// ------------------------------
// --- ARDUINO ADC PROPERTIES ---
// ------------------------------

#define ARDUINO_ADC_RESOLUTION_BITS     10
#define ARDUINO_ADC_MAX_VALUE           ((1 << ARDUINO_ADC_RESOLUTION_BITS) - 1) // 1023 for 10-bit ADC
#define ARDUINO_ADC_VOLTAGE_REF          5.0f


// --------------------------------
// --- MOTOR & ESCON PARAMETERS ---
// --------------------------------

// --- Motor limits ---
#define MOTOR_MAX_PERMISSIBLE_RPM    4092.0f
#define MOTOR_MIN_DEADZONE_RPM        500.0f

// --- Motor directions ---
#define MOTOR_CW_DIRECTION    1
#define MOTOR_CCW_DIRECTION   0

// --- PWM parameters ---
#define ESCON_PWM_FREQUENCY_HZ       5000.0f
#define ESCON_PWM_DUTY_CYCLE_MIN       10.0f
#define ESCON_PWM_DUTY_CYCLE_MAX       90.0f
#define ESCON_PWM_SPEED_RPM_MIN         0.0f
#define ESCON_PWM_SPEED_RPM_MAX         MOTOR_MAX_PERMISSIBLE_RPM

// --- Analog output configuration ---
// Arduino ADC read 0-5V, so ESCON MUST output 0-4V only
#define ESCON_ANALOG_VOLTAGE_MAX        4.0f
#define ESCON_ANALOG_VOLTAGE_MIN        0.0f

#define ESCON_CURRENT_AT_VOLTAGE_MAX    3.06f
#define ESCON_CURRENT_AT_VOLTAGE_MIN   -3.06f

#define ESCON_RPM_AT_VOLTAGE_MAX     4092.0f
#define ESCON_RPM_AT_VOLTAGE_MIN    -4092.0f

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
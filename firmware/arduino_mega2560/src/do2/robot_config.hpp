#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// ---------------------------
// --- MAXON CONFIGURATION ---
// ---------------------------

// --- Mechanical properties ---
#define MAXON_GEAR_RATIO            18.0f

// -----------------------
// --- HARDWARE PINOUT ---
// -----------------------

// --- Right Sweeper Motor ---
#define PIN_RIGHT_SWEEPER_PWM            9
#define PIN_RIGHT_SWEEPER_DIR            8
#define PIN_RIGHT_SWEEPER_CURR_SENSE     24 // Orange cable

// --- Left Sweeper Motor ---
#define PIN_LEFT_SWEEPER_PWM           10
#define PIN_LEFT_SWEEPER_DIR           11
#define PIN_LEFT_SWEEPER_CURR_SENSE    25

// --- Stepper Motor ---
#define PIN_STEPPER_DIR                 30
#define PIN_STEPPER_STEP                32
#define PIN_STEPPER_ENABLE              34

// --------------------------------
// --- DC MOTOR PARAMETERS ---
// --------------------------------

// --- Mechanical properties ---
#define DC_MOTOR_GEAR_RATIO             75.0f // gear ratio

// --- Motor limits ---
#define DC_MOTOR_MAX_RPM                (int)(10000.0f / DC_MOTOR_GEAR_RATIO) // in RPM
#define DC_MOTOR_MAX_VELOCITY_RAD_SEC   (float)(DC_MOTOR_MAX_RPM * 2.0f * PI / 60.0f) // in radians per second
#define DC_MOTOR_STALL_CURRENT_A        6.0f // in amps

#define DC_MOTOR_MAX_VOLTAGE_V          6.0f // in volts
#define DC_MOTOR_MAX_PWM_DUTY_CYCLE     (int)(ARDUINO_PWM_MAX_COUNT * DC_MOTOR_MAX_VOLTAGE_V / NOMINAL_BATTERY_VOLTAGE_V) // Max duty cycle for acceptable voltage to the motor (4-8V depending on battery level)

// --- Driver limits ---
#define DRI0018_MAX_CURRENT_A           15.0f // in amps
#define DRI0018_MAX_OUTPUT_VOLTAGE      5.0f // in volts

// --- Motor cruise speed ---
#define DC_MOTOR_CRUISE_VEL_RAD_SEC     11.0f // in rad/s (maintain ~6V to the motor)

// --------------------------------
// --- STEPPER MOTOR PARAMETERS ---
// --------------------------------

// --- Stepper motor parameters ---
#define STEPPER_DROPOFF_VEL_STEPS_SEC     500.0f // in steps/s

#endif
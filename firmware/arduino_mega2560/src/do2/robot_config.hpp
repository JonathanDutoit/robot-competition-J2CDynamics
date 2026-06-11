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

// --- Left Sweeper Motor ---
#define PIN_LEFT_SWEEPER_PWM           10
#define PIN_LEFT_SWEEPER_DIR           11

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
#define STEP_PER_REV                        200 // for a 1.8 degree stepper
#define STEPPER_MICROSTEPPING               2 // microstepping factor (e.g., 16 for 1/16 microstepping)
#define STEPPER_EFFECTIVE_STEPS_PER_REV     (STEP_PER_REV * STEPPER_MICROSTEPPING) // effective steps per revolution considering microstepping
#define QUARTER_TURN_STEPS                  (STEPPER_EFFECTIVE_STEPS_PER_REV / 6) // steps for a quarter turn

#define STEPPER_MAX_SPEED_STEPS_PER_SEC     100.0f // in steps per second
#define DROP_OFF_SPEED                      10.0f // in steps per second (same as max speed for continuous rotation)

#define STEPPER_ACCELERATION_STEPS_PER_SEC2 25.0f

// --------------------------
// --- MISSION PARAMETERS ---
// --------------------------
#define MAX_DUPLO_COUNT                     6
#define COLLECTING_JAM_THRESHOLD_MS         500 // 0.5 seconds
#define DROP_OFF_JAM_THRESHOLD_MS           1500 // 1.5 seconds
#define UNJAM_STEP_DELAY_MS                 300 // Delay between unjam steps

#endif
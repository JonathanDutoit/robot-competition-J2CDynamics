/**
 * TODO: Add description
 */
#include <Arduino.h>
#include <robot_config.hpp>
#include <common/drivers/escon_driver.hpp>
#include <common/controllers/differential_drive_controller.hpp>

EsconDriver leftMotor(PIN_LEFT_MOTOR_PWM, PIN_LEFT_MOTOR_EN, PIN_LEFT_MOTOR_DIR, 
                        PIN_LEFT_MOTOR_READY, PIN_LEFT_MOTOR_SPEED_ANA, 
                        PIN_LEFT_MOTOR_CURR_ANA);
EsconDriver rightMotor(PIN_RIGHT_MOTOR_PWM, PIN_RIGHT_MOTOR_EN, PIN_RIGHT_MOTOR_DIR, 
                        PIN_RIGHT_MOTOR_READY, PIN_RIGHT_MOTOR_SPEED_ANA, 
                        PIN_RIGHT_MOTOR_CURR_ANA);

DifferentialDriveController driveController(&leftMotor, &rightMotor); // Example wheel diameter and gear ratio

struct Measurement {
    float leftWheelRadPerSec;
    float rightWheelRadPerSec;
    float leftMotorCurrentA;
    float rightMotorCurrentA;
};
Measurement measurement;

void set_speed_command(float leftPWM, float rightPWM);
void get_odometry();
bool parseInput(String input, char* command, float& leftPWM, float& rightPWM, int& parsedCount);

void setup()
{
  //initialize the serial port
  Serial.begin(9600);

  // initialize Escon motor drivers
  leftMotor.init();
  rightMotor.init();
}

void loop()
{
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    char command[10];
    float leftPWM = 0, rightPWM = 0;
    int parsedCount = 0;

    bool valid = parseInput(input, command, leftPWM, rightPWM, parsedCount);

    if (!valid) {
      return;
    }

    // ── SPEED command ─────────────────────────────
    if (strcmp(command, "SPEED") == 0) {
      if (parsedCount == 3) {
        set_speed_command(leftPWM, rightPWM);
    }

    // ── ODOMETRY command ────────────────────────────
    } else if (strcmp(command, "ODOMETRY") == 0) {
      get_odometry();
    }
  }
}

void set_speed_command(float leftPWM, float rightPWM) {
  driveController.setWheelVelocities(leftPWM, rightPWM);
}

void get_odometry() {
  float leftVel = 0, rightVel = 0;

  driveController.getWheelVelocities(leftVel, rightVel);

  Serial.print("ODOMETRY: ");
  Serial.print("L_vel=");
  Serial.print(leftVel, 2);
  Serial.print(" rad/s ");

  Serial.print("R_vel=");
  Serial.print(rightVel, 2);
  Serial.println(" rad/s ");
}

bool parseInput(String input, char* command, float& leftPWM, float& rightPWM, int& parsedCount) {
  input.trim();

  char buffer[64];
  input.toCharArray(buffer, sizeof(buffer));

  parsedCount = 0;

  // First token (command)
  char* token = strtok(buffer, " \r\n");
  if (!token) return false;

  strncpy(command, token, 10);  // copy safely
  command[9] = '\0';            // ensure null termination
  parsedCount++;

  // leftPWM
  char* arg1 = strtok(nullptr, " \r\n");
  if (arg1) {
    leftPWM = atof(arg1);
    parsedCount++;
  }

  // rightPWM
  char* arg2 = strtok(nullptr, " \r\n");
  if (arg2) {
    rightPWM = atof(arg2);
    parsedCount++;
  }

  return true;
}
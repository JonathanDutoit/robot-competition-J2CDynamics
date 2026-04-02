/**
 * TODO: Add description
 */
#include <Arduino.h>
#include <common/robot_config.hpp>
#include <common/drivers/escon_driver.hpp>
#include <main/differential_drive_controller.hpp>

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
void status_command();
bool parseInput(String input, char* command, float& leftPWM, float& rightPWM, int& parsedCount);

void setup()
{
  //initialize the serial port
  Serial.begin(9600);

  // initialize Escon motor drivers
  leftMotor.init();
  rightMotor.init();

  Serial.println("Motor ready! Waiting for new commands...");
  while (!Serial);

  Serial.println("Enter a word and press Enter:");
}

void loop()
{
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    Serial.print("Received: ");
    Serial.println(input);

    char command[10];
    float leftPWM = 0, rightPWM = 0;
    int parsedCount = 0;

    bool valid = parseInput(input, command, leftPWM, rightPWM, parsedCount);

    if (!valid) {
      Serial.println("ERR: Empty command");
      return;
    }

    // ── SPEED command ─────────────────────────────
    if (strcmp(command, "SPEED") == 0) {
      if (parsedCount == 3) {
        set_speed_command(leftPWM, rightPWM);
      } else {
        Serial.println("ERR: Usage -> SPEED <left> <right>");
      }
    }

    // ── STATUS command ────────────────────────────
    else if (strcmp(command, "STATUS") == 0) {
      status_command();
    }

    // ── Unknown command ───────────────────────────
    else {
      Serial.println("ERR: Unknown command");
    }
  }
}

void set_speed_command(float leftPWM, float rightPWM) {
  driveController.setWheelVelocities(leftPWM, rightPWM);
  Serial.print("OK: L=");
  Serial.print(leftPWM);
  Serial.print(" R=");
  Serial.println(rightPWM);
}

void status_command() {
  float leftVel = 0, rightVel = 0;

  driveController.getWheelVelocities(leftVel, rightVel);

  float leftCur  = leftMotor.getCurrent();
  float rightCur = rightMotor.getCurrent();

  Serial.print("STATUS: ");
  Serial.print("L_vel=");
  Serial.print(leftVel, 2);
  Serial.print(" rad/s ");

  Serial.print("R_vel=");
  Serial.print(rightVel, 2);
  Serial.print(" rad/s ");

  Serial.print("L_cur=");
  Serial.print(leftCur, 2);
  Serial.print(" A ");

  Serial.print("R_cur=");
  Serial.print(rightCur, 2);
  Serial.println(" A");
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
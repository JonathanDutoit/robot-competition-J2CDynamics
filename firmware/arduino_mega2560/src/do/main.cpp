/**
 * TODO: Add description
 */
#include <Arduino.h>
#include <common_config.hpp>
#include <common/drivers/escon_driver.hpp>
#include <common/controllers/differential_drive_controller.hpp>
#include <common/sensors/duplo_counter.hpp>
#include <common/scheduler/periodic_task.hpp>

EsconDriver leftMotor(PIN_LEFT_MAXON_PWM, PIN_LEFT_MAXON_EN, PIN_LEFT_MAXON_DIR, 
                        PIN_LEFT_MAXON_READY, PIN_LEFT_MAXON_SPEED_ANA, 
                        PIN_LEFT_MAXON_CURR_ANA);
EsconDriver rightMotor(PIN_RIGHT_MAXON_PWM, PIN_RIGHT_MAXON_EN, PIN_RIGHT_MAXON_DIR, 
                        PIN_RIGHT_MAXON_READY, PIN_RIGHT_MAXON_SPEED_ANA, 
                        PIN_RIGHT_MAXON_CURR_ANA);
DifferentialDriveController driveController(&leftMotor, &rightMotor); // Example wheel diameter and gear ratio

DuploCounter duploCounter(PIN_DUPLO_IR_SENSOR);

// Define tasks schedule
PeriodicTask duploTask(20);

void process_serial_input();
void set_speed_command(float leftPWM, float rightPWM);
void get_odometry();
void get_duplo_count();
bool parseInput(String input, char* command, float& leftPWM, float& rightPWM, int& parsedCount);

void setup()
{
  //initialize the serial port
  Serial.begin(115200);

  // initialize Escon motor drivers
  leftMotor.init();
  rightMotor.init();

  // initialize sensors
  duploCounter.init();
}

void loop()
{
  if (duploTask.ready()){
      duploCounter.update();
  }

  process_serial_input();
}

void process_serial_input() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    char command[12];
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
    } 
    
    // ── ODOMETRY command ────────────────────────────
    else if (strcmp(command, "ODOMETRY") == 0) {
      get_odometry();
    }

    // ── DUPLO COUNT command ─────────────────────────────
    else if (strcmp(command, "DUPLO_COUNT") == 0) {
      get_duplo_count();
    }

    // ── UNKNOWN command ─────────────────────────────
    else {
      Serial.print("ERROR: Unknown command '");
      Serial.print(command);
      Serial.println("'");
    }
  }
}

void set_speed_command(float leftPWM, float rightPWM) {
  driveController.setVelocities(leftPWM, rightPWM);
}

void get_odometry() {
  float leftVel = 0, rightVel = 0;

  driveController.getVelocities(leftVel, rightVel);

  Serial.print("ODOMETRY: ");
  Serial.print("L_vel=");
  Serial.print(leftVel, 2);
  Serial.print(" rad/s ");

  Serial.print("R_vel=");
  Serial.print(rightVel, 2);
  Serial.println(" rad/s ");
}

void get_duplo_count() {
  uint8_t count = duploCounter.getCount();

  Serial.print("DUPLO_COUNT: ");
  Serial.println(count);
}

bool parseInput(String input, char* command, float& leftPWM, float& rightPWM, int& parsedCount) {
  input.trim();

  char buffer[64];
  input.toCharArray(buffer, sizeof(buffer));

  parsedCount = 0;

  // First token (command)
  char* token = strtok(buffer, " \r\n");
  if (!token) return false;

  strncpy(command, token, 12);  // copy safely
  command[11] = '\0';            // ensure null termination
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
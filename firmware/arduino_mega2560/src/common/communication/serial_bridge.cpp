#include <common/communication/serial_bridge.hpp>
#include <common_config.hpp>

void SerialBridge::init() {
    //initialize the serial port
    Serial.begin(SERIAL_BAUD_RATE);
}

void SerialBridge::update() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        char command[12];
        int parsedCount = 0;
        float args[2] = {0}; // For commands that require up to 2 float arguments

        bool valid = _parseInput(input, command, args, parsedCount);

        if (!valid) {
            return;
        }

        // ── SPEED command ─────────────────────────────
        if (strcmp(command, "SPEED") == 0) {
        if (parsedCount == 3) {
            set_speed_command(args[0], args[1]);
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

bool SerialBridge::_parseInput(
    String input, char* command, float (&args)[2], int& parsedCount
) {
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

  // arg1
  char* arg1 = strtok(nullptr, " \r\n");
  if (arg1) {
    args[0] = atof(arg1);
    parsedCount++;
  }

  // arg2
  char* arg2 = strtok(nullptr, " \r\n");
  if (arg2) {
    args[1] = atof(arg2);
    parsedCount++;
  }

  return true;
}
#include <common/communication/serial_bridge.hpp>
#include <common_config.hpp>

SerialBridge::SerialBridge(RobotCommand& cmd, RobotState& state):
_cmd(cmd), _state(state) {}

void SerialBridge::init() {
    //initialize the serial port
    Serial.begin(SERIAL_BAUD_RATE);
}

void SerialBridge::update() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        char command[12];
        int parsedCount = 0;
        float arg_left, arg_right; // For commands that require up to 2 float arguments

        bool valid = _parseInput(input, command, arg_left, arg_right, parsedCount);

        if (!valid) {
            return;
        }

        // ── SPEED command ─────────────────────────────
        if (strcmp(command, "SPEED") == 0) {
            if (parsedCount == 3) {
                _setNextSpeedCommand(arg_left, arg_right);
            }
        } 
        
        // ── ODOMETRY command ────────────────────────────
        else if (strcmp(command, "ODOMETRY") == 0) {
            _sendOdometry();
        }

        // ── DUPLO COUNT command ─────────────────────────────
        else if (strcmp(command, "DUPLO_COUNT") == 0) {
            _sendDuploCountCommand();
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
    String input, char* command, float& arg_left, float& arg_right, int& parsedCount
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
        arg_left = atof(arg1);
        parsedCount++;
    }

    // arg2
    char* arg2 = strtok(nullptr, " \r\n");
    if (arg2) {
        arg_right = atof(arg2);
        parsedCount++;
    }

    return true;
}

void SerialBridge::_setNextSpeedCommand(float leftVel, float rightVel) {
    _cmd.leftWheelVelocity = leftVel;
    _cmd.rightWheelVelocity = rightVel;
}

void SerialBridge::_sendOdometry() {
    Serial.print("ODOMETRY: ");
    Serial.print("L_vel=");
    Serial.print(_state.leftWheelVelocity, 2);
    Serial.print(" rad/s ");
    Serial.print("R_vel=");
    Serial.print(_state.rightWheelVelocity, 2);
    Serial.println(" rad/s ");
}

void SerialBridge::_sendDuploCountCommand() {
    Serial.print("DUPLO_COUNT: ");
    Serial.println(_state.duploCount);
}
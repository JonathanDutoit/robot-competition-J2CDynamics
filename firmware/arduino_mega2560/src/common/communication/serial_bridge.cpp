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
        float args[2]; // For commands that require up to 2 float arguments

        bool valid = _parseInput(input, command, args, parsedCount);

        if (!valid) {
            return;
        }

        _processCommand(command, args, parsedCount);
    }
}

bool SerialBridge::registerHandler(ICommandHandler* handler)
{
    if (_handlerCount >= MAX_COMMAND_HANDLERS) {
        return false; // Cannot register more handlers
    }
    _handlers[_handlerCount++] = handler;
    return true;
}

bool SerialBridge::_parseInput(
    String input, char* command, float* args, int& parsedCount
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

bool SerialBridge::_processCommand(
    const char* command, const float* args, const int& parsedCount
) {
    // ── SPEED command ─────────────────────────────
    if (strcmp(command, "SPEED") == 0) {
        if (parsedCount == 3) {
            _setNextSpeedCommand(args[0], args[1]);
            return true;
        }
        return false;
    } 
    
    // ── ODOMETRY command ────────────────────────────
    else if (strcmp(command, "ODOMETRY") == 0) {
        _sendOdometry();
        return true;
    }

    // ── DUPLO COUNT command ─────────────────────────────
    else if (strcmp(command, "DUPLO_COUNT") == 0) {
        _sendDuploCountCommand();
        return true;
    }

    for (uint8_t i = 0; i < _handlerCount; ++i) {
        HandlerResponse response = _handlers[i]->handleCommand(command, args);
        if (response.success) {
            if (response.message.length() > 0) {
                Serial.println(response.message);
            }
            return true;
        } else {
            Serial.print("ERROR: Handler ");
            Serial.print(i);
            Serial.print(" failed to process command '");
            Serial.print(command);
            Serial.println("'");
            Serial.print("-> Error Response message: ");
            Serial.println(response.message);
        }
    }

    return false;
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
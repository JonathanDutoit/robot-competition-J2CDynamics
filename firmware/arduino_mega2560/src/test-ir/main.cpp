#include <Arduino.h>
#include <common_config.hpp>
#include <common/sensors/duplo_counter.hpp>

DuploCounter duploCounter(PIN_DUPLO_IR_SENSOR);

static constexpr uint32_t UPDATE_PERIOD_MS = 20;   // 50 Hz (matches robot loop)
static constexpr uint32_t PRINT_PERIOD_MS  = 100;  // 10 Hz logging

uint32_t lastUpdateTime = 0;
uint32_t lastPrintTime  = 0;

// =======================
// SERIAL BUFFER (non-blocking optional reset)
// =======================
char buffer[32];
uint8_t idx = 0;

// =======================
// NON-BLOCKING SERIAL INPUT
// =======================
void readSerial()
{
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            buffer[idx] = '\0';

            if (strcmp(buffer, "RESET") == 0)
            {
                duploCounter.reset();
                Serial.println("OK RESET");
            }

            idx = 0;
        }
        else
        {
            if (idx < sizeof(buffer) - 1)
            {
                buffer[idx++] = c;
            }
        }
    }
}

// =======================
// SETUP
// =======================
void setup()
{
    Serial.begin(9600);

    duploCounter.init();

    Serial.println("DuploCounter test bench started");
}

// =======================
// LOOP (FULLY NON-BLOCKING)
// =======================
void loop()
{
    uint32_t now = millis();

    // -----------------------
    // SENSOR UPDATE (50 Hz)
    // -----------------------
    if (now - lastUpdateTime >= UPDATE_PERIOD_MS)
    {
        lastUpdateTime = now;

        duploCounter.update();
    }

    // -----------------------
    // SERIAL INPUT
    // -----------------------
    readSerial();

    // -----------------------
    // DEBUG OUTPUT (10 Hz)
    // -----------------------
    if (now - lastPrintTime >= PRINT_PERIOD_MS)
    {
        lastPrintTime = now;

        Serial.print("COUNT=");
        Serial.println(duploCounter.getCount());
    }
}
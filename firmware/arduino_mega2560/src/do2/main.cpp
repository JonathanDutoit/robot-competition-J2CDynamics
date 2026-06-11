/**
 * TODO: Add description
 */
#include <Arduino.h>
#include <do2/robot_config.hpp>
#include <do2/drivers/dri0018_driver_channel.hpp>
#include <do2/communication/sweeper_command_handler.hpp>
#include <do2/controllers/sweeper_controller.hpp>
#include <do2/data/sweeper_state.hpp>
#include <common_config.hpp>
#include <common/drivers/escon_driver.hpp>
#include <common/controllers/differential_drive_controller.hpp>
#include <common/sensors/duplo_counter.hpp>
#include <common/scheduler/periodic_task.hpp>
#include <common/communication/serial_bridge.hpp>
#include <common/data/robot_command.hpp>
#include <common/data/robot_state.hpp>
#include <do2/controllers/plate_controller.hpp>
#include <do2/communication/reset_command_handler.hpp>

// Global instances
SweeperState sweeperState;
SweeperMode previousMode = SweeperMode::Idle;
RobotCommand cmd;
RobotState robotState;
uint8_t previousDuploCount = 0;


SerialBridge serialBridge(cmd, robotState);
SweeperCommandHandler sweeperHandler(sweeperState);
ResetCommandHandler resetHandler;

EsconDriver leftMotor(PIN_LEFT_MAXON_PWM, PIN_LEFT_MAXON_EN, PIN_LEFT_MAXON_DIR, 
                        PIN_LEFT_MAXON_READY, PIN_LEFT_MAXON_SPEED_ANA, 
                        PIN_LEFT_MAXON_CURR_ANA);
EsconDriver rightMotor(PIN_RIGHT_MAXON_PWM, PIN_RIGHT_MAXON_EN, PIN_RIGHT_MAXON_DIR, 
                        PIN_RIGHT_MAXON_READY, PIN_RIGHT_MAXON_SPEED_ANA, 
                        PIN_RIGHT_MAXON_CURR_ANA);
DifferentialDriveController driveController(&leftMotor, &rightMotor, cmd, robotState); // Example wheel diameter and gear ratio

DuploCounter duploCounter(PIN_DUPLO_IR_SENSOR);

DRI0018DriverChannel leftSweeper(PIN_LEFT_SWEEPER_PWM, PIN_LEFT_SWEEPER_DIR);
DRI0018DriverChannel rightSweeper(PIN_RIGHT_SWEEPER_PWM, PIN_RIGHT_SWEEPER_DIR);
SweeperController sweepersController(&leftSweeper, &rightSweeper, sweeperState, duploCounter);

PlateController plateController(sweeperState, robotState);

// Define tasks schedule
PeriodicTask controlTask(20); // 20 ms period for control loop (50 Hz)
PeriodicTask duploTask(100); // 100 ms period for duplo counter update (10 Hz)
PeriodicTask sweepersTask(20); // 50 ms period for sweepers control loop (50 Hz)
PeriodicTask plateTurningTask(0.1); // 0.1 ms period for plate control loop (10000 Hz)

unsigned long lastDuploOutTime = 0;
bool unjamInProgress = false;
uint8_t unjamStep = 0;

void setup()
{
    // Initialize components
    leftMotor.init();
    rightMotor.init();
    driveController.init();

    leftSweeper.init();
    rightSweeper.init();
    sweepersController.init();

    plateController.init();

    duploCounter.init();

    serialBridge.init();
    if (!serialBridge.registerHandler(&sweeperHandler)) {
        Serial.println("ERROR: Failed to register SWEEPER command handler");
    }
    if (!serialBridge.registerHandler(&resetHandler)) {
        Serial.println("ERROR: Failed to register RESET command handler");
    }

    Serial.println("Robot initialized");
}

void loop()
{
    // Update tasks
    if (controlTask.ready()){
        driveController.update();
    }

    if (duploTask.ready()){
        duploCounter.update();
        uint8_t currentCount = duploCounter.getCount();

        if (currentCount > previousDuploCount) {
                lastDuploOutTime = millis();   // Reset the timer when a new duplo is detected

                if (currentCount < MAX_DUPLO_COUNT) {
                    plateController.rotateQuarterTurn(); // Rotate the plate by a quarter turn for each new duplo detected
                }
        }
        
        previousDuploCount = currentCount;
        robotState.duploCount = currentCount;
    }

    if (sweepersTask.ready()){
        sweepersController.update();
    }

    if (plateTurningTask.ready()){
        plateController.update();
    }

    serialBridge.update();


    // FSM logic
    if (sweeperState.mode == SweeperMode::Dropoff &&
        previousMode != SweeperMode::Dropoff) {
        duploCounter.reset();
    } else if (sweeperState.mode != SweeperMode::Dropoff &&
        previousMode == SweeperMode::Dropoff) {
        duploCounter.reset();
    }
    previousMode = sweeperState.mode;
}